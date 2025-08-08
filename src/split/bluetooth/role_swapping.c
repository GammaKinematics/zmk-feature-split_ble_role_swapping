#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>
#include <zmk/battery.h>
#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/split/central.h>
#include <zmk/split/transport/central.h>

#include <zmk_feature_split_ble_role_swapping/split/bluetooth/role_swapping.h>

LOG_MODULE_REGISTER(zmk_role_swapping, CONFIG_ZMK_LOG_LEVEL);

// External init functions from ZMK core - no headers needed
extern int zmk_split_bt_central_init(void);
extern int zmk_peripheral_ble_init(void);

#define ROLE_SWAP_MAGIC 0xAB
#define RST_TIMEOUT_MS 2000
#define PERIPHERAL_RESET_DELAY_MS 100
#define SETTINGS_KEY_IS_CENTRAL "ble_role_swap/is_central"
#define SETTINGS_KEY_CENTRAL_PROFILE "ble_role_swap/central_profile"
#define SETTINGS_KEY_PERIPHERAL_PROFILE "ble_role_swap/peripheral_profile"

// Current role state
static bool current_role_is_central = false;

// Settings storage
static bool is_central = false;
static uint8_t central_profile_index = 0;
static uint8_t peripheral_profile_index = 0;

// Role switch coordination state (central only)
static bool waiting_for_ack = false;
static struct k_work_delayable rst_timeout_work;

// Forward declarations
static int load_settings(void);
static int save_current_role_settings(bool new_is_central);
static int initiate_role_switch(void);
static void rst_timeout_handler(struct k_work *work);
static int handle_role_switch_ack(uint8_t magic);
static int handle_role_switch_rst(uint8_t magic);

/**
 * Determine role for first boot when no settings exist
 */
static bool get_default_role(void) {
    return IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_SWAPPING_CENTRAL_DEFAULT);
}

/**
 * Load settings from flash storage
 */
static int load_settings(void) {
    int ret;
    size_t len;

    // Load is_central setting
    len = sizeof(is_central);
    ret = settings_load_binary(SETTINGS_KEY_IS_CENTRAL, &is_central, &len);
    if (ret != 0 || len != sizeof(is_central)) {
        LOG_INF("No role settings found, using default");
        is_central = get_default_role();
    }

    // Load central profile index
    len = sizeof(central_profile_index);
    ret = settings_load_binary(SETTINGS_KEY_CENTRAL_PROFILE, &central_profile_index, &len);
    if (ret != 0 || len != sizeof(central_profile_index)) {
        central_profile_index = 0; // Default to profile 0
    }

    // Load peripheral profile index
    len = sizeof(peripheral_profile_index);
    ret = settings_load_binary(SETTINGS_KEY_PERIPHERAL_PROFILE, &peripheral_profile_index, &len);
    if (ret != 0 || len != sizeof(peripheral_profile_index)) {
        peripheral_profile_index = 0; // Default to profile 0
    }

    LOG_INF("Loaded settings: is_central=%d, central_profile=%d, peripheral_profile=%d",
            is_central, central_profile_index, peripheral_profile_index);

    return 0;
}

/**
 * Save current settings to flash storage
 */
static int save_current_role_settings(bool new_is_central) {
    int ret;
    
    // Save current active profile to appropriate role setting
    uint8_t active_profile = zmk_ble_active_profile_index();
    
    if (current_role_is_central) {
        // Currently central, save current profile as central profile
        ret = settings_save_one(SETTINGS_KEY_CENTRAL_PROFILE, &active_profile, sizeof(active_profile));
        if (ret) {
            LOG_ERR("Failed to save central profile: %d", ret);
            return ret;
        }
        central_profile_index = active_profile;
    } else {
        // Currently peripheral, save current profile as peripheral profile
        ret = settings_save_one(SETTINGS_KEY_PERIPHERAL_PROFILE, &active_profile, sizeof(active_profile));
        if (ret) {
            LOG_ERR("Failed to save peripheral profile: %d", ret);
            return ret;
        }
        peripheral_profile_index = active_profile;
    }

    // Save new role
    ret = settings_save_one(SETTINGS_KEY_IS_CENTRAL, &new_is_central, sizeof(new_is_central));
    if (ret) {
        LOG_ERR("Failed to save is_central: %d", ret);
        return ret;
    }

    is_central = new_is_central;
    LOG_INF("Saved role switch: new_is_central=%d", new_is_central);
    
    return 0;
}

/**
 * Activity state listener - triggers role switch evaluation on idle
 */
static int activity_state_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *activity_ev = as_zmk_activity_state_changed(eh);
    
    if (!activity_ev || activity_ev->state != ZMK_ACTIVITY_IDLE) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Only central evaluates role switches
    if (!current_role_is_central) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Don't start new role switch if one is in progress
    if (waiting_for_ack) {
        LOG_DBG("Role switch in progress - skipping evaluation");
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("Activity idle - evaluating role switch");

    // Get local battery level
    uint8_t local_battery = zmk_battery_state_of_charge();
    
    // Get peripheral battery level
    uint8_t remote_battery;
    int ret = zmk_split_central_get_peripheral_battery_level(0, &remote_battery);
    if (ret) {
        LOG_DBG("Cannot get peripheral battery level - skipping role switch evaluation");
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Calculate battery difference
    int battery_diff = (int)remote_battery - (int)local_battery;
    
    LOG_DBG("Battery levels: local=%d%%, remote=%d%%, diff=%d%%", 
            local_battery, remote_battery, battery_diff);

    // Check if switch is warranted
    if (abs(battery_diff) < CONFIG_ZMK_SPLIT_BLE_ROLE_SWAPPING_BATTERY_THRESHOLD) {
        LOG_DBG("Battery difference below threshold (%d%%), no switch needed", 
                CONFIG_ZMK_SPLIT_BLE_ROLE_SWAPPING_BATTERY_THRESHOLD);
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Switch if remote has higher battery
    if (remote_battery > local_battery) {
        LOG_INF("Remote battery higher (%d%% vs %d%%) - initiating role switch", 
                remote_battery, local_battery);
        initiate_role_switch();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/**
 * Split transport event listener - handles coordination messages
 */
static int split_transport_listener(const zmk_event_t *eh) {
    // Handle peripheral battery state changed events for ACK messages on central
    if (current_role_is_central) {
        const struct zmk_peripheral_battery_state_changed *battery_ev;
        if ((battery_ev = as_zmk_peripheral_battery_state_changed(eh)) != NULL) {
            // This is the existing battery event - we don't interfere with it
            return ZMK_EV_EVENT_BUBBLE;
        }
        
        // TODO: Add handler for role switch ACK events when transport extension is implemented
        // For now, we'd need to add this once we extend the transport system
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/**
 * Split command handler (peripheral side) - handles RST commands
 * NOTE: This would be called by the split transport when our command type is received
 */
static int split_command_handler(uint8_t source, struct zmk_split_transport_central_command *cmd) {
    if (cmd->type != ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_ROLE_SWAP_RST) {
        return 0; // Not our command
    }

    return handle_role_switch_rst(cmd->data.role_switch_rst.magic);
}

/**
 * Initiate role switch by coordinating with peripheral via RST/ACK handshake
 */
static int initiate_role_switch(void) {
    int ret;

    if (waiting_for_ack) {
        LOG_WRN("Role switch already in progress, ignoring request");
        return -EBUSY;
    }

    LOG_INF("Initiating role switch - sending RST command to peripheral");

    // Send RST command to peripheral
    struct zmk_split_transport_central_command command = {
        .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_ROLE_SWITCH_RST,
        .data = {.role_switch_rst = {.magic = ROLE_SWAP_MAGIC}}
    };

    ret = zmk_split_central_send_command(0, command);
    if (ret) {
        LOG_ERR("Failed to send RST command: %d", ret);
        return ret;
    }

    // Set waiting state and start timeout
    waiting_for_ack = true;
    k_work_schedule(&rst_timeout_work, K_MSEC(RST_TIMEOUT_MS));

    LOG_DBG("RST command sent, waiting for ACK (timeout: %dms)", RST_TIMEOUT_MS);
    return 0;
}

/**
 * Handle timeout waiting for ACK from peripheral
 */
static void rst_timeout_handler(struct k_work *work) {
    if (!waiting_for_ack) {
        return;
    }

    LOG_WRN("Timeout waiting for role switch ACK from peripheral - aborting");
    waiting_for_ack = false;
}

/**
 * Handle ACK received from peripheral - complete the role switch
 */
static int handle_role_switch_ack(uint8_t magic) {
    if (!waiting_for_ack) {
        LOG_WRN("Received unexpected role switch ACK");
        return 0;
    }

    if (magic != ROLE_SWAP_MAGIC) {
        LOG_WRN("Received role switch ACK with invalid magic: 0x%02X", magic);
        return -EINVAL;
    }

    LOG_INF("Received role switch ACK - completing role switch");

    // Cancel timeout and clear waiting state
    k_work_cancel_delayable(&rst_timeout_work);
    waiting_for_ack = false;

    // Save settings with swapped role
    int ret = save_current_role_settings(!current_role_is_central);
    if (ret) {
        LOG_ERR("Failed to save settings for role switch: %d", ret);
        return ret;
    }

    // Brief delay then reset
    k_sleep(K_MSEC(100));
    
    LOG_INF("Resetting to complete role switch");
    sys_reboot(SYS_REBOOT_WARM);

    return 0; // Never reached
}

/**
 * Handle RST command received from central (peripheral side)
 */
static int handle_role_switch_rst(uint8_t magic) {
    if (magic != ROLE_SWAP_MAGIC) {
        LOG_WRN("Received role switch RST with invalid magic: 0x%02X", magic);
        return -EINVAL;
    }

    LOG_INF("Received role switch RST command from central");

    // Check if we're currently active
    if (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE) {
        LOG_INF("Currently active - ignoring role switch request");
        return 0; // Don't send ACK
    }

    LOG_INF("Ready for role switch - sending ACK and preparing to reset");

    // Send ACK back to central
    struct zmk_split_transport_peripheral_event ev = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_ROLE_SWITCH_ACK,
        .data = {.role_switch_ack = {.magic = ROLE_SWAP_MAGIC}}
    };

    int ret = zmk_split_peripheral_report_event(&ev);
    if (ret) {
        LOG_ERR("Failed to send role switch ACK: %d", ret);
        return ret;
    }

    // Save settings with swapped role
    ret = save_current_role_settings(!current_role_is_central);
    if (ret) {
        LOG_ERR("Failed to save settings for role switch: %d", ret);
        // Continue anyway - central will reset and we should follow
    }

    // Delay briefly to coordinate with central, then reset
    k_sleep(K_MSEC(PERIPHERAL_RESET_DELAY_MS));
    
    LOG_INF("Resetting to complete role switch");
    sys_reboot(SYS_REBOOT_WARM);

    return 0; // Never reached
}

/**
 * Main initialization function - replaces the original split init SYS_INIT calls
 */
static int zmk_role_swapping_init(void) {
    int ret;

    LOG_INF("Initializing role swapping module");

    // Load settings to determine current role
    ret = load_settings();
    if (ret) {
        LOG_ERR("Failed to load settings: %d", ret);
        return ret;
    }

    current_role_is_central = is_central;

    // Call appropriate split init function first
    if (current_role_is_central) {
        LOG_INF("Initializing as central");
        ret = zmk_split_bt_central_init();
        if (ret) {
            LOG_ERR("Failed to initialize central: %d", ret);
            return ret;
        }
    } else {
        LOG_INF("Initializing as peripheral");
        ret = zmk_peripheral_ble_init();
        if (ret) {
            LOG_ERR("Failed to initialize peripheral: %d", ret);
            return ret;
        }
    }

    // Select appropriate profile after split init completes
    uint8_t target_profile = current_role_is_central ? central_profile_index : peripheral_profile_index;
    uint8_t current_active_profile = zmk_ble_active_profile_index();
    
    // Check if this boot resulted from a role switch by comparing profiles
    if (target_profile != current_active_profile) {
        LOG_INF("Detected role switch boot (target=%d, current=%d) - will restore idle state", 
                target_profile, current_active_profile);
        
        // Set correct profile first
        ret = zmk_ble_prof_select(target_profile);
        if (ret) {
            LOG_WRN("Failed to select target profile %d: %d", target_profile, ret);
        } else {
            LOG_INF("Selected profile %d for %s role", target_profile, 
                    current_role_is_central ? "central" : "peripheral");
        }
        
        // Brief delay to ensure initialization is complete, then restore idle state
        k_sleep(K_MSEC(500));
        
        ret = zmk_activity_set_state(ZMK_ACTIVITY_IDLE);
        if (ret) {
            LOG_WRN("Failed to restore idle state: %d", ret);
        } else {
            LOG_INF("Restored idle state after role switch");
        }
    } else {
        // Normal boot - just set the correct profile
        ret = zmk_ble_prof_select(target_profile);
        if (ret) {
            LOG_WRN("Failed to select profile %d: %d", target_profile, ret);
        } else {
            LOG_INF("Selected profile %d for %s role", target_profile, 
                    current_role_is_central ? "central" : "peripheral");
        }
    }

    LOG_INF("Role swapping module initialized successfully as %s", 
            current_role_is_central ? "central" : "peripheral");

    return 0;
}

// Event listener registration
ZMK_LISTENER(role_swapping, activity_state_listener);
ZMK_SUBSCRIPTION(role_swapping, zmk_activity_state_changed);

/**
 * Public API functions
 */
bool zmk_role_swapping_is_central(void) {
    return current_role_is_central;
}

int zmk_role_swapping_manual_switch(void) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_SWAPPING_MANUAL_SWITCH_BEHAVIOR)
    LOG_INF("Manual role switch triggered");
    return initiate_role_switch();
#else
    LOG_WRN("Manual role switching not enabled in configuration");
    return -ENOTSUP;
#endif
}

bool zmk_role_swapping_is_active(void) {
    // Role switching is active if we're central (only central monitors battery)
    return current_role_is_central;
}

// Initialize at the same priority as the original split init functions
SYS_INIT(zmk_role_swapping_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
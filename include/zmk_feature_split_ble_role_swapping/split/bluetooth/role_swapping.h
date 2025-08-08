#pragma once

#include <zephyr/types.h>

/**
 * @brief ZMK Split BLE Role Swapping Module
 * 
 * This module provides dynamic role switching between central and peripheral
 * roles for ZMK split keyboards based on battery levels to optimize power
 * consumption.
 * 
 * Key features:
 * - Boot-time role selection based on persistent settings
 * - Runtime role switching triggered by battery level differences
 * - Profile management for seamless reconnection after role switches
 * - Coordination between split halves during role transitions
 */

/**
 * @brief Manually trigger a role switch (for testing/debugging)
 * 
 * @return 0 on success, negative error code on failure
 */
int zmk_role_swapping_manual_switch(void);

/**
 * @brief Handle RST command received from central (peripheral side)
 * 
 * Called by core ZMK when peripheral receives role swap reset command.
 * Module should validate the command and send ACK if ready to proceed.
 * 
 * @param data RST command data containing magic validation byte
 * @return 0 on success, negative error code on failure
 */
int zmk_role_swapping_handle_rst_command(const struct zmk_split_transport_central_role_switch_rst_data *data);

/**
 * @brief Handle ACK event received from peripheral (central side)
 * 
 * Called by core ZMK when central receives role swap acknowledgment.
 * Module should proceed with role switch after receiving this ACK.
 * 
 * @param source Peripheral source ID that sent the ACK
 * @param data ACK event data containing magic validation byte
 * @return 0 on success, negative error code on failure
 */
int zmk_role_swapping_handle_ack_event(uint8_t source, const struct zmk_split_transport_peripheral_role_switch_ack_data *data);

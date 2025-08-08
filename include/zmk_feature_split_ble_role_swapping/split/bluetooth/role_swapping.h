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
 * @brief Check if currently operating as central
 * 
 * @return true if current role is central, false if peripheral
 */
bool zmk_role_swapping_is_central(void);

/**
 * @brief Manually trigger a role switch (for testing/debugging)
 * 
 * @return 0 on success, negative error code on failure
 */
int zmk_role_swapping_manual_switch(void);

/**
 * @brief Get the current battery-based role switching status
 * 
 * @return true if role switching is active and monitoring battery levels
 */
bool zmk_role_swapping_is_active(void);
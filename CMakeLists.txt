if(CONFIG_ZMK_SPLIT_BLE_ROLE_SWAPPING)
    target_sources(app PRIVATE src/split/bluetooth/role_swapping.c)
    target_include_directories(app PRIVATE include)
endif()
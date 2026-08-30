set(TETHER_SRCS
    main.cpp
    networking/espnow_transport.cpp
    networking/peer_manager.cpp
    tracking/proximity.cpp
    tracking/signal_sweep.cpp
    sensors/imu.cpp
    audio/audio.cpp
    ui/ui.cpp
    ui/screens/home_screen.cpp
    hub/hub.cpp
    hub/hub_ui.cpp)

idf_component_register(SRCS ${TETHER_SRCS}
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_wifi esp_event esp_timer driver esp_lcd
                                esp_lcd_touch esp32_s3_touch_amoled_1_8)

# Device identity injected by the build system (see tools/flash.sh)
if(NOT DEFINED TETHER_DEVICE_ID)
    set(TETHER_DEVICE_ID 1)
endif()
if(NOT DEFINED TETHER_ROLE)
    set(TETHER_ROLE 0)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    TETHER_DEVICE_ID=${TETHER_DEVICE_ID}
    TETHER_ROLE=${TETHER_ROLE})

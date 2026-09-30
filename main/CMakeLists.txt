set(esp_sdr_supported_targets esp32s3)
if(NOT IDF_TARGET IN_LIST esp_sdr_supported_targets)
    message(FATAL_ERROR "This standalone TX build targets ESP32-S3 only")
endif()

option(SAMPLE_RATE_PROBE "Build volatile hardware sample-rate diagnostics (never package)" OFF)

set(target_dir "targets/${IDF_TARGET}")
set(receiver "${target_dir}/receiver.c")
set(sources "common/burst_serial.c" "${target_dir}/display.c" "${target_dir}/esp_lcd_st7735.c")
set(dependencies esp_phy esp_wifi nvs_flash esp_timer esp_driver_uart esp_driver_usb_serial_jtag esp_driver_spi esp_driver_gpio esp_lcd)
set(probes)

list(APPEND probes SAMPLE_RATE_PROBE FILTER_REGISTER_PROBE S3_RF_PROBE)

idf_component_register(SRCS "${receiver}" ${sources}
    PRIV_INCLUDE_DIRS "common" "${target_dir}" "diagnostics"
    REQUIRES ${dependencies})

target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/${target_dir}/sram_guard.ld")

foreach(probe IN LISTS probes)
    if(${probe})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE "${probe}=1")
    endif()
endforeach()

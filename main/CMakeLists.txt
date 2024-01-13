set(srcs
    "joystick_main.cpp"
    "esp_hidd_prf_api.c"
    "hid_dev.c"
    "hid_device_le_prf.c"
    "joystick_buttons.c")
set(include_dirs  ".")

if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp32-button bno055 tracker
                       PRIV_REQUIRES bt nvs_flash)
else()
    set(COMPONENT_SRCDIRS "${srcs}")
    set(COMPONENT_ADD_INCLUDEDIRS "${include_dirs}")
    set(COMPONENT_REQUIRES esp_hid driver esp32-button bno055 tracker)
endif()


if(GCC_NOT_5_2_0)
    component_compile_options(-Wno-unused-const-variable)
    endif()
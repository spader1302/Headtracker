set(srcs "joystick_main.c"
    "esp_hidd_prf_api.c"
    "hid_dev.c"
    "hid_device_le_prf.c"
    "joystick_buttons.c")
set(include_dirs  ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp32-button
                       PRIV_REQUIRES bt nvs_flash)

if(GCC_NOT_5_2_0)
    component_compile_options(-Wno-unused-const-variable)
    endif()

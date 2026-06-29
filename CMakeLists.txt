idf_component_register(
    SRCS
        Adafruit_I2CDevice.cpp
        arg_parser.cpp
        astroclock.cpp
        cobalt_alien_17.c
        cobalt_alien_25.c
        config.cpp
        custommapping.cpp
        hextodig.cpp
        i2c_helper.cpp
        ins.cpp
        LMST.cpp
        main.cpp
        matrix.cpp
        meteors.cpp
        Mono_Bold_12.c
        Mono_Bold_14.c
        Mono_Bold_16.c
        multiplexers.cpp
        RTC_DS3231.cpp
        RTCLib.cpp
        satio.cpp
        satio_file.cpp
        satio_lvgl.cpp
        sdcard_helper.cpp
        serial_infocmd.cpp
        sidereal_helper.cpp
        SiderealObjects.cpp
        SiderealObjectsTables.cpp
        SiderealPlanets.cpp
        strval.cpp
        system_data.cpp
        task_handler.cpp
        unscii_12.c
        wit_c_sdk.cpp
        wt901.cpp
        wtgps300p.cpp
    INCLUDE_DIRS
        .
)

# add_definitions(-DARDUINO_USB_MODE -DARDUINO_USB_CDC_ON_BOOT)

# # Include extra CMake options to relax warnings-as-errors
# include("${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt.extra")

# Disable strict warnings for SiderealPlanets library (Arduino port)
set_source_files_properties(
    SiderealPlanets.cpp
    SiderealObjects.cpp
    PROPERTIES COMPILE_FLAGS 
    "-Wno-dangling-else -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized"
)
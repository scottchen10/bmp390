add_library(pico_bmp390 STATIC
    src/driver_bmp390.c
    interface/driver_bmp390_interface.c
    example/driver_bmp390_basic.c
)

target_include_directories(pico_bmp390
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/interface
        ${CMAKE_CURRENT_SOURCE_DIR}/example
)

target_link_libraries(pico_bmp390
    PUBLIC
        hardware_gpio
        hardware_i2c
        hardware_spi
        pico_stdlib
        pico_stdio
)

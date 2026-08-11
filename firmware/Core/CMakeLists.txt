cmake_minimum_required(VERSION 3.16)
project(mcr_firmware_sil LANGUAGES C)

# --- SIL target: compile firmware as a Linux host binary ---
# The firmware sources are compiled with mock HAL + FreeRTOS headers so
# they run as a native executable alongside the UART simulator.

set(FIRMWARE_SRC
    # Application logic
    Src/ahrs.c
    Src/encoder.c
    Src/main.c
    Src/mecanum_ik.c
    Src/motor.c
    Src/mpu6050.c
    Src/nrf24l01.c
    Src/pid.c
    Src/remote_control.c
    Src/robot_control.c
    Src/tof_sensor.c

    # Mock layer
    SIL/mocks/freertos_stubs.c
    SIL/mocks/hal_stubs.c
    SIL/mocks/mock_hal.c

    # SIL entry point (replaces firmware main.c's main())
    SIL/sil_main.c

    # Shared protocol library (used by robot_control.c)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/protocol.c
)

add_executable(sil_firmware ${FIRMWARE_SRC})

# --- Include paths ---
# Mock headers MUST come first so they shadow any real HAL/system headers.
target_include_directories(sil_firmware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/SIL/mocks    # mock FreeRTOS, HAL headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc           # firmware app headers
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared  # protocol.h
)

# --- Compile definitions ---
# Rename the firmware's main() so sil_main.c's main() is the real entry point.
target_compile_definitions(sil_firmware PRIVATE
    SIL_BUILD=1
)

# --- Compiler flags ---
target_compile_options(sil_firmware PRIVATE
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-missing-field-initializers
    -std=c11
)

# --- Linker ---
target_link_libraries(sil_firmware PRIVATE m)

# --- CI test target ---
add_custom_target(sil_test
    COMMAND $<TARGET_FILE:sil_firmware> --ci
    DEPENDS sil_firmware
    COMMENT "Running SIL firmware test (--ci mode)"
)

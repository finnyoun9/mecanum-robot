cmake_minimum_required(VERSION 3.16)
project(mcr_firmware_sil LANGUAGES C)

include(CTest)

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
# -Werror keeps the SIL build at zero warnings (see docs/agent-handoff.md:
# the xLastWakeTime set-but-unused warning is a known regression risk).
target_compile_options(sil_firmware PRIVATE
    -Wall -Wextra -Werror
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-missing-field-initializers
    -std=c11
)

# --- Linker ---
target_link_libraries(sil_firmware PRIVATE m)

# --- Tests (registered with CTest: run `ctest` from the build dir) ---
if(BUILD_TESTING)
    # 1. Full-firmware SIL: PID + encoder + protocol closed loop (--ci mode)
    add_test(NAME sil_firmware_ci
             COMMAND sil_firmware --ci)

    # 2. Host unit tests (pure C, no FreeRTOS/HAL deps)
    add_executable(test_pid           Test/test_pid.c           Src/pid.c)
    add_executable(test_mecanum_ik    Test/test_mecanum_ik.c    Src/mecanum_ik.c)
    add_executable(test_ahrs          Test/test_ahrs.c          Src/ahrs.c)
    add_executable(test_remote_control Test/test_remote_control.c
                   Src/remote_control.c Src/mecanum_ik.c)

    foreach(host_test test_pid test_mecanum_ik test_ahrs test_remote_control)
        target_include_directories(${host_test} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
        target_compile_options(${host_test} PRIVATE
            -Wall -Wextra -Werror -std=c11)
        target_link_libraries(${host_test} PRIVATE m)
        add_test(NAME ${host_test} COMMAND ${host_test})
    endforeach()
endif()

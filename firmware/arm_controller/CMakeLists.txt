cmake_minimum_required(VERSION 3.16)
project(arm_controller_sil LANGUAGES C)

include(CTest)

# --- SIL target: compile the arm controller core as a Linux host binary. ---
# The vendor servo protocol is not yet confirmed, so servo_bus is provided
# by the SIL mock (SIL/mock_servo_bus.c) instead of a real STM32 driver.

set(ARM_CONTROLLER_SRC
    app/arm_control.c
    app/safety_monitor.c
    drivers/host_protocol.c

    SIL/sil_main.c
    SIL/mock_servo_bus.c

    # Shared protocol library (frame encode/decode for the host link)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/protocol.c
)

add_executable(sil_arm_controller ${ARM_CONTROLLER_SRC})

target_include_directories(sil_arm_controller PRIVATE
    app
    drivers
    SIL
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared
)

# -Werror per the closed-loop roadmap (M5): the packed-member misalignment
# fix must stay clean under -Wall -Wextra -Werror before it goes on the MCU.
target_compile_options(sil_arm_controller PRIVATE
    -Wall -Wextra -Werror
    -Wno-unused-parameter
    -std=c11
)

target_link_libraries(sil_arm_controller PRIVATE m)

# --- Tests (registered with CTest: run `ctest` from the build dir) ---
if(BUILD_TESTING)
    add_test(NAME sil_arm_controller_ci
             COMMAND sil_arm_controller --ci)
endif()

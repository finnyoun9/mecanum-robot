# STM32F103C8T6 hardware targets

This directory contains standalone, bare-metal targets for real-hardware
bring-up. They do not start FreeRTOS and must not be described as the
encoder/PID/UART application.

## Verification status (2026-08-22)

The following targets were built from clean artifacts with `-Wall -Werror` on
the macOS ARM GNU Toolchain. The sizes are `text + data + bss` bytes.

| Target | Purpose | Build result |
| --- | --- | --- |
| `bringup` | PC13 LED smoke test | pass, 2,820 + 12 + 1,540 = 4,372 |
| `fl_motor_debug` | FL forward/reverse | pass, 4,176 + 12 + 1,620 = 5,808 |
| `fr_motor_debug` | FR forward/reverse | pass, 4,188 + 12 + 1,620 = 5,820 |
| `front_motors_debug` | FL + FR forward/reverse | pass, 4,224 + 12 + 1,684 = 5,920 |
| `three_motors_debug` | FL + FR + RL forward/reverse | pass, 4,392 + 12 + 1,756 = 6,160 |
| `four_motors_debug` | FL + FR + RL + RR forward/reverse | pass, 4,376 + 12 + 1,756 = 6,144 |
| `drive_control` | four-wheel drive command demo | pass, 4,672 + 16 + 1,856 = 6,544 |

The host SIL regression was also rebuilt and passed with CTest: 5/5
(`sil_firmware_ci`, PID, mecanum IK, AHRS, and remote-control tests).

Hardware verification completed during the same bring-up sequence:

- ST-Link/OpenOCD connected to the STM32F103C8T6; PC13 blink was observed.
- Each TB6612 channel was brought up incrementally: FL, FR, RL, then RR.
- `four_motors_debug` verified that all four wheels turn in the intended common
  physical direction after the motor lead polarity was corrected.
- `drive_control` was flashed and exercised with the chassis lifted. Its
  70% PWM demo runs forward, left translation, and counter-clockwise rotation,
  then disables both TB6612 bridges.

This proves the open-loop GPIO/PWM/bridge mapping only. Encoder inputs,
speed feedback, PID, UART control, battery operation, and on-ground motion
remain unverified.

## Build all verified targets

Run from this directory. `drive_control` alone links the production
`motor.c` implementation.

```sh
for target in bringup fl_motor_debug fr_motor_debug front_motors_debug \
  three_motors_debug four_motors_debug; do
  make clean
  make TARGET="$target"
done

make clean
make TARGET=drive_control EXTRA_SRCS='../Src/motor.c'
```

## Target overview

`bringup` is the PC13 LED smoke test. `fl_motor_debug` is a one-shot,
open-loop test for the front-left motor only; it does not start FreeRTOS or
touch any other motor pin.

## Front-left test

Before flashing, lift the chassis so the wheel can spin freely, use a
current-limited motor supply, and confirm the board-1 connections:

- `PB14` -> TB6612 `STBY`
- `PA2` -> `PWMA` / TIM2_CH3
- `PA4` -> `AIN1`
- `PA5` -> `AIN2`

Build and flash:

```sh
make TARGET=fl_motor_debug
make TARGET=fl_motor_debug flash
```

After a two-second disabled window, the program applies 20% PWM forward for
one second, disables the bridge for one second, applies 20% PWM reverse for
one second, then disables the bridge until reset. PC13 resumes its 500 ms
heartbeat only after this sequence completes.

For a GDB session, start the server in one terminal and attach from another:

```sh
make TARGET=fl_motor_debug debug-server
arm-none-eabi-gdb fl_motor_debug.elf
```

```gdb
target extended-remote :3333
monitor reset halt
print fl_debug_phase
print fl_debug_compare
continue
```

`stlink_stm32f1.cfg` contains the board's observed SW-DP ID
`0x2ba01477`, which is required by this ST-Link/OpenOCD combination.

## Front-right test

Turn off the motor supply before wiring. Board 1's `STBY` remains on `PB14`;
the front-right motor uses the TB6612 B channel:

- `PB0` -> `PWMB` / TIM3_CH3
- `PA11` -> `BIN1`
- `PA12` -> `BIN2`
- Front-right motor -> `BO1/BO2`

Build and flash only after confirming those connections:

```sh
make TARGET=fr_motor_debug
make TARGET=fr_motor_debug flash
```

The sequence and safety behaviour are the same as the front-left test.

## Multi-wheel debug targets

All multi-wheel targets keep both TB6612 `STBY` pins low during the initial
two-second window and after each test. Lift the chassis before flashing.

| Target | Wheel set | Sequence |
| --- | --- | --- |
| `front_motors_debug` | FL, FR | 70% forward 5 s, stop 1 s, 70% reverse 5 s, stop |
| `three_motors_debug` | FL, FR, RL | same sequence |
| `four_motors_debug` | FL, FR, RL, RR | same sequence |

For example:

```sh
make TARGET=four_motors_debug flash
```

## Formal open-loop four-wheel control

`drive_control` uses `firmware/Core/Src/motor.c` rather than direct register
writes. The current boot demo is safe by default and executes after a
two-second disabled window:

1. forward at 70% PWM for 1.5 s;
2. stop with both bridges disabled for 0.6 s;
3. left translation at 70% PWM for 1.5 s;
4. stop for 0.6 s;
5. counter-clockwise rotation at 70% PWM for 1.5 s;
6. stop with both bridges disabled forever.

Build and flash it with:

```sh
make TARGET=drive_control EXTRA_SRCS='../Src/motor.c' flash
```

The target is intentionally an open-loop smoke test. Do not use it as the
final vehicle controller until encoder feedback, an input command path, and
the battery power system are integrated.

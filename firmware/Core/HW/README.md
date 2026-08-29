# STM32F103C8T6 hardware targets

This directory contains real-hardware targets. Most are standalone bare-metal
bring-up probes; `rtos_drive` is the production-shaped FreeRTOS application and
must not be confused with those probes.

## Verification status (2026-08-29)

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
| `encoder_debug` | four-wheel drive + software quadrature decode | pass, 5,108 + 16 + 1,888 = 7,012 |
| `encoder_count` | passive counter, motors never driven | pass, 3,412 + 12 + 1,572 = 4,996 |
| `duty_sweep` | duty -> speed curve across 10 levels | pass, 5,320 + 16 + 2,072 = 7,408 |
| `encoder_port_check` | production encoder.c on hardware | pass, 6,544 + 16 + 1,936 = 8,496 |
| `remote_drive` | NRF24 remote -> four-wheel open-loop drive | pass, 7,512 + 16 + 1,904 = 9,432 |
| `remote_pid_drive` | NRF24 remote -> four independent speed-PI loops | build + basic hardware motion pass, 9,208 + 16 + 2,232 = 11,456 |
| `rr_encoder_probe` | RR passive raw A/B continuity diagnostic | hardware pass: 10 turns -> A/B 4,516 each; 3,016 + 12 + 1,548 = 4,576 |
| `rr_motor_encoder_probe` | RR-only 1.5 s full-duty raw A/B diagnostic | hardware pass: A/B 3,384/3,383, decoded +3,384; 4,484 + 16 + 1,728 = 6,228 |
| `uart_link_probe` | Pi ↔ STM32 USART1 protocol transport only | build pass, 4,816 + 12 + 1,716 = 6,544; hardware pending |
| `i2c_bus_probe` | no-motor MPU6050/VL53L0X scan + SSD1306 all-pixels test | hardware pass |
| `rtos_drive` | FreeRTOS drive + UART DMA + MPU6050 on I2C2 | build + flash + IMU hardware pass, 25,204 + 360 + 14,040 = 39,604 |
| `rtos_drive TOF=1` | adds VL53L0X continuous ranging on shared I2C2 | hardware pass, 26,956 + 360 + 14,056 = 41,372 |
| `rtos_drive TOF=1 OLED=1` | adds three-page SSD1306 chassis UI | hardware pass, 29,416 + 360 + 15,120 = 44,896 |

The host regression passes with CTest: 10/10 (`sil_firmware_ci`, PID,
mecanum IK, AHRS, remote-control, encoder, stall, MPU6050, VL53L0X and SSD1306).

The MPU6050 hardware path was closed on 2026-08-29. Build and flash with the
ST-Link path verified on this chassis (the probe's old `0x1ba01477` SWD IDCODE
is rejected by OpenOCD 0.12), then run the watcher on the Pi. It sends heartbeat
only, so it does not release the motors from their stopped state:

```sh
make TARGET=rtos_drive RTOS=1 flash-stlink
python3 tools/imu_watch.py --duration 15
```

The accepted run received 754 ODOM samples at about 50 Hz with zero CRC
failures, unit quaternions, non-zero gyro data and `error_flags=0`. The observed
stationary Y-axis gyro bias was about +0.045 rad/s; bias calibration remains the
next IMU accuracy task.

With VL53L0X wired to PB10/PB11 alongside the MPU6050, enable and validate it:

```sh
make TARGET=rtos_drive RTOS=1 TOF=1 flash-stlink
python3 tools/tof_watch.py --duration 15
```

The accepted hardware run received 754 ODOM frames in 15 seconds with zero CRC
failures, a valid 771 mm range and `error_flags=0`. The SSD1306 at `0x3c` also
passed address detection, initialization, clear and all-pixels-on testing on the
same bus. `i2c_bus_probe` is the no-motor diagnostic target for repeating these
checks.

Enable the chassis status UI with:

```sh
make TARGET=rtos_drive RTOS=1 TOF=1 OLED=1 flash-stlink
```

It rotates overview, wheel-speed and IMU pages. It uses the same SSD1306
page-address and control-byte transport proven in `stm32-smart-home-ota`.
The module's `0x78` address-select silk screen is its 8-bit write address;
the firmware correctly uses 7-bit `0x3C`.

Hardware verification completed during the same bring-up sequence:

- ST-Link/OpenOCD connected to the STM32F103C8T6; PC13 blink was observed.
- Each TB6612 channel was brought up incrementally: FL, FR, RL, then RR.
- `four_motors_debug` verified that all four wheels turn in the intended common
  physical direction after the motor lead polarity was corrected.
- `drive_control` was flashed and exercised with the chassis lifted. Its
  70% PWM demo runs forward, left translation, and counter-clockwise rotation,
  then disables both TB6612 bridges.

Encoder bring-up followed (2026-08-23, chassis lifted, 40% duty for 1.5s,
`encoder_debug`): all four channels decode, counting positive while the
robot drives physically forward — FL 631, FR 633, RL 640, RR 619, a ~3%
spread across wheels. Two calibration fixes were needed to get there:

- **DIR polarity.** A positive duty drove the chassis *backwards* on all
  four wheels. Fixed by passing each motor's DIR pins to `motor_set_tim()`
  in (AIN2, AIN1) order, so `positive duty == forward` holds for every
  caller rather than pushing sign flips up into kinematics/PID/teleop.
- **FL encoder phase.** FL counted negative while the other three counted
  positive for the same physical direction; its two encoder wires were
  swapped in the harness.

All four motors are driven from TIM2/3/4 PWM, so no timer remains for
STM32 hardware encoder mode (TIM1's encoder channels sit on PA8/PA9,
taken by NRF24L01 CE and USART1). All four encoders therefore use
software EXTI decode — see the header comment in `encoder_debug_main.c`
for the EXTI line allocation and why RL uses PB7 rather than PB6.

`duty_sweep` then measured the duty -> speed curve; results and analysis
are in [docs/hardware-closed-loop-roadmap.md](../../../docs/hardware-closed-loop-roadmap.md).

`encoder_port_check` closes the loop on the encoder.c port from
hardware-timer mode to software decode. Unlike `encoder_debug`, which
carried private counters, it links the production `../Src/encoder.c` and
feeds it through `encoder_on_edge()` exactly as the FreeRTOS application
will. Measured at 40% duty with the chassis lifted: counts
{577, 577, 582, 565} and `encoder_get_speed_rads()` returning
{10.24, 10.38, 10.38, 10.24} rad/s. That speed independently agrees with
the 10.25 rad/s the duty sweep predicts for 40%, via a separate code path
— evidence the edges-to-rad/s conversion is right, not just self-consistent.

This proves open-loop GPIO/PWM/bridge mapping, encoder direction and
counting, and speed feedback in engineering units. The `remote_pid_drive`
target has additionally passed suspended four-wheel forward/reverse/strafe/
rotation checks and low-speed ground basic-motion checks. Long-duration
loaded tracking and quantified ground performance remain unverified; K9 and
remote-loss safety were re-verified on the M3 image.

**Measuring these counters:** `st-util` resets the target both on start
and when GDB attaches, so halting right after attach samples a chip that
restarted milliseconds earlier — still inside the 2s startup delay, which
reads as "encoders stuck at zero while the wheels visibly spin". Let it
free-run first, then interrupt:

```sh
# NRF24 remote drive. Links the radio driver, the joystick mapping and the
# motor driver -- the first target to build nrf24l01.c against the real HAL.
make clean
make TARGET=remote_drive EXTRA_SRCS='../Src/motor.c ../Src/nrf24l01.c ../Src/remote_control.c ../Src/mecanum_ik.c'
```

```sh
arm-none-eabi-gdb -q --batch \
  -ex "target extended-remote localhost:4242" \
  -ex "continue" \
  -ex "print fl_count" -ex "print fr_count" \
  -ex "print rl_count" -ex "print rr_count" \
  encoder_debug.elf > /tmp/gdb.log 2>&1 &
sleep 8; kill -INT %1; sleep 2; grep '^\$' /tmp/gdb.log
```

## Build all verified targets

Run from this directory. `encoder_port_check` links both production
`motor.c` and `encoder.c`; `drive_control`, `encoder_debug` and
`duty_sweep` link `motor.c` only; the rest drive registers directly.

```sh
for target in bringup fl_motor_debug fr_motor_debug front_motors_debug \
  three_motors_debug four_motors_debug; do
  make clean
  make TARGET="$target"
done

make clean
make TARGET=encoder_count

# These link production implementations from ../Src.
for target in drive_control encoder_debug duty_sweep; do
  make clean
  make TARGET="$target" EXTRA_SRCS='../Src/motor.c'
done

make clean
make TARGET=encoder_port_check EXTRA_SRCS='../Src/motor.c ../Src/encoder.c'

# M3 speed-PI target. It starts disabled; keep the chassis lifted whenever
# inspecting raw wheel telemetry or changing gains.
make clean
make TARGET=remote_pid_drive EXTRA_SRCS='../Src/motor.c ../Src/encoder.c ../Src/pid.c ../Src/nrf24l01.c ../Src/remote_control.c ../Src/mecanum_ik.c'

# M4 step 1.  PA9/PA10 at 921600 bit/s; sends empty odometry at 20 Hz and
# acknowledges heartbeats.  It does not initialise motors, TB6612 or PWM.
make clean
make TARGET=uart_link_probe EXTRA_SRCS='../../../shared/protocol.c'
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

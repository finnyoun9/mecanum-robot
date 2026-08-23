#!/usr/bin/env bash
# Run one PID step test on the target and print the response metrics.
#
# Usage: ./pid_run.sh <kp> <ki> <setpoint_rad_s>
#
# Drives the gains into RAM over GDB and triggers a run, so a tuning sweep
# needs no reflashing between gain values. See pid_step_main.c.
set -euo pipefail

KP=${1:?usage: pid_run.sh <kp> <ki> <setpoint>}
KI=${2:?usage: pid_run.sh <kp> <ki> <setpoint>}
SP=${3:?usage: pid_run.sh <kp> <ki> <setpoint>}

TOOLCHAIN=/Users/finn/.local/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin
HERE="$(cd "$(dirname "$0")" && pwd)"
ELF="$HERE/pid_step.elf"
RAW=/tmp/pid_run_raw.log

pkill -9 st-util 2>/dev/null || true
pkill -9 -f "arm-none-eabi-gdb" 2>/dev/null || true
sleep 1
(st-util > /tmp/st-util.log 2>&1 &)
sleep 2

# st-util resets the target on attach, so configure and trigger from inside
# this one session rather than across separate attaches.
"$TOOLCHAIN/arm-none-eabi-gdb" -q --batch \
  -ex "target extended-remote localhost:4242" \
  -ex "set confirm off" \
  -ex "set \$sp_val = $SP" \
  -ex "continue" \
  "$ELF" > "$RAW" 2>&1 &
GDBPID=$!
sleep 3
kill -INT $GDBPID 2>/dev/null || true
sleep 1

"$TOOLCHAIN/arm-none-eabi-gdb" -q --batch \
  -ex "target extended-remote localhost:4242" \
  -ex "set confirm off" \
  -ex "set var cfg_kp = $KP" \
  -ex "set var cfg_ki = $KI" \
  -ex "set var cfg_setpoint = $SP" \
  -ex "set var run_done = 0" \
  -ex "set var cmd_run = 1" \
  -ex "continue" \
  -ex "print run_done" \
  -ex "print log_count" \
  -ex "set print elements 300" \
  -ex "set print repeats 0" \
  -ex "print log_speed" \
  -ex "print log_pwm" \
  -ex "print log_period_us" \
  "$ELF" > "$RAW" 2>&1 &
GDBPID=$!
sleep 6
kill -INT $GDBPID 2>/dev/null || true
sleep 3

pkill -9 st-util 2>/dev/null || true
pkill -9 -f "arm-none-eabi-gdb" 2>/dev/null || true

python3 "$HERE/pid_metrics.py" "$RAW" "$KP" "$KI" "$SP"

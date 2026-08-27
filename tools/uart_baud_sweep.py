#!/usr/bin/env python3
"""Listen-only UART baud sweep on /dev/ttyAMA0.

The STM32 streams 50 Hz ODOM frames (sync A5 5A). At the CORRECT baud we
see ~1.3 KB/s structured frames; at wrong bauds mostly 0xFF (idle)/garbage.
Reveals the STM32 actual baud WITHOUT touching SWD: if only 115200 decodes
clean, the STM32 is running at 8 MHz (PLL switch never happened) though
BRR was computed for 64 MHz.
"""
import os, sys, time, termios

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
BAUDS = [921600, 460800, 230400, 115200]
SECS = 3

for baud in BAUDS:
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    bconst = getattr(termios, "B%d" % baud)
    attrs[4] = bconst; attrs[5] = bconst
    attrs[0] = 0; attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    time.sleep(0.1)
    total = bytearray(); end = time.time() + SECS
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                total += chunk
        except BlockingIOError:
            time.sleep(0.02)
    os.close(fd)
    n = len(total)
    syncs = total.count(b"\xa5\x5a")
    ff = total.count(0xff)
    verdict = "<== CLEAN FRAMES" if syncs > SECS * 20 else ""
    print("baud=%7d: %5d B in %ds (%7.0f B/s)  A5_5A pairs=%4d  0xFF=%5d  %s"
          % (baud, n, SECS, n / SECS, syncs, ff, verdict))

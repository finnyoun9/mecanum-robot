#!/usr/bin/env python3
"""Host checks for the measured static wheel model in stm32_uart_sim.py."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from stm32_uart_sim import (  # noqa: E402
    EDGES_PER_WHEEL_REV,
    MAX_WHEEL_RAD_S,
    MIN_START_WHEEL_RAD_S,
    Stm32Sim,
)


class Stm32SimWheelModelTest(unittest.TestCase):
    @staticmethod
    def integrate(command_w: float, ticks: int = 50) -> int:
        sim = Stm32Sim(-1)
        sim.commanded_w = [command_w] * 4
        for _ in range(ticks):
            sim.send_odometry()
        return sim.counts[0]

    def test_deadband_produces_no_encoder_edges(self):
        self.assertEqual(self.integrate(MIN_START_WHEEL_RAD_S * 0.99), 0)
        self.assertEqual(self.integrate(-MIN_START_WHEEL_RAD_S * 0.99), 0)

    def test_speed_limit_caps_encoder_rate(self):
        capped = self.integrate(MAX_WHEEL_RAD_S)
        above_limit = self.integrate(MAX_WHEEL_RAD_S * 2.0)
        self.assertEqual(above_limit, capped)
        self.assertEqual(capped, int(4.27 * EDGES_PER_WHEEL_REV))

    def test_limit_preserves_direction(self):
        self.assertEqual(
            self.integrate(-MAX_WHEEL_RAD_S * 2.0),
            -self.integrate(MAX_WHEEL_RAD_S * 2.0),
        )


if __name__ == "__main__":
    unittest.main()

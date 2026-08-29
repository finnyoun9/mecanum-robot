import struct
import unittest

from imu_watch import OdomParser, quaternion_to_euler
from link_check import encode_frame


class ImuWatchTest(unittest.TestCase):
    def setUp(self):
        self.payload = struct.pack(
            "<4iH4f3fBB",
            1, 2, 3, 4, 123,
            1.0, 0.0, 0.0, 0.0,
            0.1, -0.2, 0.3,
            88, 0x04,
        )

    def test_decodes_fragmented_odom_frame(self):
        frame = encode_frame(0x20, self.payload, 7)
        parser = OdomParser()

        self.assertEqual(parser.feed(frame[:11]), [])
        samples = parser.feed(frame[11:])

        self.assertEqual(len(samples), 1)
        q, gyro, errors = samples[0]
        self.assertEqual(q, (1.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(gyro[0], 0.1)
        self.assertAlmostEqual(gyro[1], -0.2)
        self.assertAlmostEqual(gyro[2], 0.3)
        self.assertEqual(errors, 0x04)
        self.assertEqual(parser.crc_failures, 0)

    def test_rejects_bad_crc(self):
        frame = bytearray(encode_frame(0x20, self.payload, 7))
        frame[-1] ^= 0xFF
        parser = OdomParser()

        self.assertEqual(parser.feed(frame), [])
        self.assertEqual(parser.crc_failures, 1)

    def test_identity_quaternion_is_zero_euler(self):
        self.assertEqual(quaternion_to_euler((1.0, 0.0, 0.0, 0.0)),
                         (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()

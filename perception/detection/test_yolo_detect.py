import json
import unittest

import numpy as np

from perception.detection.yolo_detect import Detection, INPUT_SIZE, decode_predictions, detection_message, preprocess


class TestYoloDetect(unittest.TestCase):
    def test_preprocess_produces_normalised_nchw_tensor(self):
        frame = np.full((4, 6, 3), 255, dtype=np.uint8)
        tensor = preprocess(frame)
        self.assertEqual(tensor.shape, (1, 3, INPUT_SIZE, INPUT_SIZE))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertAlmostEqual(float(tensor[0, 0, 0, 0]), 1.0)

    def test_decoder_scales_and_suppresses_same_class_overlap(self):
        output = np.zeros((1, 84, 3), dtype=np.float32)
        output[0, :4, 0] = (320, 320, 320, 320)
        output[0, 4 + 62, 0] = 0.90  # tv
        output[0, :4, 1] = (322, 322, 320, 320)
        output[0, 4 + 62, 1] = 0.80
        output[0, :4, 2] = (100, 100, 40, 40)
        output[0, 4, 2] = 0.85  # person survives despite another class
        detections = decode_predictions(output, 1280, 960, 0.5, 0.5)
        self.assertEqual([item.label for item in detections], ["tv", "person"])
        self.assertEqual((detections[0].x1, detections[0].y1, detections[0].x2, detections[0].y2), (320, 240, 960, 720))

    def test_detection_message_is_compact_and_versioned(self):
        payload = detection_message(7, 640, 480, [Detection(0, 0.87654, 1, 2, 30, 40)])
        message = json.loads(payload)
        self.assertEqual(message["schema"], "mcr.perception.detections.v1")
        self.assertEqual(message["frame_id"], 7)
        self.assertEqual(message["detections"][0]["label"], "person")
        self.assertEqual(message["detections"][0]["confidence"], 0.8765)


if __name__ == "__main__":
    unittest.main()

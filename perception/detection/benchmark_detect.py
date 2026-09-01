#!/usr/bin/env python3
"""Benchmark harness for yolo_detect.py: per-model latency/FPS/CPU/memory report.

Standard library only, matching yolo_detect.py's dependency convention. Reuses
its camera/preprocess/decode helpers so the benchmark exercises the same code
path as the production smoke test, just with per-frame timing and resource
accounting added around it.
"""
import argparse
import json
import resource
import time
from pathlib import Path

import onnxruntime as ort

import yolo_detect as yd


def run_benchmark(model_path: Path, camera: str, device: int, width: int, height: int,
                   frames: int, confidence: float, nms: float) -> dict:
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    read_frame, close_camera = yd.open_camera(camera, width, height, device)
    latencies_ms: list[float] = []
    detection_counts: list[int] = []
    started = time.monotonic()
    try:
        for _ in range(frames):
            frame_rgb = read_frame()
            tensor = yd.preprocess(frame_rgb)
            t0 = time.monotonic()
            output = session.run(None, {input_name: tensor})[0]
            latencies_ms.append((time.monotonic() - t0) * 1000)
            detections = yd.decode_predictions(output, frame_rgb.shape[1], frame_rgb.shape[0], confidence, nms)
            detection_counts.append(len(detections))
    finally:
        close_camera()
    elapsed = max(time.monotonic() - started, 1e-9)
    usage = resource.getrusage(resource.RUSAGE_SELF)
    latencies_ms.sort()
    p95 = latencies_ms[min(len(latencies_ms) - 1, int(len(latencies_ms) * 0.95))]
    return {
        "model": model_path.name,
        "model_size_mb": round(model_path.stat().st_size / (1024 * 1024), 2),
        "frames": frames,
        "camera": camera,
        "resolution": f"{width}x{height}",
        "fps_overall": round(frames / elapsed, 2),
        "inference_latency_ms_avg": round(sum(latencies_ms) / len(latencies_ms), 2),
        "inference_latency_ms_p95": round(p95, 2),
        "inference_latency_ms_min": round(latencies_ms[0], 2),
        "inference_latency_ms_max": round(latencies_ms[-1], 2),
        "cpu_time_s": round(usage.ru_utime + usage.ru_stime, 2),
        "cpu_percent_of_one_core": round((usage.ru_utime + usage.ru_stime) / elapsed * 100, 1),
        "peak_rss_mb": round(usage.ru_maxrss / 1024, 1),
        "detections_total": sum(detection_counts),
        "frames_with_zero_detections": sum(1 for count in detection_counts if count == 0),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, nargs="+", required=True)
    parser.add_argument("--camera", choices=("csi", "usb"), default="csi")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--nms", type=float, default=0.50)
    parser.add_argument("--output", type=Path, default=Path("benchmark_report.json"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = []
    for model_path in args.models:
        if not model_path.is_file():
            raise SystemExit(f"model not found: {model_path}")
        print(f"benchmarking {model_path.name} ({args.frames} frames)...")
        result = run_benchmark(model_path, args.camera, args.device, args.width, args.height,
                                args.frames, args.confidence, args.nms)
        results.append(result)
        print(json.dumps(result, indent=2))
        time.sleep(1)  # let the camera fully release before the next model reopens it
    args.output.write_text(json.dumps(results, indent=2, ensure_ascii=False))
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

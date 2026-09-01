#!/usr/bin/env python3
"""NCNN benchmark, mirrors benchmark_detect.py's methodology (same camera
loop, P95 latency, CPU/RSS via resource.getrusage) so the numbers are
comparable to the ONNX Runtime fp32/int8 results in BENCHMARK.md.
"""
import argparse
import json
import resource
import time
from pathlib import Path

import ncnn
import numpy as np

import yolo_detect as yd

INPUT_SIZE = 640


def run_benchmark(param_path: Path, bin_path: Path, camera: str, device: int,
                   width: int, height: int, frames: int) -> dict:
    net = ncnn.Net()
    net.load_param(str(param_path))
    net.load_model(str(bin_path))

    read_frame, close_camera = yd.open_camera(camera, width, height, device)
    latencies_ms = []
    usage_start = resource.getrusage(resource.RUSAGE_SELF)
    t_start = time.monotonic()
    try:
        for _ in range(frames):
            frame_rgb = read_frame()
            tensor = yd.preprocess(frame_rgb)[0]  # CHW float32, 0..1
            mat_in = ncnn.Mat(tensor)
            t0 = time.monotonic()
            ex = net.create_extractor()
            ex.input("in0", mat_in)
            _, mat_out = ex.extract("out0")
            t1 = time.monotonic()
            latencies_ms.append((t1 - t0) * 1000.0)
    finally:
        close_camera()
    elapsed = max(time.monotonic() - t_start, 1e-9)
    usage_end = resource.getrusage(resource.RUSAGE_SELF)
    latencies_ms.sort()
    p95 = latencies_ms[int(len(latencies_ms) * 0.95) - 1]
    cpu_s = ((usage_end.ru_utime + usage_end.ru_stime)
             - (usage_start.ru_utime + usage_start.ru_stime))
    return {
        "model": "yolov8n_ncnn (fp32)",
        "model_size_mb": round(bin_path.stat().st_size / (1024 * 1024), 2),
        "frames": frames,
        "camera": camera,
        "resolution": f"{width}x{height}",
        "fps_overall": round(frames / elapsed, 2),
        "inference_latency_ms_avg": round(sum(latencies_ms) / len(latencies_ms), 2),
        "inference_latency_ms_p95": round(p95, 2),
        "inference_latency_ms_min": round(latencies_ms[0], 2),
        "inference_latency_ms_max": round(latencies_ms[-1], 2),
        "cpu_time_s": round(cpu_s, 2),
        "cpu_percent_of_one_core": round(cpu_s / elapsed * 100, 1),
        "peak_rss_mb": round(usage_end.ru_maxrss / 1024, 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", type=Path, required=True)
    ap.add_argument("--camera", choices=("csi", "usb"), default="csi")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--output", type=Path, default=Path("benchmark_ncnn_report.json"))
    args = ap.parse_args()

    result = run_benchmark(
        args.model_dir / "model.ncnn.param", args.model_dir / "model.ncnn.bin",
        args.camera, args.device, args.width, args.height, args.frames,
    )
    print(json.dumps(result, indent=2))
    args.output.write_text(json.dumps(result, indent=2))
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()

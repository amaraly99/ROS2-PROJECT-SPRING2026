#!/usr/bin/env python3
"""
vlm_detector_producer — open-vocabulary VLM SHM detector (text-prompted).

Reads camera frames from /ovcam_frames and writes the standard /yolo_shm
schema, so yolo_bridge and the controller are unchanged. The detector is
OWL-ViT (google/owlvit-base-patch32 via HuggingFace transformers): zero-shot,
text-prompted ("stop sign" works with no retraining), pure CPU.

Model choice note: the original plan specified GroundingDINO, but its pip
packaging is fragile on ARM (compiles custom C++ ops, hard torch pins).
OWL-ViT delivers the same open-vocabulary story through transformers, which
installs cleanly next to the existing torch 2.10 CPU build. Expect roughly
1-10 s/frame on RPi5 CPU — the very low rate is itself a paper result (the
framework tolerates detectors from 25 ms NPU YOLO to multi-second VLM with
zero downstream changes).

class_id convention: 100 = the VLM class slot. Map it in yolo_bridge via
class_name_map:='100:vlm_detection' and point visp at
target_class:='vlm_detection' (run_stack_hil.sh --detector vlm does both).

Telemetry: same 7-stamp format as yolo_shm_producer; for this detector
  preprocess  = RGB→PIL + OWL-ViT processor (resize/normalise/tokenise)
  inference   = transformer forward pass
  nms         = score threshold + box rescale to source pixels

If transformers/torch are missing this exits with code 1 — no silent
fallback to another detector (a misconfigured sweep must fail loudly).

CLI:
    python3 vlm_detector_producer.py --prompt "stop sign"
            [--box-threshold 0.3] [--model google/owlvit-base-patch32]
            [--threads 4] [--benchmark] [--frames N] [--telemetry_csv PATH]
"""

import argparse
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from detector_producer_base import run_producer  # noqa: E402

VLM_CLASS_ID = 100


def _now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


class OwlViTPredictor:
    """Predictor-contract wrapper around OWL-ViT (see detector_producer_base
    for the infer()/timing contract)."""

    def __init__(self, prompt: str, box_threshold: float,
                 model_name: str, threads: int):
        try:
            import torch
            from transformers import OwlViTProcessor, OwlViTForObjectDetection
            from PIL import Image
        except ImportError as e:
            print(f"ERROR: VLM dependencies missing ({e}). "
                  f"Install with: pip3 install --break-system-packages transformers",
                  file=sys.stderr)
            sys.exit(1)

        self._torch = torch
        self._PILImage = Image
        torch.set_num_threads(threads)

        # OWL-ViT was trained with CLIP-style templates; wrapping the raw
        # prompt measurably improves zero-shot scores.
        self.query = f"a photo of a {prompt}"
        self.box_threshold = box_threshold

        print(f"VLM model loading... ({model_name}; first run downloads weights)",
              file=sys.stderr)
        t0 = time.monotonic()
        self.processor = OwlViTProcessor.from_pretrained(model_name)
        self.model = OwlViTForObjectDetection.from_pretrained(model_name)
        self.model.eval()
        print(f"VLM model loaded in {time.monotonic() - t0:.1f}s", file=sys.stderr)

    def infer(self, rgb, timing=None):
        if timing is None:
            timing = {}
        torch = self._torch
        h, w = rgb.shape[:2]

        pil = self._PILImage.fromarray(rgb)
        inputs = self.processor(text=[[self.query]], images=pil,
                                return_tensors="pt")
        timing["pre_done"] = _now_ns()

        timing["inf_start"] = _now_ns()
        with torch.inference_mode():
            outputs = self.model(**inputs)
        timing["inf_done"] = _now_ns()

        target_sizes = torch.tensor([[h, w]])
        # transformers 5.x renamed post_process_object_detection →
        # post_process_grounded_object_detection; support both.
        post = getattr(self.processor, "post_process_grounded_object_detection",
                       None) or self.processor.post_process_object_detection
        results = post(outputs, threshold=self.box_threshold,
                       target_sizes=target_sizes)[0]

        dets = []
        for box, score in zip(results["boxes"], results["scores"]):
            x1, y1, x2, y2 = (float(v) for v in box)
            dets.append((max(0, int(x1)), max(0, int(y1)),
                         min(w - 1, int(x2)), min(h - 1, int(y2)),
                         VLM_CLASS_ID, float(score)))
        dets.sort(key=lambda d: d[5], reverse=True)
        timing["nms_done"] = _now_ns()
        return dets

    def close(self):
        pass


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="OWL-ViT VLM SHM detector producer")
    ap.add_argument("--prompt", default="stop sign",
                    help="open-vocabulary text query")
    ap.add_argument("--box-threshold", dest="box_threshold", type=float,
                    default=0.3, help="detection score threshold")
    ap.add_argument("--model", default="google/owlvit-base-patch32",
                    help="HuggingFace model id")
    ap.add_argument("--threads", type=int, default=4,
                    help="torch intra-op threads (ignored when taskset-pinned to 1 core)")
    ap.add_argument("--benchmark", action="store_true",
                    help="print a timing summary on exit")
    ap.add_argument("--frames", dest="max_frames", type=int, default=0,
                    help="stop after N frames (0 = unlimited)")
    ap.add_argument("--telemetry_csv", default=None,
                    help="per-frame 7-stamp telemetry CSV path")
    ap.add_argument("--with-image", dest="no_image", action="store_false",
                    default=True, help="also write the frame into /yolo_shm")
    return ap.parse_args(argv)


def main(argv=None):
    cfg = parse_args(argv)
    if cfg.benchmark and not cfg.telemetry_csv:
        cfg.telemetry_csv = "/tmp/vlm_detector_telemetry.csv"

    predictor = OwlViTPredictor(cfg.prompt, cfg.box_threshold,
                                cfg.model, cfg.threads)
    run_producer(
        cfg, predictor, label=f"owlvit:{cfg.prompt}",
        startup_line=("Producer: vlm-owlvit, resolution: {w}x{h}, "
                      f"target: '{cfg.prompt}' (class_id={VLM_CLASS_ID}), "
                      "backend: cpu"))

    if cfg.benchmark and cfg.telemetry_csv and os.path.exists(cfg.telemetry_csv):
        import csv as _csv
        with open(cfg.telemetry_csv) as f:
            rows = list(_csv.DictReader(f))
        if rows:
            det_ms = sorted(float(r["inference_ms"]) for r in rows)
            n = len(det_ms)
            print(f"[benchmark] {n} frames  "
                  f"inference p50={det_ms[n//2]:.0f}ms p95={det_ms[int(n*0.95)]:.0f}ms")


if __name__ == "__main__":
    main()

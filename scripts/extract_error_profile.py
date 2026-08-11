#!/usr/bin/env python3
"""Extract a perception error profile from a trained YOLO cone detector.

Runs OFFLINE wherever the model lives (Colab, a laptop with the weights) --
NOT on the VM, not in ROS. Produces error_profile.json, whose fields map
1:1 onto the fs_autonomy Monte-Carlo harness sensor model, plus a
P(detect | range) plot.

    pip install ultralytics
    python extract_error_profile.py --weights best.pt --data path/to/val \
        --focal-px 700 --cone-height 0.325

Expects YOLO-format validation data (the split the model was NOT trained on):
    <data>/images/*.jpg|*.png
    <data>/labels/<same stem>.txt    lines: class cx cy w h (normalized)

Range is estimated from box height via the pinhole model:
    range_m = focal_px * cone_height_m / box_height_px
NOTE: this needs the focal length of the camera that took the images. For a
single known camera, use its calibrated value. For FSOCO-style mixed-camera
data the per-image focal length is unknown -- pass a nominal value and treat
the range axis as approximate (bucket ORDER is still valid; absolute metres
shift with the true focal length). State this caveat wherever results are
reported.
"""

import argparse
import glob
import json
import os
import time


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", required=True, help="trained .pt (ultralytics)")
    ap.add_argument("--data", required=True, help="dir with images/ and labels/")
    ap.add_argument("--split", default="test",
                    help="split subdir under images//labels/ (e.g. test, val); "
                         "use '' if images/ holds files directly")
    ap.add_argument("--focal-px", type=float, default=700.0,
                    help="camera focal length in pixels (nominal if unknown)")
    ap.add_argument("--cone-height", type=float, default=0.325,
                    help="physical cone height in metres (FS small cone 0.325)")
    ap.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    ap.add_argument("--iou-match", type=float, default=0.5,
                    help="IoU above which a prediction matches a GT box")
    ap.add_argument("--buckets", default="0,2.5,5,7.5,10,12.5,15,20,30",
                    help="range bucket edges in metres, comma-separated")
    ap.add_argument("--out", default="error_profile.json")
    ap.add_argument("--limit", type=int, default=0, help="use only N images (0=all)")
    return ap.parse_args()


def iou(a, b):
    # boxes as (x1, y1, x2, y2) in pixels
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def load_gt(label_path, w, h):
    boxes = []
    if not os.path.exists(label_path):
        return boxes
    with open(label_path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 5:
                continue
            cls, cx, cy, bw, bh = int(parts[0]), *map(float, parts[1:5])
            boxes.append({
                "cls": cls,
                "xyxy": ((cx - bw / 2) * w, (cy - bh / 2) * h,
                         (cx + bw / 2) * w, (cy + bh / 2) * h),
                "h_px": bh * h,
                "cx_px": cx * w,
            })
    return boxes


def main():
    args = parse_args()
    from ultralytics import YOLO  # noqa: deferred so --help works without it
    import cv2

    model = YOLO(args.weights)
    names = model.names  # {idx: name}
    print("model classes:", names)

    edges = [float(x) for x in args.buckets.split(",")]
    n_b = len(edges) - 1
    bucket = lambda r: next((i for i in range(n_b) if edges[i] <= r < edges[i + 1]), None)

    gt_count = [0] * n_b          # GT cones per range bucket
    det_count = [0] * n_b         # ... of which detected (IoU-matched)
    color_ok = [0] * n_b          # ... of which with the correct class
    bearing_err = [[] for _ in range(n_b)]   # degrees
    range_err = [[] for _ in range(n_b)]     # fractional
    fp_total, images_used = 0, 0
    latencies = []

    img_dir = os.path.join(args.data, "images", args.split).rstrip("/")
    lbl_dir = os.path.join(args.data, "labels", args.split).rstrip("/")
    images = sorted(glob.glob(os.path.join(img_dir, "*.jpg"))
                    + sorted(glob.glob(os.path.join(img_dir, "*.png"))))
    if args.limit:
        images = images[: args.limit]
    if not images:
        raise SystemExit("no images found under %s" % img_dir)

    for img_path in images:
        img = cv2.imread(img_path)
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = os.path.splitext(os.path.basename(img_path))[0]
        gt = load_gt(os.path.join(lbl_dir, stem + ".txt"), w, h)

        t0 = time.perf_counter()
        res = model.predict(img, conf=args.conf, verbose=False)[0]
        latencies.append((time.perf_counter() - t0) * 1000.0)
        images_used += 1

        preds = []
        for b, c, k in zip(res.boxes.xyxy.tolist(),
                           res.boxes.conf.tolist(),
                           res.boxes.cls.tolist()):
            preds.append({"xyxy": tuple(b), "conf": c, "cls": int(k),
                          "h_px": b[3] - b[1], "cx_px": (b[0] + b[2]) / 2,
                          "matched": False})

        # greedy match: each GT takes its best remaining prediction by IoU
        for g in sorted(gt, key=lambda g: -g["h_px"]):
            r_gt = args.focal_px * args.cone_height / max(g["h_px"], 1e-6)
            bi = bucket(r_gt)
            if bi is None:
                continue
            gt_count[bi] += 1
            best, best_iou = None, args.iou_match
            for p in preds:
                if p["matched"]:
                    continue
                v = iou(g["xyxy"], p["xyxy"])
                if v >= best_iou:
                    best, best_iou = p, v
            if best is None:
                continue
            best["matched"] = True
            det_count[bi] += 1
            if best["cls"] == g["cls"]:
                color_ok[bi] += 1
            bearing = (best["cx_px"] - g["cx_px"]) / args.focal_px  # radians
            bearing_err[bi].append(abs(bearing) * 57.2958)
            r_pred = args.focal_px * args.cone_height / max(best["h_px"], 1e-6)
            range_err[bi].append(abs(r_pred - r_gt) / r_gt)

        fp_total += sum(1 for p in preds if not p["matched"])

    def mean(xs):
        return sum(xs) / len(xs) if xs else None

    profile = {
        "meta": {
            "weights": os.path.basename(args.weights),
            "images": images_used,
            "focal_px_assumed": args.focal_px,
            "cone_height_m": args.cone_height,
            "conf_threshold": args.conf,
            "note": "range axis approximate unless focal_px is calibrated",
        },
        "range_buckets_m": edges,
        "p_detect": [det_count[i] / gt_count[i] if gt_count[i] else None
                     for i in range(n_b)],
        "gt_per_bucket": gt_count,
        "color_accuracy": [color_ok[i] / det_count[i] if det_count[i] else None
                           for i in range(n_b)],
        "bearing_err_deg_mean": [mean(bearing_err[i]) for i in range(n_b)],
        "range_err_frac_mean": [mean(range_err[i]) for i in range(n_b)],
        "false_positives_per_image": fp_total / images_used if images_used else None,
        "latency_ms_mean_THIS_MACHINE": mean(latencies),
    }

    with open(args.out, "w") as f:
        json.dump(profile, f, indent=2)
    print(json.dumps(profile, indent=2))
    print("\nwrote", args.out)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        mids = [(edges[i] + edges[i + 1]) / 2 for i in range(n_b)]
        pd = [p if p is not None else float("nan") for p in profile["p_detect"]]
        plt.figure(figsize=(6, 3.5))
        plt.plot(mids, pd, marker="o")
        plt.xlabel("range (m, approx)")
        plt.ylabel("P(detect)")
        plt.ylim(0, 1.05)
        plt.grid(alpha=0.3)
        plt.title("Detection probability vs range")
        plt.tight_layout()
        plt.savefig(os.path.splitext(args.out)[0] + ".png", dpi=150)
        print("wrote", os.path.splitext(args.out)[0] + ".png")
    except Exception as e:  # plot is a bonus, never fail the run
        print("plot skipped:", e)


if __name__ == "__main__":
    main()

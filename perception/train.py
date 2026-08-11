"""
train.py — Train YOLOv5 on the FSOCO cone detection dataset.

Usage:
  # Convert dataset and train
  python train.py --fsoco_dir data/fsoco/amz

  # Skip conversion if already done
  python train.py --skip_convert

  # Quick smoke test (5 epochs)
  python train.py --fsoco_dir data/fsoco/amz --fast
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(ROOT))

from utils.dataset import convert


def parse_args():
    p = argparse.ArgumentParser(description="Train YOLOv5 on FSOCO")
    p.add_argument("--fsoco_dir",    type=Path, default=None,
                   help="Path to FSOCO Supervisely dataset (e.g. data/fsoco/amz)")
    p.add_argument("--data_dir",     type=Path, default=ROOT / "data" / "processed",
                   help="Where to save/read the converted dataset")
    p.add_argument("--epochs",       type=int,  default=50)
    p.add_argument("--batch_size",   type=int,  default=16)
    p.add_argument("--img_size",     type=int,  default=640)
    p.add_argument("--model",        type=str,  default="yolov5n.pt",
                   help="YOLOv5 variant: yolov5n.pt / yolov5s.pt / yolov5m.pt")
    p.add_argument("--device",       type=str,  default="auto",
                   help="'auto', 'cpu', or '0' for GPU")
    p.add_argument("--skip_convert", action="store_true",
                   help="Skip dataset conversion (use if already converted)")
    p.add_argument("--fast",         action="store_true",
                   help="Quick test: 5 epochs, batch 8")
    return p.parse_args()


def main():
    args = parse_args()

    if args.fast:
        print("⚡ Fast mode: 5 epochs, batch 8")
        args.epochs     = 5
        args.batch_size = 8

    # ── 1. Convert dataset ────────────────────────────────────────────────────
    yaml_path = args.data_dir / "dataset.yaml"

    if not args.skip_convert:
        if args.fsoco_dir is None:
            print("ERROR: --fsoco_dir is required unless --skip_convert is set.")
            sys.exit(1)
        yaml_path = convert(args.fsoco_dir, args.data_dir)
    else:
        if not yaml_path.exists():
            print(f"ERROR: {yaml_path} not found. Run without --skip_convert first.")
            sys.exit(1)
        print(f"Skipping conversion, using existing dataset at {args.data_dir}")

    # ── 2. Train ──────────────────────────────────────────────────────────────
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed. Run: pip install ultralytics")
        sys.exit(1)

    import torch
    device = args.device
    if device == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"

    print(f"\n{'='*50}")
    print(f"Training YOLOv5 on FSOCO")
    print(f"  Model:   {args.model}")
    print(f"  Device:  {device}")
    print(f"  Epochs:  {args.epochs}")
    print(f"  Batch:   {args.batch_size}")
    print(f"  ImgSize: {args.img_size}")
    print(f"{'='*50}\n")

    model = YOLO(args.model)
    model.train(
        data    = str(yaml_path),
        epochs  = args.epochs,
        imgsz   = args.img_size,
        batch   = args.batch_size,
        device  = device,
        project = "runs",
        name    = "yolov5_fsoco",
        exist_ok= True,
    )

    best = Path("runs/yolov5_fsoco/weights/best.pt")
    if best.exists():
        print(f"\n✓ Training complete. Best weights: {best}")
        print(f"  To use in ROS: copy {best} to your ROS package")
    else:
        print("\n✓ Training complete.")


if __name__ == "__main__":
    main()

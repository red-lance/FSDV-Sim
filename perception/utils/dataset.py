"""
dataset.py — Converts FSOCO Supervisely format to YOLO format.

FSOCO cone classes:
  0: blue_cone
  1: yellow_cone
  2: orange_cone
  3: large_orange_cone
  4: unknown_cone
"""

import os
import json
import shutil
import random
import zlib
import base64
from pathlib import Path
from io import BytesIO

import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm


FSOCO_CLASSES = {
    "blue_cone": 0,          "seg_blue_cone": 0,
    "yellow_cone": 1,        "seg_yellow_cone": 1,
    "orange_cone": 2,        "seg_orange_cone": 2,
    "large_orange_cone": 3,  "seg_large_orange_cone": 3,
    "unknown_cone": 4,       "seg_unknown_cone": 4,
}
CLASS_NAMES = ["blue_cone", "yellow_cone", "orange_cone", "large_orange_cone", "unknown_cone"]


def _bitmap_to_bbox(bitmap_data: dict, img_w: int, img_h: int):
    """Decode a Supervisely zlib-compressed PNG bitmap and return (x1,y1,x2,y2)."""
    try:
        raw       = base64.b64decode(bitmap_data["data"])
        png_bytes = zlib.decompress(raw)
        origin    = bitmap_data.get("origin", [0, 0])
        ox, oy    = origin[0], origin[1]

        img = Image.open(BytesIO(png_bytes)).convert("L")
        arr = np.array(img)
        rows, cols = np.where(arr > 0)
        if len(rows) == 0:
            return None

        x1 = int(cols.min()) + ox
        y1 = int(rows.min()) + oy
        x2 = int(cols.max()) + ox
        y2 = int(rows.max()) + oy
        x1, x2 = max(0, x1), min(img_w, x2)
        y1, y2 = max(0, y1), min(img_h, y2)
        return x1, y1, x2, y2
    except Exception:
        return None


def parse_annotation(ann_path: Path, img_w: int, img_h: int):
    """Parse a single Supervisely annotation JSON and return YOLO-format boxes."""
    with open(ann_path) as f:
        data = json.load(f)

    boxes = []
    for obj in data.get("objects", []):
        class_title = obj.get("classTitle", "").lower()
        class_id    = FSOCO_CLASSES.get(class_title)
        if class_id is None:
            continue

        geom = obj.get("geometryType", "")
        if geom == "bitmap":
            result = _bitmap_to_bbox(obj["bitmap"], img_w, img_h)
            if result is None:
                continue
            x1, y1, x2, y2 = result
        elif geom in ("rectangle", "polygon"):
            exterior = obj.get("points", {}).get("exterior", [])
            if not exterior:
                continue
            xs = [p[0] for p in exterior]
            ys = [p[1] for p in exterior]
            x1, x2 = max(0, min(xs)), min(img_w, max(xs))
            y1, y2 = max(0, min(ys)), min(img_h, max(ys))
        else:
            continue

        if x2 <= x1 or y2 <= y1:
            continue

        cx = ((x1 + x2) / 2) / img_w
        cy = ((y1 + y2) / 2) / img_h
        w  = (x2 - x1) / img_w
        h  = (y2 - y1) / img_h
        boxes.append((class_id, cx, cy, w, h))

    return boxes


def discover_samples(fsoco_root: Path):
    """Find all (image, annotation) pairs under an FSOCO Supervisely export."""
    fsoco_root = Path(fsoco_root).resolve()
    print(f"Searching under: {fsoco_root}")

    ann_files = [f for f in sorted(fsoco_root.rglob("*.json"))
                 if f.parent.name == "ann"]
    print(f"Found {len(ann_files)} annotation files")

    pairs = []
    for ann_file in ann_files:
        # Handle filenames like amz_00016.jpg.json
        img_stem = ann_file.stem
        for ext in (".jpg", ".jpeg", ".png", ".bmp"):
            if img_stem.lower().endswith(ext):
                img_stem = img_stem[:-len(ext)]
                break

        img_dir = ann_file.parent.parent / "img"
        img_file = None
        for ext in (".jpg", ".jpeg", ".png", ".bmp"):
            candidate = img_dir / (img_stem + ext)
            if candidate.exists():
                img_file = candidate
                break

        if img_file:
            pairs.append((img_file, ann_file))

    print(f"Matched {len(pairs)} image-annotation pairs")
    return pairs


def convert(fsoco_root: Path, output_dir: Path,
            val_ratio: float = 0.15, test_ratio: float = 0.10,
            seed: int = 42) -> Path:
    """Convert FSOCO dataset to YOLO format and return path to dataset.yaml."""
    output_dir = Path(output_dir)
    pairs = discover_samples(fsoco_root)
    if not pairs:
        raise FileNotFoundError(f"No samples found under {fsoco_root}")

    random.seed(seed)
    random.shuffle(pairs)
    n       = len(pairs)
    n_test  = max(1, int(n * test_ratio))
    n_val   = max(1, int(n * val_ratio))
    splits  = {
        "test":  pairs[:n_test],
        "val":   pairs[n_test:n_test + n_val],
        "train": pairs[n_test + n_val:],
    }
    print(f"Split — train: {len(splits['train'])}, val: {len(splits['val'])}, test: {len(splits['test'])}")

    skipped = 0
    for split, split_pairs in splits.items():
        img_out = output_dir / "images" / split
        lbl_out = output_dir / "labels" / split
        img_out.mkdir(parents=True, exist_ok=True)
        lbl_out.mkdir(parents=True, exist_ok=True)

        for img_path, ann_path in tqdm(split_pairs, desc=f"Converting {split}"):
            img = cv2.imread(str(img_path))
            if img is None:
                skipped += 1
                continue
            h, w = img.shape[:2]
            boxes = parse_annotation(ann_path, w, h)
            if not boxes:
                skipped += 1
                continue

            shutil.copy2(img_path, img_out / img_path.name)
            with open(lbl_out / (img_path.stem + ".txt"), "w") as f:
                for cls, cx, cy, bw, bh in boxes:
                    f.write(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

    if skipped:
        print(f"Skipped {skipped} samples (unreadable or empty)")

    yaml_path = output_dir / "dataset.yaml"
    yaml_path.write_text(
        f"path: {output_dir.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"test: images/test\n\n"
        f"nc: 5\n"
        f"names: {CLASS_NAMES}\n"
    )
    print(f"✓ Dataset ready at {output_dir}")
    return yaml_path

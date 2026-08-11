# YOLOv5 Cone Detection — FSOCO

YOLOv5 trained on the [FSOCO dataset](https://fsoco.github.io/fsoco-dataset/) for Formula Student cone detection.

**5 cone classes:** blue, yellow, orange, large orange, unknown

---

## Setup

```bash
pip install -r requirements.txt
```

---

## Dataset

1. Download FSOCO from https://fsoco.github.io/fsoco-dataset/download
   - Select **Segmentation** format
2. Extract to `data/fsoco/` — should look like:
```
data/fsoco/amz/
    ann/   ← JSON annotation files
    img/   ← JPG images
```

---

## Train

```bash
# Convert dataset and train (50 epochs)
python train.py --fsoco_dir data/fsoco/amz

# Quick smoke test
python train.py --fsoco_dir data/fsoco/amz --fast

# Skip conversion if already done
python train.py --skip_convert

# Larger model for more accuracy
python train.py --fsoco_dir data/fsoco/amz --model yolov5s.pt
```

Weights saved to `runs/yolov5_fsoco/weights/best.pt`

---

## Models

| Model | Speed | Accuracy |
|-------|-------|----------|
| yolov5n.pt | Fastest | Good |
| yolov5s.pt | Fast | Better |
| yolov5m.pt | Moderate | Best |

---

## Results (AMZ subset, 50 epochs, nano)

| Metric | Value |
|--------|-------|
| mAP@0.5 | 55.9% |
| FPS (CPU) | 35 |
| Latency | 28.7 ms |

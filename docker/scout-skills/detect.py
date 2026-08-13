"""YOLO11n object detection on CPU via onnxruntime — no torch in the image
(the ONNX is exported in a throwaway Docker build stage; see Dockerfile).

640-letterbox in, (1, 84, 8400) out: 4 xywh rows + 80 COCO class rows per
anchor. Decode + NMS here in numpy. ~0.5–1 s/frame on the Pi 5 — fine at
tool-call cadence, not a tracking loop.
"""

from io import BytesIO

import numpy as np
from PIL import Image, ImageDraw

MODEL_PATH = "/app/yolo11n.onnx"
INPUT_SIZE = 640

# COCO-80, index-aligned with the model's class rows.
COCO = (
    "person bicycle car motorcycle airplane bus train truck boat traffic-light "
    "fire-hydrant stop-sign parking-meter bench bird cat dog horse sheep cow "
    "elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee "
    "skis snowboard sports-ball kite baseball-bat baseball-glove skateboard "
    "surfboard tennis-racket bottle wine-glass cup fork knife spoon bowl banana "
    "apple sandwich orange broccoli carrot hot-dog pizza donut cake chair couch "
    "potted-plant bed dining-table toilet tv laptop mouse remote keyboard "
    "cell-phone microwave oven toaster sink refrigerator book clock vase "
    "scissors teddy-bear hair-drier toothbrush"
).split()

_session = None


def _get_session():
    global _session
    if _session is None:
        import onnxruntime  # deferred: ~1 s import + session build, once

        _session = onnxruntime.InferenceSession(
            MODEL_PATH, providers=["CPUExecutionProvider"]
        )
    return _session


def _letterbox(rgb: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    h, w = rgb.shape[:2]
    r = min(INPUT_SIZE / w, INPUT_SIZE / h)
    nw, nh = round(w * r), round(h * r)
    resized = np.asarray(
        Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR)
    )
    canvas = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
    dx, dy = (INPUT_SIZE - nw) // 2, (INPUT_SIZE - nh) // 2
    canvas[dy : dy + nh, dx : dx + nw] = resized
    return canvas, r, dx, dy


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> list[int]:
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[rest, 2] - boxes[rest, 0]) * (
            boxes[rest, 3] - boxes[rest, 1]
        )
        iou = inter / (area_i + area_r - inter + 1e-9)
        order = rest[iou <= iou_thr]
    return keep


def detect(rgb: np.ndarray, min_conf: float = 0.35, iou_thr: float = 0.5) -> list[dict]:
    """[{label, confidence, box: [x1, y1, x2, y2]}] in source-image pixels."""
    img, r, dx, dy = _letterbox(rgb)
    blob = img.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
    (out,) = _get_session().run(None, {"images": blob})
    pred = out[0].T  # (8400, 84)

    cls_scores = pred[:, 4:]
    cls_ids = cls_scores.argmax(1)
    confs = cls_scores[np.arange(len(cls_ids)), cls_ids]
    mask = confs >= min_conf
    if not mask.any():
        return []
    pred, cls_ids, confs = pred[mask], cls_ids[mask], confs[mask]

    # xywh (letterbox px) → xyxy (source px).
    xy, wh = pred[:, :2], pred[:, 2:4]
    boxes = np.concatenate([xy - wh / 2, xy + wh / 2], 1)
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dx) / r
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dy) / r
    h, w = rgb.shape[:2]
    boxes = boxes.clip([0, 0, 0, 0], [w - 1, h - 1, w - 1, h - 1])

    dets = []
    for cls in np.unique(cls_ids):
        idx = np.flatnonzero(cls_ids == cls)
        for k in _nms(boxes[idx], confs[idx], iou_thr):
            i = idx[k]
            dets.append(
                {
                    "label": COCO[int(cls)],
                    "confidence": round(float(confs[i]), 3),
                    "box": [round(float(v), 1) for v in boxes[i]],
                }
            )
    dets.sort(key=lambda d: -d["confidence"])
    return dets


def annotate(rgb: np.ndarray, dets: list[dict]) -> bytes:
    """PNG of the frame with labeled boxes, for the vision model / user."""
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    for d in dets:
        x1, y1, x2, y2 = d["box"]
        draw.rectangle((x1, y1, x2, y2), outline=(255, 60, 60), width=2)
        tag = f"{d['label']} {d['confidence']:.2f}"
        if d.get("distance_m") is not None:
            tag += f" @{d['distance_m']:.2f}m"
        draw.text((x1 + 2, max(0, y1 - 12)), tag, fill=(255, 60, 60))
    buf = BytesIO()
    pil.save(buf, "PNG")
    return buf.getvalue()

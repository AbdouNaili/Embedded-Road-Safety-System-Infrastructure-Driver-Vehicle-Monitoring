import multiprocessing as mp
import time
import cv2
import numpy as np

from config import (
    MODEL_PATH, MODEL_INPUT_SIZE,
    CONF_THRESHOLD_NMS, CONF_THRESHOLDS_PER_CLASS,
    NMS_IOU_THRESHOLD, MIN_BOX_AREA,
    CLASS_NAMES, CLASS_LABELS_FR,
    ROAD_CAM_INDEX,
)

CLS_COLORS: dict = {
    "chaussee_degradee":     (0,   0, 255),
    "ralentisseur":          (0, 230, 230),
    "cedez_le_passage":      (0, 140, 255),
    "passage_pietons":       (200, 200,  0),
    "stop":                  (0,   0, 200),
    "succession_de_virages": (0, 100, 255),
    "feu_rouge":             (0,  30, 230),
    "feu_vert":              (0, 200,  50),
    "vitesse_100":           (180,  0, 220),
    "vitesse_30":            (180,  0, 220),
    "vitesse_40":            (180,  0, 220),
    "vitesse_60":            (180,  0, 220),
    "vitesse_80":            (180,  0, 220),
}
_DEFAULT_COLOR = (0, 255, 120)


def _load_interpreter(path):
    """Loads the TFLite model with the lightest runtime that is installed."""
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite import Interpreter
    interp = Interpreter(model_path=path)
    interp.allocate_tensors()
    return interp


def _preprocess(frame, size, in_scale, in_zero, in_dtype):
    img = cv2.resize(frame, (size, size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.expand_dims(img, 0)
    if in_scale != 0:
        # quantized model: round and keep the values inside the int range
        img = np.round(img / in_scale + in_zero)
        info = np.iinfo(in_dtype)
        img = np.clip(img, info.min, info.max)
    return img.astype(in_dtype)


def _infer(frame, interp, in_idx, out_idx,
           in_scale, in_zero, in_dtype, out_scale, out_zero) -> list:
    tensor = _preprocess(frame, MODEL_INPUT_SIZE,
                         in_scale, in_zero, in_dtype)
    interp.set_tensor(in_idx, tensor)
    interp.invoke()
    raw  = interp.get_tensor(out_idx)
    h, w = frame.shape[:2]

    preds = raw.astype(np.float32)
    if out_scale != 0:
        preds = (preds - out_zero) * out_scale
    preds = preds[0]

    # YOLOv8 output is (4 + classes, anchors): one column per anchor
    n_cls = len(CLASS_NAMES)
    if preds.shape[0] == 4 + n_cls:
        preds = preds.T
    if preds.shape[1] != 4 + n_cls:
        raise ValueError(
            f"model has {preds.shape[1] - 4} classes but config.CLASS_NAMES has {n_cls}")

    # All anchors at once (much faster than a Python loop on the Pi)
    cls_ids = np.argmax(preds[:, 4:], axis=1)
    confs   = preds[np.arange(len(preds)), 4 + cls_ids]

    thresholds = np.array([CONF_THRESHOLDS_PER_CLASS.get(CLASS_NAMES[i], CONF_THRESHOLD_NMS)
                           for i in range(n_cls)])
    keep = (confs >= CONF_THRESHOLD_NMS) & (confs >= thresholds[cls_ids])
    if not keep.any():
        return []

    xywh    = preds[keep, :4]
    confs   = confs[keep]
    cls_ids = cls_ids[keep]

    # normalized center/size -> pixel corners
    x1 = ((xywh[:, 0] - xywh[:, 2] / 2) * w).astype(int)
    y1 = ((xywh[:, 1] - xywh[:, 3] / 2) * h).astype(int)
    x2 = ((xywh[:, 0] + xywh[:, 2] / 2) * w).astype(int)
    y2 = ((xywh[:, 1] + xywh[:, 3] / 2) * h).astype(int)
    big = (np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)) >= MIN_BOX_AREA
    if not big.any():
        return []

    x1, y1, x2, y2 = x1[big], y1[big], x2[big], y2[big]
    confs, cls_ids = confs[big], cls_ids[big]

    # cv2 NMS expects boxes as [x, y, width, height]
    rects   = [[int(a), int(b), int(c - a), int(d - b)] for a, b, c, d in zip(x1, y1, x2, y2)]
    indices = cv2.dnn.NMSBoxes(rects, confs.tolist(), CONF_THRESHOLD_NMS, NMS_IOU_THRESHOLD)

    dets = []
    for i in np.array(indices).flatten():
        cls_key = CLASS_NAMES[int(cls_ids[i])]
        dets.append({
            "class":    cls_key,
            "label_fr": CLASS_LABELS_FR.get(cls_key, cls_key),
            "conf":     float(confs[i]),
            "box":      [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
        })
    return dets


def _annotate(frame, dets, fps, lidar_anomaly, lidar_dist):
    h, w = frame.shape[:2]

    for det in dets:
        x1, y1, x2, y2 = det["box"]
        x1 = max(0, x1);  y1 = max(0, y1)
        x2 = min(w-1, x2); y2 = min(h-1, y2)
        cls   = det["class"]
        color = CLS_COLORS.get(cls, _DEFAULT_COLOR)

        prefix = "WARNING " if cls == "chaussee_degradee" else ""
        label  = f"{prefix}{det['label_fr']}  {det['conf']:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        lbl_y0 = max(0, y1 - th - 6)
        cv2.rectangle(frame, (x1, lbl_y0), (x1 + tw + 4, y1), color, -1)
        cv2.putText(frame, label,
                    (x1 + 2, max(th + 2, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (10, 10, 10), 1, cv2.LINE_AA)

        if (lidar_anomaly and lidar_dist is not None
                and lidar_anomaly == cls
                and cls in ("chaussee_degradee", "ralentisseur")):
            cx = (x1 + x2) // 2
            ay = h - 10
            cv2.arrowedLine(frame, (cx, ay), (cx, y2 + 4),
                            (0, 0, 230), 2, tipLength=0.15)
            dist_lbl = f"{lidar_dist} cm"
            (dw, dh), _ = cv2.getTextSize(
                dist_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            dx = cx + 8
            dy = (ay + y2) // 2
            cv2.rectangle(frame,
                          (dx - 2, dy - dh - 2), (dx + dw + 4, dy + 4),
                          (0, 0, 0), -1)
            cv2.putText(frame, dist_lbl, (dx, dy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 40, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, f"FPS {int(fps)}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 230, 255), 2, cv2.LINE_AA)
    return frame


def _road_worker(frame_q: mp.Queue, det_q: mp.Queue,
                 lidar_q: mp.Queue, stop_ev: mp.Event):
    model_ok = False
    interp = in_idx = out_idx = None
    in_scale = in_zero = out_scale = out_zero = 0
    in_dtype = None

    try:
        interp   = _load_interpreter(MODEL_PATH)
        ind      = interp.get_input_details()
        outd     = interp.get_output_details()
        in_idx   = ind[0]["index"]
        out_idx  = outd[0]["index"]
        in_scale,  in_zero  = ind[0]["quantization"]
        out_scale, out_zero = outd[0]["quantization"]
        in_dtype = ind[0]["dtype"]
        n_out    = min(outd[0]["shape"][1:])
        if n_out != 4 + len(CLASS_NAMES):
            raise ValueError(
                f"le modèle a {n_out - 4} classes, config.CLASS_NAMES en a {len(CLASS_NAMES)}")
        model_ok = True
        print("[RoadWorker] Modèle TFLite chargé.")
    except Exception as e:
        print(f"[RoadWorker] Modèle non chargé : {e}")

    cap = cv2.VideoCapture(ROAD_CAM_INDEX)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    lidar_anomaly = None
    lidar_dist    = None
    prev_t = time.time()

    while not stop_ev.is_set():
        try:
            while True:
                lidar_anomaly, lidar_dist = lidar_q.get_nowait()
        except Exception:
            pass

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        now    = time.time()
        fps    = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now

        dets = []
        if model_ok:
            try:
                dets = _infer(frame, interp, in_idx, out_idx,
                              in_scale, in_zero, in_dtype,
                              out_scale, out_zero)
            except Exception as e:
                print(f"[RoadWorker] Inférence erreur : {e}")

        annotated = _annotate(frame.copy(), dets, fps,
                              lidar_anomaly, lidar_dist)

        ok, buf = cv2.imencode(".jpg", annotated,
                               [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            try:
                if frame_q.full():
                    frame_q.get_nowait()
                frame_q.put_nowait(buf.tobytes())
            except Exception:
                pass

        if dets:
            det_list = [{"class": d["class"], "conf": d["conf"]}
                        for d in dets]
            try:
                if det_q.full():
                    det_q.get_nowait()
                det_q.put_nowait(det_list)
            except Exception:
                pass

        time.sleep(0.008)

    cap.release()


class RoadDetector:

    def __init__(self):
        self._frame_q    = mp.Queue(maxsize=2)
        self._det_q      = mp.Queue(maxsize=6)
        self._lidar_q    = mp.Queue(maxsize=4)
        self._stop_ev    = mp.Event()
        self._proc       = None
        self._last_frame = None

    def start(self):
        self._proc = mp.Process(
            target=_road_worker,
            args=(self._frame_q, self._det_q,
                  self._lidar_q, self._stop_ev),
            daemon=True)
        self._proc.start()

    def stop(self):
        self._stop_ev.set()
        if self._proc and self._proc.is_alive():
            self._proc.join(timeout=3)

    def set_lidar(self, anomaly, dist_cm):
        try:
            if self._lidar_q.full():
                self._lidar_q.get_nowait()
            self._lidar_q.put_nowait((anomaly, dist_cm))
        except Exception:
            pass

    def get_latest_frame(self):
        jpeg = None
        try:
            while True:
                jpeg = self._frame_q.get_nowait()
        except Exception:
            pass
        if jpeg is not None:
            buf = np.frombuffer(jpeg, dtype=np.uint8)
            f   = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if f is not None:
                self._last_frame = f
        return self._last_frame

    def pop_detections(self) -> list:
        latest: list = []
        try:
            while True:
                latest = self._det_q.get_nowait()
        except Exception:
            pass
        return latest

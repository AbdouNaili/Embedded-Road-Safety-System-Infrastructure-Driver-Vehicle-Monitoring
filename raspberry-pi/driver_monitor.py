import multiprocessing as mp
import time
import cv2
import numpy as np

from config import (
    DRIVER_CAM_INDEX,
    CALIBRATION_TIME,
    EAR_DROP, EAR_THRESH_MIN, EAR_THRESH_MAX,
    SLEEP_TIME_THRESH,
    MAR_THRESHOLD, YAWN_TIME_THRESH,
    EAR_SMOOTHING, MAR_SMOOTHING,
    LEFT_EYE, RIGHT_EYE,
)

FACE_LOST_RESET_FRAMES = 15
MIN_CALIB_SAMPLES      = 15


def _dist(p1, p2) -> float:
    return float(np.linalg.norm(np.array(p1) - np.array(p2)))


def _ear(landmarks, indices: list, w: int, h: int) -> float:
    pts = [(landmarks[i].x * w, landmarks[i].y * h) for i in indices]
    A = _dist(pts[1], pts[5])
    B = _dist(pts[2], pts[4])
    C = _dist(pts[0], pts[3])
    return (A + B) / (2.0 * C) if C > 1e-6 else 0.0


def _mar(landmarks, w: int, h: int) -> float:
    upper = (landmarks[13].x  * w, landmarks[13].y  * h)
    lower = (landmarks[14].x  * w, landmarks[14].y  * h)
    left  = (landmarks[78].x  * w, landmarks[78].y  * h)
    right = (landmarks[308].x * w, landmarks[308].y * h)
    v  = _dist(upper, lower)
    hz = _dist(left,  right)
    return v / hz if hz > 1e-6 else 0.0


def _q_put(q: mp.Queue, item):
    try:
        if q.full():
            q.get_nowait()
        q.put_nowait(item)
    except Exception:
        pass


def _driver_worker(frame_q, state_q, alert_q, stop_ev):
    import mediapipe as mp_lib

    face_mesh = mp_lib.solutions.face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(DRIVER_CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Calibration
    ear_baseline  = None
    ear_thresh    = 0.0
    calib_start   = None
    calib_samples = []

    # Buffers de lissage
    ear_buf = []
    mar_buf = []

    # Compteurs
    face_lost_frames = 0

    # Timers
    eye_closed_since = None
    sleep_alerted    = False
    yawn_since       = None
    yawn_alerted     = False

    # FPS
    prev_t  = time.time()
    fps_buf = []

    # Valeurs publiées
    ear_val = 0.0
    mar_val = 0.0
    status  = "Calibration"

    while not stop_ev.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame   = cv2.flip(frame, 1)
        display = frame.copy()
        h_dis, w_dis = display.shape[:2]

        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb)

        face_detected = (
            results is not None
            and results.multi_face_landmarks is not None
            and len(results.multi_face_landmarks) > 0
        )

        now = time.time()

        if face_detected:
            face_lost_frames = 0
            lm = results.multi_face_landmarks[0].landmark

            raw_ear = (
                _ear(lm, LEFT_EYE,  w_dis, h_dis)
                + _ear(lm, RIGHT_EYE, w_dis, h_dis)
            ) / 2.0
            raw_mar = _mar(lm, w_dis, h_dis)

            ear_buf.append(raw_ear)
            if len(ear_buf) > EAR_SMOOTHING:
                ear_buf.pop(0)

            mar_buf.append(raw_mar)
            if len(mar_buf) > MAR_SMOOTHING:
                mar_buf.pop(0)

            ear_val = float(np.mean(ear_buf))
            mar_val = float(np.mean(mar_buf))

            # Landmarks
            eye_col   = (0, 255, 100)
            mouth_col = (0, 200, 255)

            def pt(idx):
                return (int(lm[idx].x * w_dis), int(lm[idx].y * h_dis))

            lpts = [pt(i) for i in LEFT_EYE]
            rpts = [pt(i) for i in RIGHT_EYE]
            cv2.polylines(display, [np.array(lpts)], True, eye_col,   1)
            cv2.polylines(display, [np.array(rpts)], True, eye_col,   1)
            for i in LEFT_EYE + RIGHT_EYE:
                cv2.circle(display, pt(i), 2, eye_col, -1)

            mpts = [pt(i) for i in [78, 13, 308, 14]]
            cv2.polylines(display, [np.array(mpts)], True, mouth_col, 1)
            for i in [13, 14, 78, 308]:
                cv2.circle(display, pt(i), 3, mouth_col, -1)

            # Calibration
            if ear_baseline is None:
                if calib_start is None:
                    calib_start = now

                calib_samples.append(raw_ear)
                elapsed  = now - calib_start
                progress = min(elapsed / CALIBRATION_TIME, 1.0)

                bx     = w_dis // 5
                by     = h_dis // 2
                bw_max = int(w_dis * 0.6)
                bw_cur = int(bw_max * progress)
                cv2.rectangle(display,
                              (bx, by - 12), (bx + bw_max, by + 12),
                              (40, 40, 40), -1)
                cv2.rectangle(display,
                              (bx, by - 12), (bx + bw_cur, by + 12),
                              (0, 210, 110), -1)
                cv2.putText(display,
                            f"Calibration ... {int(progress * 100)}%",
                            (bx, by - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 210, 110), 2)

                if (elapsed >= CALIBRATION_TIME
                        and len(calib_samples) >= MIN_CALIB_SAMPLES):
                    ear_baseline = float(np.median(calib_samples))
                    raw_thresh   = ear_baseline - EAR_DROP
                    ear_thresh   = float(
                        max(EAR_THRESH_MIN, min(EAR_THRESH_MAX, raw_thresh))
                    )
                    print(
                        f"[Driver] baseline={ear_baseline:.4f}  "
                        f"seuil={ear_thresh:.4f}  "
                        f"({len(calib_samples)} échantillons)"
                    )

                status = "Calibration"

            else:
                # Somnolence
                if ear_val < ear_thresh:
                    if eye_closed_since is None:
                        eye_closed_since = now
                    else:
                        closed_dur = now - eye_closed_since
                        if closed_dur >= SLEEP_TIME_THRESH:
                            status = "SOMNOLENCE DETECTEE"
                            if not sleep_alerted:
                                sleep_alerted = True
                                _q_put(alert_q, "sleep")
                else:
                    eye_closed_since = None
                    sleep_alerted    = False
                    if status == "SOMNOLENCE DETECTEE":
                        status = "Eveille"

                if status != "SOMNOLENCE DETECTEE":
                    status = "Eveille"

                # Bâillement
                if mar_val > MAR_THRESHOLD:
                    if yawn_since is None:
                        yawn_since = now
                    elif now - yawn_since >= YAWN_TIME_THRESH:
                        if status == "Eveille":
                            status = "BAILLEMENT DETECTE"
                        if not yawn_alerted:
                            yawn_alerted = True
                            _q_put(alert_q, "yawn")
                else:
                    yawn_since   = None
                    yawn_alerted = False

        else:
            face_lost_frames += 1
            if face_lost_frames >= FACE_LOST_RESET_FRAMES:
                ear_buf.clear()
                mar_buf.clear()
                eye_closed_since = None
                yawn_since       = None
                sleep_alerted    = False
                yawn_alerted     = False
                ear_val          = 0.0
                mar_val          = 0.0

            # Head down or turned away for a while: do not report "awake"
            if ear_baseline is None:
                status = "Calibration"
            elif face_lost_frames >= FACE_LOST_RESET_FRAMES:
                status = "Visage non detecte"

            cv2.putText(display,
                        "Visage non detecte",
                        (w_dis // 2 - 100, h_dis // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 100, 200), 2, cv2.LINE_AA)

        curr_t = time.time()
        fps_buf.append(1.0 / max(curr_t - prev_t, 1e-6))
        if len(fps_buf) > 10:
            fps_buf.pop(0)
        prev_t  = curr_t
        fps_val = int(np.mean(fps_buf))

        cv2.rectangle(display, (0, 0), (w_dis, 38), (0, 0, 0), -1)
        cv2.putText(display, f"EAR: {ear_val:.3f}",
                    (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.70,
                    (0, 230, 255), 2)
        cv2.putText(display, f"MAR: {mar_val:.3f}",
                    (w_dis // 2 - 65, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.70,
                    (0, 230, 255), 2)
        cv2.putText(display, f"FPS: {fps_val}",
                    (w_dis - 110, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.70,
                    (0, 255, 0), 2)

        # Publication
        ok, buf = cv2.imencode(".jpg", display,
                               [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            _q_put(frame_q, buf.tobytes())

        _q_put(state_q, {
            "status":    status,
            "ear":       ear_val,
            "mar":       mar_val,
            "baseline":  ear_baseline,
            "threshold": ear_thresh,
        })

        time.sleep(0.010)

    cap.release()
    face_mesh.close()


class DriverMonitor:

    def __init__(self):
        self._frame_q = mp.Queue(maxsize=2)
        self._state_q = mp.Queue(maxsize=4)
        self._alert_q = mp.Queue(maxsize=8)
        self._stop_ev = mp.Event()
        self._proc    = None

        self._last_frame = None
        self._last_state = {
            "status":    "Calibration",
            "ear":       0.0,
            "mar":       0.0,
            "baseline":  None,
            "threshold": 0.0,
        }

    def start(self):
        self._proc = mp.Process(
            target=_driver_worker,
            args=(self._frame_q, self._state_q,
                  self._alert_q, self._stop_ev),
            daemon=True)
        self._proc.start()

    def stop(self):
        self._stop_ev.set()
        if self._proc and self._proc.is_alive():
            self._proc.join(timeout=3)

    def get_frame_and_state(self):
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

        try:
            while True:
                self._last_state = self._state_q.get_nowait()
        except Exception:
            pass

        return self._last_frame, dict(self._last_state)

    def pop_alerts(self):
        alerts = []
        try:
            while True:
                alerts.append(self._alert_q.get_nowait())
        except Exception:
            pass
        return alerts

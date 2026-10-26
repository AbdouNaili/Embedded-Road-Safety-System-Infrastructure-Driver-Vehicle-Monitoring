import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import os
import time
import config as _cfg

from road_detector  import RoadDetector
from driver_monitor import DriverMonitor
from fault_detector import FaultDetector
from audio_manager  import audio


OBD_DISPLAY = [
    ("RPM",       "Régime moteur",   "tr/mn"),
    ("Pedal%",    "Position pédale", "%"),
    ("CoolantC",  "Temp. liquide",   "°C"),
    ("ECU_V",     "Tension ECU",     "V"),
    ("FuelTgt",   "Carb. cible",     ""),
    ("FuelAct",   "Carb. réel",      ""),
    ("Boost",     "Pression boost",  ""),
    ("DPF_mbar",  "Pression FAP",    ""),
    ("Speed",     "Vitesse",         "km/h"),
    ("IntakeC",   "Temp. admission", "°C"),
    ("Torque",    "Couple moteur",   "Nm"),
    ("Brake_bar", "Pression frein",  "bar"),
]
_UNIT_MAP = {k: u for k, _, u in OBD_DISPLAY}

DRIVER_COLORS = {
    "Calibration":         "#FFD600",
    "Eveille":             "#00E676",
    "SOMNOLENCE DETECTEE": "#FF1744",
    "BAILLEMENT DETECTE":  "#FF6B35",
    "Visage non detecte":  "#FF9800",
}

_ICON_W      = 90
_ICON_H      = 90
_ICON_MARGIN = 6
_SPD_LBL_H   = 18

_FACE_BAND_H  = 34
_HANDS_BAND_H = 26

_POPUP_BG = "#0A0018"

AUDIO_CONF_THRESHOLD = 0.55


AUDIO_PERSIST_FRAMES = 5


EMPTY_TICK_TOLERANCE = 3

# Seconds without a LiDAR message before the anomaly is cleared
LIDAR_CLEAR_S = 1.0


class RoadSafetyApp(tk.Frame):

    def __init__(self, master):
        super().__init__(master, bg=_cfg.C_BG)
        self._master = master

        SW    = _cfg.SCREEN_W
        SH    = _cfg.SCREEN_H
        TOP_H = _cfg.TOP_H
        BOT_H = _cfg.BOT_H
        WL    = _cfg.HALF_W_L
        WR    = _cfg.HALF_W_R
        HDR   = 20

        self._RP_X = 1;       self._RP_Y = 1
        self._RP_W = WL - 2;  self._RP_H = TOP_H - 2
        self._RV_H = self._RP_H - HDR

        self._DP_X = WL + 1;  self._DP_Y = 1
        self._DP_W = WR - 2;  self._DP_H = TOP_H - 2
        self._DV_H = self._DP_H - HDR - _FACE_BAND_H - _HANDS_BAND_H

        self._OP_X = 1;       self._OP_Y = TOP_H + 1
        self._OP_W = WL - 2;  self._OP_H = BOT_H - 2

        self._FP_X = WL + 1;  self._FP_Y = TOP_H + 1
        self._FP_W = WR - 2;  self._FP_H = BOT_H - 2

        N_ROWS       = len(OBD_DISPLAY) // 2
        self._CELL_W = (self._OP_W - 2) // 2
        self._CELL_H = max(20, (self._OP_H - HDR) // N_ROWS)
        self._HDR    = HDR
        self._SW     = SW
        self._SH     = SH

        self._serial = getattr(master, "_serial_manager", None)
        if self._serial is None:
            from serial_manager import SerialManager
            self._serial = SerialManager()
            self._serial.start()

        self._road_det  = RoadDetector()
        self._drv_mon   = DriverMonitor()
        self._fault_det = FaultDetector()


        self._obd_data    : dict = {}
        self._faults      : list = []
        self._prev_faults : set  = set()

        self._hands_off : bool = False


        self._popup_active : bool = False
        self._popup_timer         = None

        self._current_speed_lim : int | None  = None
        self._speed_warn_active : bool        = False
        self._icon_cache        : dict        = {}
        self._pending_speed_cls : str | None  = None

        self._audio_last : dict = {}

        self._lidar_anomaly : str | None = None
        self._lidar_time    : float      = 0.0

        self._det_persist : dict = {}   
        self._empty_ticks : int  = 0   

        self._load_icons()
        self._build_ui()
        self._road_det.start()
        self._drv_mon.start()

        self._loop_road()
        self._loop_driver()
        self._loop_obd()
        self._loop_serial()
        self._loop_speed_check()

    def _load_icons(self):
        for cls_name, limit in _cfg.SPEED_LIMIT_CLASSES.items():
            path = os.path.join(_cfg.SIGN_ICONS_DIR, f"vitesse_{limit}.png")
            if not os.path.exists(path):
                print(f"[App] Icône manquante : {path}")
                continue
            try:
                img = Image.open(path).convert("RGBA")
                img = img.resize((_ICON_W, _ICON_H), Image.LANCZOS)
                self._icon_cache[limit] = ImageTk.PhotoImage(img)
            except Exception as e:
                print(f"[App] Erreur chargement {path} : {e}")


    def _build_ui(self):
        C   = _cfg
        HDR = self._HDR

        root = tk.Frame(self, bg=C.C_BG)
        root.place(x=0, y=0, width=self._SW, height=self._SH)

        road_p = tk.Frame(root, bg=C.C_PANEL,
                          highlightbackground=C.C_BORDER,
                          highlightthickness=1)
        road_p.place(x=self._RP_X, y=self._RP_Y,
                     width=self._RP_W, height=self._RP_H)

        self._road_title = tk.Label(
            road_p,
            text="DÉTECTION INFRASTRUCTURE ROUTIÈRE",
            bg=C.C_PANEL, fg=C.C_ACCENT,
            font=("Courier", 9, "bold"),
            anchor="w", padx=6)
        self._road_title.place(x=0, y=0, width=self._RP_W, height=HDR)

        self._road_lbl = tk.Label(road_p, bg="#000000")
        self._road_lbl.place(x=0, y=HDR, width=self._RP_W, height=self._RV_H)

        _OVL_W = _ICON_W + 2 * _ICON_MARGIN
        _OVL_H = _ICON_H + _SPD_LBL_H + 2 * _ICON_MARGIN + 2
        _OVL_X = _ICON_MARGIN
        _OVL_Y = HDR + _ICON_MARGIN
        self._SIGN_BOX_POS = (_OVL_X, _OVL_Y, _OVL_W, _OVL_H)

        self._sign_box = tk.Frame(
            road_p, bg="#06111F",
            highlightbackground=C.C_BORDER,
            highlightthickness=1)
        self._sign_box.place_forget()

        self._sign_icon_lbl = tk.Label(self._sign_box, bg="#06111F",
                                       anchor="center")
        self._sign_icon_lbl.place(x=_ICON_MARGIN, y=_ICON_MARGIN,
                                  width=_ICON_W, height=_ICON_H)

        self._sign_spd_lbl = tk.Label(
            self._sign_box, text="",
            bg="#06111F", fg=C.C_SUCCESS,
            font=("Courier", 8, "bold"), anchor="center")
        self._sign_spd_lbl.place(x=0, y=_ICON_H + _ICON_MARGIN + 2,
                                 width=_OVL_W, height=_SPD_LBL_H)

        # ── Panneau conducteur — haut droite ──────────────────
        drv_p = tk.Frame(root, bg=C.C_PANEL,
                         highlightbackground=C.C_BORDER,
                         highlightthickness=1)
        drv_p.place(x=self._DP_X, y=self._DP_Y,
                    width=self._DP_W, height=self._DP_H)

        tk.Label(drv_p, text="SURVEILLANCE CONDUCTEUR",
                 bg=C.C_PANEL, fg=C.C_ACCENT,
                 font=("Courier", 9, "bold"),
                 anchor="w", padx=6
                 ).place(x=0, y=0, width=self._DP_W, height=HDR)

        self._drv_lbl = tk.Label(drv_p, bg="#000000")
        self._drv_lbl.place(x=0, y=HDR,
                            width=self._DP_W, height=self._DV_H)

        face_band_y = HDR + self._DV_H
        self._drv_state = tk.Label(
            drv_p,
            text="État conducteur :  Calibration",
            bg="#060E1C", fg=C.C_WARNING,
            font=("Courier", 11, "bold"), anchor="center")
        self._drv_state.place(x=0, y=face_band_y,
                              width=self._DP_W, height=_FACE_BAND_H)

        hands_band_y = face_band_y + _FACE_BAND_H
        self._hands_lbl = tk.Label(
            drv_p,
            text="✓  Mains sur le volant",
            bg="#04120A", fg=C.C_SUCCESS,
            font=("Courier", 9, "bold"), anchor="center")
        self._hands_lbl.place(x=0, y=hands_band_y,
                              width=self._DP_W, height=_HANDS_BAND_H)
        self._hands_lbl.lift()

        obd_p = tk.Frame(root, bg=C.C_PANEL,
                         highlightbackground=C.C_BORDER,
                         highlightthickness=1)
        obd_p.place(x=self._OP_X, y=self._OP_Y,
                    width=self._OP_W, height=self._OP_H)

        tk.Label(obd_p, text="PARAMÈTRES VÉHICULE — OBD-II",
                 bg=C.C_PANEL, fg=C.C_ACCENT,
                 font=("Courier", 8, "bold"),
                 anchor="w", padx=6
                 ).place(x=0, y=0, width=self._OP_W, height=HDR)

        self._obd_frame = obd_p
        self._obd_vals  : dict = {}
        self._build_obd_table()

        diag_p = tk.Frame(root, bg=C.C_PANEL,
                          highlightbackground=C.C_BORDER,
                          highlightthickness=1)
        diag_p.place(x=self._FP_X, y=self._FP_Y,
                     width=self._FP_W, height=self._FP_H)

        tk.Label(diag_p, text="DIAGNOSTIC TEMPS RÉEL",
                 bg=C.C_PANEL, fg=C.C_ACCENT,
                 font=("Courier", 8, "bold"),
                 anchor="w", padx=6
                 ).place(x=0, y=0, width=self._FP_W, height=HDR)

        self._fault_canvas = tk.Canvas(diag_p, bg=C.C_PANEL,
                                       highlightthickness=0)
        self._fault_canvas.place(x=0, y=HDR,
                                 width=self._FP_W,
                                 height=self._FP_H - HDR)

    def _build_obd_table(self):
        C     = _cfg
        HDR   = self._HDR
        CW    = self._CELL_W
        CH    = self._CELL_H
        lbl_w = int(CW * 0.60)
        val_w = CW - lbl_w - 1

        for i, (key, label, _) in enumerate(OBD_DISPLAY):
            col = i % 2
            row = i // 2
            x   = col * CW + 1
            y   = HDR + row * CH
            bg  = C.C_PANEL if row % 2 == 0 else C.C_PANEL_ALT

            cell = tk.Frame(self._obd_frame, bg=bg, width=CW, height=CH)
            cell.place(x=x, y=y)
            cell.pack_propagate(False)

            tk.Label(cell, text=label,
                     bg=bg, fg=C.C_TEXT_LBL,
                     font=("Courier", 7, "bold"),
                     anchor="w", padx=3
                     ).place(x=0, y=0, width=lbl_w, height=CH)

            tk.Frame(cell, bg=C.C_BORDER
                     ).place(x=lbl_w, y=2, width=1, height=CH - 4)

            val_lbl = tk.Label(cell, text="N/A",
                               bg=bg, fg=C.C_TEXT_DIM,
                               font=("Courier", 8, "bold"),
                               anchor="e", padx=3)
            val_lbl.place(x=lbl_w + 1, y=0, width=val_w, height=CH)
            self._obd_vals[key] = val_lbl



    def _loop_road(self):
        frame = self._road_det.get_latest_frame()
        if frame is not None:
            self._show_frame(self._road_lbl, frame,
                             self._RP_W, self._RV_H)

        dets = self._road_det.pop_detections()
        if dets:
            self._empty_ticks = 0
            self._handle_detections(dets)
        else:

            self._empty_ticks += 1
            if self._empty_ticks > EMPTY_TICK_TOLERANCE:
                self._det_persist.clear()
                self._empty_ticks = 0

        self.after(28, self._loop_road)

    def _loop_driver(self):
 
        frame, state = self._drv_mon.get_frame_and_state()
        if frame is not None:
            self._show_frame(self._drv_lbl, frame,
                             self._DP_W, self._DV_H)

        status = state.get("status", "Calibration")
        color  = DRIVER_COLORS.get(status, _cfg.C_TEXT)
        self._drv_state.config(
            text=f"État conducteur :  {status}", fg=color)

        for alert in self._drv_mon.pop_alerts():
            if alert == "sleep":
                self._play_once("sleep", _cfg.SIGN_SPEAK_COOLDOWN)
                self._show_popup("SOMNOLENCE")
            elif alert == "yawn":
                self._play_once("yawn", _cfg.SIGN_SPEAK_COOLDOWN)
                self._show_popup("BÂILLEMENT")

        self.after(28, self._loop_driver)

    def _loop_obd(self):
        for key, widget in self._obd_vals.items():
            val  = self._obd_data.get(key)
            unit = _UNIT_MAP.get(key, "")
            if val is None:
                widget.config(text="N/A", fg=_cfg.C_TEXT_DIM)
            else:
                try:
                    txt = f"{val:.1f}" + (f" {unit}" if unit else "")
                    widget.config(text=txt, fg=_cfg.C_VAL_OK)
                except Exception:
                    widget.config(text="—", fg=_cfg.C_TEXT_DIM)
        self._render_faults()
        self.after(200, self._loop_obd)

    def _loop_serial(self):
        try:
            for _ in range(50):
                msg = self._serial.queue.get_nowait()
                self._handle_serial(msg)
        except Exception:
            pass

        # The ESP32 only sends a message while it sees an anomaly.
        # No message for a while = the road is normal again.
        if (self._lidar_anomaly is not None
                and time.time() - self._lidar_time > LIDAR_CLEAR_S):
            self._handle_serial({"type": "infra", "anomaly": "NORMAL"})

        self.after(28, self._loop_serial)

    def _loop_speed_check(self):

        if self._current_speed_lim is not None:
            spd = self._obd_data.get("Speed")
            if spd is not None:
                over = spd > self._current_speed_lim + _cfg.SPEED_LIMIT_TOLERANCE
                if over and not self._speed_warn_active:
                    self._speed_warn_active = True
                    self._sign_spd_lbl.config(
                        text=f"{int(spd)} > {self._current_speed_lim} km/h",
                        fg=_cfg.C_DANGER)
                    self._play_once("vitesse_depassee", 10.0)
                    self._show_popup("VITESSE", speed=spd,
                                     limit=self._current_speed_lim)
                elif not over:
                    self._speed_warn_active = False
                    self._sign_spd_lbl.config(
                        text=f"{int(spd)} km/h",
                        fg=_cfg.C_SUCCESS)
            else:
                self._sign_spd_lbl.config(
                    text=f"Limite {self._current_speed_lim} km/h",
                    fg=_cfg.C_TEXT_LBL)

        self.after(200, self._loop_speed_check)


    def _handle_detections(self, dets: list):

        best: dict = {}
        for det in dets:
            cls  = det["class"]
            prev = best.get(cls)
            if prev is None or det["conf"] > prev["conf"]:
                best[cls] = det


        all_audio_cls = (
            set(_cfg.SPEED_LIMIT_CLASSES)
            | _cfg.AUDIO_ONLY_CLASSES
            | {"chaussee_degradee", "ralentisseur"}
        )
        audio_eligible: set = set()

        for cls in all_audio_cls:
            det = best.get(cls)
            if det is not None and det["conf"] >= AUDIO_CONF_THRESHOLD:
                self._det_persist[cls] = self._det_persist.get(cls, 0) + 1
                if self._det_persist[cls] >= AUDIO_PERSIST_FRAMES:
                    audio_eligible.add(cls)
            else:
                self._det_persist[cls] = 0   

        found_limit = None
        for cls in best:
            if cls in _cfg.SPEED_LIMIT_CLASSES:
                lim = _cfg.SPEED_LIMIT_CLASSES[cls]
                if found_limit is None or lim < found_limit:
                    found_limit = lim

        if found_limit is not None:
            spd_cls = f"vitesse_{found_limit}"

            if found_limit != self._current_speed_lim:
                self._current_speed_lim = found_limit
                self._speed_warn_active = False
                self._update_speed_overlay(found_limit)
                self._pending_speed_cls = spd_cls
            else:
                self._ensure_overlay_visible()


            if (self._pending_speed_cls == spd_cls
                    and spd_cls in audio_eligible):
                self._play_once(spd_cls, _cfg.SIGN_SPEAK_COOLDOWN)
                self._pending_speed_cls = None

        for cls in _cfg.AUDIO_ONLY_CLASSES:
            if cls in audio_eligible:
                self._play_once(cls, _cfg.SIGN_SPEAK_COOLDOWN)

        for cls in ("chaussee_degradee", "ralentisseur"):
            if cls in audio_eligible and self._lidar_anomaly == cls:
                self._play_once(cls, _cfg.SIGN_SPEAK_COOLDOWN)


    def _update_speed_overlay(self, limit: int):

        img = self._icon_cache.get(limit)
        if img:
            self._sign_icon_lbl.config(image=img)
            self._sign_icon_lbl.imgtk = img
        self._sign_spd_lbl.config(
            text=f"Limite {limit} km/h", fg=_cfg.C_TEXT_LBL)
        bx, by, bw, bh = self._SIGN_BOX_POS
        self._sign_box.place(x=bx, y=by, width=bw, height=bh)
        self._sign_box.lift()

    def _ensure_overlay_visible(self):

        bx, by, bw, bh = self._SIGN_BOX_POS
        self._sign_box.place(x=bx, y=by, width=bw, height=bh)
        self._sign_box.lift()


    def _handle_serial(self, msg: dict):

        C  = _cfg
        mt = msg.get("type")


        if mt == "infra":
            anom = msg.get("anomaly", "NORMAL")
            dist = msg.get("dist_cm")
            if anom in ("chaussee_degradee", "ralentisseur"):
                self._lidar_anomaly = anom
                self._lidar_time    = time.time()
                self._road_det.set_lidar(anom, dist)
                anom_fr = ("Chaussée dégradée"
                           if anom == "chaussee_degradee"
                           else "Ralentisseur")
                d_str = f"  —  {dist} cm" if dist is not None else ""
                self._road_title.config(
                    text=f"⚠  {anom_fr}{d_str}",
                    fg=C.C_DANGER if anom == "chaussee_degradee"
                    else C.C_ACCENT2)

            else:
                self._lidar_anomaly = None
                self._road_det.set_lidar(None, None)
                self._road_title.config(
                    text="DÉTECTION INFRASTRUCTURE ROUTIÈRE",
                    fg=C.C_ACCENT)

        elif mt == "driver":
            val = msg.get("value", "")
            if val == "NO_TOUCH":
                self._hands_off = True
                self._hands_lbl.config(
                    text="⚠  MAINS HORS DU VOLANT",
                    bg="#200000", fg=C.C_DANGER)
                self._play_once("hands", 8.0)
                self._show_popup("NO_TOUCH")
            elif val == "HANDS_ON":
                self._hands_off = False
                self._hands_lbl.config(
                    text="✓  Mains sur le volant",
                    bg="#04120A", fg=C.C_SUCCESS)


        elif mt == "obd":
            self._obd_data.update(msg.get("data", {}))
            self._faults = self._fault_det.update(self._obd_data)
            current_keys = {f["key"] for f in self._faults}
            for fkey in current_keys - self._prev_faults:
                audio.play_fault(fkey)
            self._prev_faults = current_keys


    def _render_faults(self):
        C = _cfg
        c = self._fault_canvas
        c.delete("all")
        W = self._FP_W
        H = self._FP_H - self._HDR

        if not self._faults:
            cx, cy = W // 2, H // 2 - 10
            c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12,
                          fill=C.C_SUCCESS, outline="")
            c.create_text(cx, cy + 26,
                          text="Tous les systèmes fonctionnels",
                          font=("Courier", 10),
                          fill=C.C_SUCCESS, anchor="center")
            return

        n     = len(self._faults)
        row_h = min(38, (H - 4) // max(n, 1))
        for i, fault in enumerate(self._faults):
            y0  = 2 + i * (row_h + 2)
            y1  = y0 + row_h
            col = fault["color"]
            c.create_rectangle(2, y0, W - 2, y1,
                                fill="#0A1020", outline=col, width=1)
            c.create_oval(8, y0 + 5, 22, y1 - 5, fill=col, outline="")
            c.create_text(30, y0 + row_h // 2,
                          text=fault["severity"],
                          font=("Courier", 7, "bold"),
                          fill=col, anchor="w")
            c.create_text(105, y0 + row_h // 2,
                          text=fault["message"],
                          font=("Courier", 8),
                          fill=C.C_TEXT, anchor="w",
                          width=W - 110)


    def _show_popup(self, reason: str, speed=None, limit=None):
        """Popup d'alerte modale, auto-fermante après 5 s."""
        if self._popup_active:
            return
        self._popup_active = True
        C = _cfg

        sw = self._master.winfo_width()  or self._SW
        sh = self._master.winfo_height() or self._SH
        pw, ph = 520, 190
        px = (sw - pw) // 2
        py = (sh - ph) // 2

        popup = tk.Toplevel(self._master)
        popup.overrideredirect(True)
        popup.geometry(f"{pw}x{ph}+{px}+{py}")
        popup.configure(bg=C.C_BG)
        popup.attributes("-topmost", True)

        border = tk.Frame(popup, bg=C.C_DANGER, bd=2)
        border.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        inner = tk.Frame(border, bg=_POPUP_BG)
        inner.pack(fill=tk.BOTH, expand=True)

        MSGS = {
            "SOMNOLENCE": (
                "⚠  ALERTE SOMNOLENCE",
                "Somnolence détectée — Arrêtez-vous en sécurité"),
            "BÂILLEMENT": (
                "⚠  FATIGUE DÉTECTÉE",
                "Bâillement détecté — Prenez une pause dès que possible"),
            "NO_TOUCH": (
                "⚠  MAINS HORS DU VOLANT",
                "Gardez les deux mains sur le volant"),
            "VITESSE": (
                "⚠  VITESSE LIMITE DÉPASSÉE",
                (f"Vitesse {int(speed)} km/h — Limite {limit} km/h"
                 if speed is not None and limit is not None
                 else "Vous dépassez la vitesse autorisée")),
        }
        title, body = MSGS.get(reason, ("⚠  ALERTE", str(reason)))

        tk.Label(inner, text=title,
                 bg=_POPUP_BG, fg=C.C_DANGER,
                 font=("Courier", 16, "bold")).pack(pady=(16, 4))
        tk.Label(inner, text=body,
                 bg=_POPUP_BG, fg=C.C_TEXT,
                 font=("Courier", 11)).pack()
        tk.Button(inner, text="  FERMER  ",
                  bg=C.C_DANGER, fg="#000000",
                  font=("Courier", 10, "bold"),
                  relief=tk.FLAT, pady=4,
                  command=lambda: self._close_popup(popup)
                  ).pack(pady=(12, 0))

        self._popup_timer = self.after(5000,
                                       lambda: self._close_popup(popup))

    def _close_popup(self, popup):
        self._popup_active = False
        if self._popup_timer:
            self.after_cancel(self._popup_timer)
            self._popup_timer = None
        try:
            popup.destroy()
        except Exception:
            pass


    def _play_once(self, key: str, cooldown: float):
        """Joue le WAV 'key' uniquement si le cooldown est écoulé."""
        now = time.time()
        if now - self._audio_last.get(key, 0.0) > cooldown:
            self._audio_last[key] = now
            audio.play(key)

    @staticmethod
    def _show_frame(widget: tk.Label, bgr: np.ndarray, tw: int, th: int):
        """Affiche une frame BGR OpenCV dans un tk.Label."""
        if tw <= 0 or th <= 0:
            return
        frame = cv2.resize(bgr, (tw, th), interpolation=cv2.INTER_LINEAR)
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img   = ImageTk.PhotoImage(Image.fromarray(rgb))
        widget.imgtk = img
        widget.configure(image=img)

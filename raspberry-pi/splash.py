import tkinter as tk
import config as _cfg
from serial_manager import SerialManager


class SplashScreen(tk.Frame):

    def __init__(self, master, on_ready_callback):
        W = _cfg.SCREEN_W
        H = _cfg.SCREEN_H

        super().__init__(master, bg=_cfg.C_BG, width=W, height=H)
        self.pack_propagate(False)

        self._callback = on_ready_callback
        self._W = W
        self._H = H
        self._launched = False   

        self._serial = SerialManager()
        self._serial.start()

        self._states = {
            "lidar":     "attente",
            "bluetooth": "attente",
            "systeme":   "attente",
        }
        self._tick = 0
        self._build_ui()
        self._poll()

    def _build_ui(self):
        W, H = self._W, self._H
        C    = _cfg

        self._cv = tk.Canvas(self, width=W, height=H,
                             bg=C.C_BG, highlightthickness=0)
        self._cv.place(x=0, y=0)

        # Titre
        self._cv.create_text(
            W // 2, int(H * 0.158),
            text="SYSTÈME DE SÉCURITÉ ROUTIÈRE",
            font=("Courier", 28, "bold"),
            fill=C.C_ACCENT, anchor="center")
        self._cv.create_text(
            W // 2, int(H * 0.225),
            text="Plateforme embarquée — ESP32  +  Raspberry Pi 5",
            font=("Courier", 12),
            fill=C.C_TEXT_DIM, anchor="center")

        # Séparateur
        self._cv.create_line(
            int(W * 0.146), int(H * 0.270),
            W - int(W * 0.146), int(H * 0.270),
            fill=C.C_ACCENT, width=1, dash=(6, 4))

        self._cv.create_text(
            W // 2, int(H * 0.327),
            text="Envoi de la commande de démarrage à l'ESP32...",
            font=("Courier", 10),
            fill=C.C_WARNING, anchor="center")

        # Sous-systèmes
        sous_sys = [
            ("lidar",     "Calibration LiDAR  (TF-Luna)"),
            ("bluetooth", "Connexion ELM327 Bluetooth"),
            ("systeme",   "Système prêt"),
        ]
        self._dots   = {}
        self._labels = {}
        y0   = int(H * 0.413)
        step = int(H * 0.100)
        for i, (key, label) in enumerate(sous_sys):
            y   = y0 + i * step
            dot = self._cv.create_oval(
                int(W * 0.264), y - 14,
                int(W * 0.291), y + 14,
                fill="#141E30", outline=C.C_TEXT_DIM, width=1)
            self._cv.create_text(
                int(W * 0.311), y, text=label,
                font=("Courier", 14), fill=C.C_TEXT_DIM, anchor="w")
            st = self._cv.create_text(
                int(W * 0.674), y, text="En attente...",
                font=("Courier", 13, "bold"),
                fill=C.C_TEXT_DIM, anchor="w")
            self._dots[key]   = dot
            self._labels[key] = st

        cx = W // 2
        self._ring = self._cv.create_arc(
            cx - 30, H - 116, cx + 30, H - 56,
            start=0, extent=270,
            outline=C.C_ACCENT, width=3, style=tk.ARC)

        self._cv.create_text(
            W // 2, H - 38,
            text="Initialisation en cours — veuillez patienter",
            font=("Courier", 10), fill=C.C_TEXT_DIM, anchor="center")


        btn_w, btn_h = 260, 34
        btn_x = (W - btn_w) // 2
        btn_y = H - 28   # bas de l'écran

        self._force_btn = tk.Button(
            self,
            text="⚡  DÉMARRAGE FORCÉ",
            bg="#1A0A00", fg=C.C_ACCENT2,
            activebackground=C.C_ACCENT2, activeforeground="#000000",
            font=("Courier", 10, "bold"),
            relief=tk.FLAT,
            bd=1,
            highlightbackground=C.C_ACCENT2,
            highlightthickness=1,
            cursor="hand2",
            command=self._force_launch)
        self._force_btn.place(x=btn_x, y=btn_y - btn_h,
                              width=btn_w, height=btn_h)


    def _poll(self):
        # Vider la queue série
        try:
            while True:
                msg = self._serial.queue.get_nowait()
                self._handle_msg(msg)
        except Exception:
            pass

        self._tick = (self._tick + 9) % 360
        self._cv.itemconfig(self._ring, start=self._tick)

        if all(v == "ok" for v in self._states.values()):
            self.after(500, self._launch)
        else:
            self.after(50, self._poll)



    def _handle_msg(self, msg: dict):
        if msg.get("type") != "sys":
            return
        val = msg.get("value", "")
        if val == "LIDAR_CALIBRATED":
            self._set_state("lidar", "ok")
        elif val == "BT_CONNECTED":
            self._set_state("bluetooth", "ok")
        elif val == "READY":
            self._set_state("lidar",     "ok")
            self._set_state("bluetooth", "ok")
            self._set_state("systeme",   "ok")
        elif val == "LIDAR_ERROR":
            self._set_state("lidar", "erreur")
        elif val == "SERIAL_LOST":
            for k in self._states:
                self._set_state(k, "erreur")

    def _set_state(self, key: str, state: str):
        C = _cfg
        if state == "ok":
            col, text, fg = C.C_SUCCESS, "✓  OK",     C.C_SUCCESS
        elif state == "erreur":
            col, text, fg = C.C_DANGER,  "✗  ERREUR", C.C_DANGER
        else:
            col, text, fg = "#141E30", "En attente...", C.C_TEXT_DIM
        self._states[key] = state
        self._cv.itemconfig(self._dots[key],   fill=col, outline=col)
        self._cv.itemconfig(self._labels[key], text=text, fill=fg)


    def _force_launch(self):
        """Démarrage immédiat — ESP32 pas encore prêt, on force."""
        if self._launched:
            return
        print("[Splash] Démarrage forcé par l'utilisateur.")

        for k, v in self._states.items():
            if v != "ok":
                self._set_state(k, "erreur")
        self._launch()

    def _launch(self):
        if self._launched:
            return
        self._launched = True
        self.master._serial_manager = self._serial
        self._callback()

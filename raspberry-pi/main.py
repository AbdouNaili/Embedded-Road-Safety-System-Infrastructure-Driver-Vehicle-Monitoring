import multiprocessing as mp

if __name__ == "__main__":
    mp.set_start_method("fork", force=True)

import tkinter as tk
import config as _cfg
from splash        import SplashScreen
from app           import RoadSafetyApp
from audio_manager import audio


def main():
    root = tk.Tk()
    root.title("Système de Sécurité Routière")
    root.configure(bg=_cfg.C_BG)

    root.update_idletasks()
    real_w = root.winfo_screenwidth()
    real_h = root.winfo_screenheight()

    root.overrideredirect(True)
    root.geometry(f"{real_w}x{real_h}+0+0")
    root.update_idletasks()

    if real_w != _cfg.SCREEN_W or real_h != _cfg.SCREEN_H:
        print(f"[Main] Résolution : {real_w}×{real_h} "
              f"(config : {_cfg.SCREEN_W}×{_cfg.SCREEN_H}) → ajustement.")
        _cfg.SCREEN_W = real_w
        _cfg.SCREEN_H = real_h
        _cfg.TOP_H    = int(real_h * 0.67)
        _cfg.BOT_H    = real_h - _cfg.TOP_H
        _cfg.HALF_W_L = real_w // 2 - 6
        _cfg.HALF_W_R = real_w - _cfg.HALF_W_L

    root.attributes("-topmost", True)
    root.lift()
    root.focus_force()

    def _quit(event=None):
        audio.cleanup()
        root.destroy()

    root.bind("<Escape>", _quit)
    root.bind("<F4>",     _quit)

    def _launch_app():
        for w in root.winfo_children():
            w.destroy()
        app = RoadSafetyApp(root)
        app.place(x=0, y=0,
                  width=_cfg.SCREEN_W,
                  height=_cfg.SCREEN_H)

    splash = SplashScreen(root, on_ready_callback=_launch_app)
    splash.place(x=0, y=0,
                 width=_cfg.SCREEN_W,
                 height=_cfg.SCREEN_H)

    root.mainloop()


if __name__ == "__main__":
    main()

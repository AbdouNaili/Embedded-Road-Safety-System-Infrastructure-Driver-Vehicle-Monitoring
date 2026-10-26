import threading
import queue
import os
import time

_SOUNDS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "sounds")

WAV_FILES: dict = {
    # Anomalies chaussée
    "chaussee_degradee":     "chaussee_degradee.wav",
    "ralentisseur":          "ralentisseur.wav",
    # Panneaux vitesse
    "vitesse_30":            "vitesse_30.wav",
    "vitesse_40":            "vitesse_40.wav",
    "vitesse_60":            "vitesse_60.wav",
    "vitesse_80":            "vitesse_80.wav",
    "vitesse_100":           "vitesse_100.wav",
    "vitesse_depassee":      "vitesse_depassee.wav",
    # Feux et panneaux
    "feu_rouge":             "feu_rouge.wav",
    "feu_vert":              "feu_vert.wav",
    "stop":                  "stop.wav",
    "cedez_le_passage":      "cedez_le_passage.wav",
    "passage_pietons":       "passage_pietons.wav",
    "succession_de_virages": "succession_de_virages.wav",
    # Alertes conducteur
    "sleep":                 "somnolence.wav",
    "yawn":                  "baillement.wav",
    "hands":                 "mains_volant.wav",
    # Pannes OBD
    "fault_cooling":         "fault_cooling.wav",
    "fault_fuel":            "fault_fuel.wav",
    "fault_turbo":           "fault_turbo.wav",
    "fault_voltage":         "fault_voltage.wav",
    "fault_brake":           "fault_brake.wav",
    "fault_harsh_brake":     "fault_harsh_brake.wav",
    "fault_harsh_accel":     "fault_harsh_accel.wav",
    "fault_loss_control":    "fault_loss_control.wav",
    "fault_delayed_brake":   "fault_delayed_brake.wav",
}

# Alias clé courte OBD → clé complète
_FAULT_ALIAS: dict = {
    "cooling":       "fault_cooling",
    "fuel":          "fault_fuel",
    "turbo":         "fault_turbo",
    "voltage":       "fault_voltage",
    "brake":         "fault_brake",
    "harsh_brake":   "fault_harsh_brake",
    "harsh_accel":   "fault_harsh_accel",
    "loss_control":  "fault_loss_control",
    "delayed_brake": "fault_delayed_brake",
}


def _wav_path(key: str):
    fname = WAV_FILES.get(key)
    if not fname:
        return None
    p = os.path.join(_SOUNDS_DIR, fname)
    return p if os.path.isfile(p) else None


class AudioManager:

    def __init__(self):
        self._queue   = queue.Queue(maxsize=16)
        self._stop    = threading.Event()
        self._mixer_ok = False
        self._thread  = threading.Thread(
            target=self._worker, daemon=True, name="AudioMgr")
        self._thread.start()

    def _check_files(self):
        if not os.path.isdir(_SOUNDS_DIR):
            print(f"[Audio] ATTENTION : dossier sons introuvable → {_SOUNDS_DIR}")
            print("        Créez 'sounds/' et placez-y les fichiers WAV.")
            return
        missing = [k for k in WAV_FILES if _wav_path(k) is None]
        if missing:
            print(f"[Audio] Fichiers WAV manquants ({len(missing)}) : {missing}")
        else:
            print(f"[Audio] Tous les {len(WAV_FILES)} fichiers WAV trouvés.")


    def play(self, key: str):
        self._enqueue(key)

    def play_fault(self, fault_key: str):
        if fault_key.startswith("fault_"):
            self._enqueue(fault_key)
        else:
            self._enqueue(_FAULT_ALIAS.get(fault_key, f"fault_{fault_key}"))

    def play_speed_exceeded(self):
        self._enqueue("vitesse_depassee")

    def cleanup(self):
        self._stop.set()
        if self._mixer_ok:
            try:
                import pygame
                pygame.mixer.music.stop()
                pygame.mixer.quit()
            except Exception:
                pass


    def _worker(self):
        try:
            import pygame
            pygame.mixer.pre_init(frequency=44100, size=-16,
                                  channels=2, buffer=1024)
            pygame.mixer.init()
            self._mixer_ok = True
            print("[Audio] pygame.mixer initialisé dans le thread worker.")
        except Exception as e:
            print(f"[Audio] pygame.mixer non disponible : {e}")
            print("        Installez pygame : pip install pygame")

        self._check_files()

        while not self._stop.is_set():
            try:
                key = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue

            path = _wav_path(key)
            if path is None:
                print(f"[Audio] WAV introuvable pour la clé : '{key}'")
                continue

            self._play_wav(path)

    def _play_wav(self, path: str):
        if not self._mixer_ok or self._stop.is_set():
            return
        try:
            import pygame
            pygame.mixer.music.stop()
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self._stop.is_set():
                    pygame.mixer.music.stop()
                    return
                time.sleep(0.1)
        except Exception as e:
            print(f"[Audio] Erreur lecture '{os.path.basename(path)}' : {e}")

    def _enqueue(self, key: str):
        try:
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
            self._queue.put_nowait(key)
        except Exception:
            pass


audio = AudioManager()

import threading
import queue
import time
import serial
from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_START_CMD


class SerialManager:

    def __init__(self):
        self.queue      = queue.Queue(maxsize=400)
        self._ser       = None
        self._stop      = threading.Event()
        self._connected = False
        self._thread    = threading.Thread(
            target=self._run, daemon=True, name="SerialMgr")

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def send(self, text: str):
        """Envoie une chaîne à l'ESP32 (thread-safe)."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(text.encode("utf-8"))
            except Exception as e:
                print(f"[Serial] send() erreur : {e}")

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Boucle principale ─────────────────────────────────────

    def _run(self):
        while not self._stop.is_set():
            try:
                self._ser = serial.Serial(
                    SERIAL_PORT, SERIAL_BAUD, timeout=1)
                self._connected = True
                print(f"[Serial] Port {SERIAL_PORT} ouvert à {SERIAL_BAUD} baud.")

                time.sleep(0.4)
                self._ser.reset_input_buffer()
                self._ser.write(SERIAL_START_CMD.encode("utf-8"))
                print(f"[Serial] → '{SERIAL_START_CMD.strip()}' envoyé à l'ESP32.")

                self._read_loop()

            except serial.SerialException as e:
                self._connected = False
                print(f"[Serial] Erreur port : {e}")
                self._put({"type": "sys", "value": "SERIAL_LOST"})
                time.sleep(3)
            finally:
                # close the old port before trying to open it again
                if self._ser:
                    try:
                        self._ser.close()
                    except Exception:
                        pass

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self._parse(line)
            except serial.SerialException:
                self._connected = False
                self._put({"type": "sys", "value": "SERIAL_LOST"})
                break
            except Exception:
                pass

    def _parse(self, line: str):
        # Ignorer les lignes de debug ESP32 (commencent par '[')
        if line.startswith("["):
            return

        if line.startswith("sys:"):
            value = line[4:].strip()
            # The ESP32 resets when the port is opened, so the first CMD:START
            # can be lost while it boots. Send it again while it is waiting.
            if value == "WAITING_FOR_START":
                self.send(SERIAL_START_CMD)
            self._put({"type": "sys", "value": value})

        elif line.startswith("infrastructure:"):
            body    = line[15:].strip()
            anomaly = "NORMAL"
            dist    = None
            for tok in body.split():
                if tok.startswith("TYPE:"):
                    anomaly = tok[5:].strip()
                elif tok.startswith("DIST:"):
                    try:
                        dist = int(tok[5:].replace("cm", "").strip())
                    except ValueError:
                        pass
            self._put({"type": "infra", "anomaly": anomaly, "dist_cm": dist})

        elif line.startswith("driver:"):
            # IMPORTANT : priorité maximale — ne jamais ignorer ou filtrer
            value = line[7:].strip()
            self._put({"type": "driver", "value": value})

        elif line.startswith("car_OBD:"):
            self._put({"type": "obd",
                       "data": self._parse_obd(line[8:].strip())})

    @staticmethod
    def _parse_obd(body: str) -> dict:
        data: dict = {}
        for part in body.split(","):
            part = part.strip()
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k = k.strip()
            v = v.strip()
            if v in ("NA", "", "nan"):
                data[k] = None
            else:
                try:
                    data[k] = float(v)
                except ValueError:
                    data[k] = None
        return data

    def _put(self, msg: dict):

        try:
            self.queue.put_nowait(msg)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.queue.put_nowait(msg)
            except queue.Empty:
                pass

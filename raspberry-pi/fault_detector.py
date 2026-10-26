import time
from config import FAULT_THRESHOLDS

FAULT_LABELS: dict = {
    "cooling":       ("CRITIQUE",  "Surchauffe moteur — arrêt immédiat recommandé"),
    "fuel":          ("ALERTE",    "Défaut injection carburant"),
    "turbo":         ("ALERTE",    "Défaillance turbo / pression admission insuffisante"),
    "voltage":       ("ALERTE",    "Défaut électrique — tension ECU hors limites"),
    "brake":         ("CRITIQUE",  "Défaillance système de freinage"),
    "harsh_brake":   ("ATTENTION", "Freinage brusque détecté"),
    "harsh_accel":   ("ATTENTION", "Accélération agressive détectée"),
    "loss_control":  ("CRITIQUE",  "Risque de perte de contrôle"),
    "delayed_brake": ("ALERTE",    "Réaction de freinage tardive"),
}

SEVERITY_COLOR: dict = {
    "CRITIQUE":  "#FF1744",
    "ALERTE":    "#FFD600",
    "ATTENTION": "#FF9800",
}


class FaultDetector:

    def __init__(self):
        self._timers: dict = {k: None for k in FAULT_THRESHOLDS}
        self._prev_speed = None
        self._prev_time  = None

    def update(self, data: dict) -> list:
        now = time.time()

        rpm      = data.get("RPM")
        speed    = data.get("Speed")
        temp     = data.get("CoolantC")
        volt     = data.get("ECU_V")
        fuel_tgt = data.get("FuelTgt")
        fuel_act = data.get("FuelAct")
        boost    = data.get("Boost")
        brake    = data.get("Brake_bar")
        pedal    = data.get("Pedal%")

        # Longitudinal acceleration in m/s² (speed is in km/h).
        # One OBD cycle takes 1 to 3 s, so the real time between two readings is used.
        accel = None
        if self._prev_speed is not None and speed is not None:
            dt = now - self._prev_time
            if dt > 0.05:
                accel = (speed - self._prev_speed) / 3.6 / dt
        self._prev_speed = speed
        self._prev_time  = now

        hits: list = []
        th = FAULT_THRESHOLDS

        def check(name: str, cond: bool) -> bool:
            if cond:
                if self._timers[name] is None:
                    self._timers[name] = now
                elif now - self._timers[name] > th[name]["time"]:
                    return True
            else:
                self._timers[name] = None
            return False

        if check("cooling",
                 temp is not None and speed is not None
                 and temp  > th["cooling"]["temp_min"]
                 and speed > th["cooling"]["speed_min"]):
            hits.append("cooling")

        if fuel_tgt and fuel_act:
            if check("fuel",
                     abs(fuel_tgt - fuel_act) / max(abs(fuel_tgt), 1)
                     > th["fuel"]["diff_pct"]):
                hits.append("fuel")

        if check("turbo",
                 rpm is not None and boost is not None
                 and rpm   > th["turbo"]["rpm_min"]
                 and boost < th["turbo"]["boost_max"]):
            hits.append("turbo")

        if check("voltage",
                 volt is not None
                 and (volt < th["voltage"]["v_low"]
                      or volt > th["voltage"]["v_high"])):
            hits.append("voltage")

        if check("brake",
                 brake is not None and speed is not None
                 and speed > th["brake"]["speed_min"]
                 and brake < th["brake"]["press_min"]):
            hits.append("brake")

        if accel is not None and brake is not None:
            if check("harsh_brake",
                     accel < th["harsh_brake"]["accel_max"]
                     and brake > th["harsh_brake"]["press_min"]):
                hits.append("harsh_brake")

        if accel is not None and pedal is not None:
            if check("harsh_accel",
                     accel > th["harsh_accel"]["accel_min"]
                     and pedal > th["harsh_accel"]["pedal_min"]):
                hits.append("harsh_accel")

        if check("loss_control",
                 speed is not None and accel is not None
                 and speed > th["loss_control"]["speed_min"]
                 and abs(accel) > th["loss_control"]["accel_abs"]):
            hits.append("loss_control")

        if check("delayed_brake",
                 speed is not None and accel is not None
                 and speed > th["delayed_brake"]["speed_min"]
                 and accel < th["delayed_brake"]["accel_max"]
                 and (brake is None
                      or brake < th["delayed_brake"]["press_max"])):
            hits.append("delayed_brake")

        result = []
        for key in hits:
            sev, msg = FAULT_LABELS.get(key, ("ALERTE", key))
            result.append({
                "key":      key,
                "severity": sev,
                "message":  msg,
                "color":    SEVERITY_COLOR.get(sev, "#FF9800"),
            })
        return result

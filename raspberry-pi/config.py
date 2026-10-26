import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# ── Port série ESP32 ───────────────────────────────────────────
SERIAL_PORT      = "/dev/ttyUSB0"
SERIAL_BAUD      = 115200
SERIAL_START_CMD = "CMD:START\n"

# ── Index caméras ──────────────────────────────────────────────
ROAD_CAM_INDEX   = 2    
DRIVER_CAM_INDEX = 0    

# ── Modèle TFLite ──────────────────────────────────────────────
MODEL_PATH = os.path.join(_HERE, "models", "best_full_integer_quant.tflite")
MODEL_INPUT_SIZE   = 416
NMS_IOU_THRESHOLD  = 0.40
MIN_BOX_AREA       = 1200   

# ── 13 classes du modèle ──────────────────────────────────────
CLASS_NAMES = {
    0:  "chaussee_degradee",
    1:  "ralentisseur",
    2:  "cedez_le_passage",
    3:  "passage_pietons",
    4:  "stop",
    5:  "succession_de_virages",
    6:  "feu_rouge",
    7:  "feu_vert",
    8:  "vitesse_100",
    9:  "vitesse_30",
    10: "vitesse_40",
    11: "vitesse_60",
    12: "vitesse_80",
}

CLASS_LABELS_FR = {
    "chaussee_degradee":     "Chaussee degradee",
    "ralentisseur":          "Ralentisseur",
    "cedez_le_passage":      "Cedez le passage",
    "passage_pietons":       "Passage pietons",
    "stop":                  "Stop",
    "succession_de_virages": "Virages",
    "feu_rouge":             "Feu rouge",
    "feu_vert":              "Feu vert",
    "vitesse_100":           "100 km/h",
    "vitesse_30":            "30 km/h",
    "vitesse_40":            "40 km/h",
    "vitesse_60":            "60 km/h",
    "vitesse_80":            "80 km/h",
}

# ── Seuils de confiance ────────────────────────────────────────
CONF_THRESHOLD_NMS = 0.30
CONF_THRESHOLDS_PER_CLASS = {
    "chaussee_degradee":     0.60,
    "ralentisseur":          0.60,
    "cedez_le_passage":      0.40,
    "passage_pietons":       0.40,
    "stop":                  0.60,
    "succession_de_virages": 0.40,
    "feu_rouge":             0.60,
    "feu_vert":              0.60,
    "vitesse_100":           0.40,
    "vitesse_30":            0.40,
    "vitesse_40":            0.40,
    "vitesse_60":            0.40,
    "vitesse_80":            0.40,
}

# ── Panneaux de limitation de vitesse ─────────────────────────
SPEED_LIMIT_CLASSES = {
    "vitesse_30":  30,
    "vitesse_40":  40,
    "vitesse_60":  60,
    "vitesse_80":  80,
    "vitesse_100": 100,
}

# Tolérance (km/h) avant alerte de dépassement
SPEED_LIMIT_TOLERANCE = 3

# Classes audio-only (pas d'icône, juste WAV)
AUDIO_ONLY_CLASSES = {
    "cedez_le_passage",
    "passage_pietons",
    "stop",
    "succession_de_virages",
    "feu_rouge",
    "feu_vert",
}

# Cooldown audio par événement (secondes)
SIGN_SPEAK_COOLDOWN = 6.0

# ── Ressources ─────────────────────────────────────────────────
SIGN_ICONS_DIR = os.path.join(_HERE, "sign_icons")
SOUNDS_DIR     = os.path.join(_HERE, "sounds")

# ── Surveillance conducteur ────────────────────────────────────

CALIBRATION_TIME   = 4

# ── Paramètres EAR (yeux) ─────────────────────────────────────

EAR_DROP       = 0.08
EAR_THRESH_MIN = 0.15
EAR_THRESH_MAX = 0.26

SLEEP_TIME_THRESH = 1.3
EAR_SMOOTHING = 10

# ── Paramètres MAR (bouche / bâillement) ──────────────────────
MAR_THRESHOLD  = 0.5    
YAWN_TIME_THRESH = 0.25 
MAR_SMOOTHING  = 5      

LEFT_EYE  = [33,  160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]


# ── Seuils de détection de pannes OBD ─────────────────────────
FAULT_THRESHOLDS = {
    "cooling":       {"temp_min": 100, "speed_min": 60,   "time": 5},
    "fuel":          {"diff_pct": 0.15,                   "time": 2},
    "turbo":         {"rpm_min": 1800, "boost_max": 1200, "time": 2},
    "voltage":       {"v_low": 11.5,   "v_high": 14.8,   "time": 3},
    "brake":         {"speed_min": 30, "press_min": 1,    "time": 1},
    "harsh_brake":   {"accel_max": -4, "press_min": 5,    "time": 1},
    "harsh_accel":   {"accel_min": 3,  "pedal_min": 70,   "time": 1},
    "loss_control":  {"speed_min": 70, "accel_abs": 3,    "time": 1},
    "delayed_brake": {"speed_min": 80, "accel_max": -3,
                      "press_max": 2,                     "time": 1},
}

# ── Mise en page GUI (résolution cible 1024×600) ──────────────
SCREEN_W = 1024
SCREEN_H = 600
TOP_H    = 402
BOT_H    = 198
HALF_W_L = 506
HALF_W_R = 518

# ── Palette de couleurs ────────────────────────────────────────
C_BG        = "#080C18"
C_PANEL     = "#0C1120"
C_PANEL_ALT = "#101626"
C_BORDER    = "#1C2B44"
C_ACCENT    = "#00D4FF"
C_ACCENT2   = "#FF6B35"
C_SUCCESS   = "#00E676"
C_WARNING   = "#FFD600"
C_DANGER    = "#FF1744"
C_TEXT      = "#FFFFFF"
C_TEXT_LBL  = "#A8C4E0"
C_TEXT_DIM  = "#4A5A78"
C_VAL_OK    = "#00E5FF"

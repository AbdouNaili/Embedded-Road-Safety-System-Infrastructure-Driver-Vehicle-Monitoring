#include <Arduino.h>
#include <BluetoothSerial.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

static constexpr int RXD2       = 16;
static constexpr int TXD2       = 17;
static constexpr int TOUCH_PIN  = 27;
static constexpr int BUZZER_PIN = 25;

static constexpr uint32_t BAUD = 115200UL;

static const char* START_CMD        = "CMD:START";

static constexpr uint32_t WAIT_PING_MS = 1000UL;

static constexpr uint32_t CALIB_DURATION_MS = 4000UL;
static constexpr uint32_t CALIB_TIMEOUT_MS  = 8000UL;
static constexpr int      MEDIAN_WIN        = 5;
static constexpr uint32_t DECISION_MS       = 200UL;  


static constexpr float    DET_THRESHOLD_CM  = 20.0f;

static constexpr float    VOTE_MAJORITY     = 0.65f;

static constexpr uint16_t MIN_STRENGTH = 50;
static constexpr float    MIN_DIST_CM  = 10.0f;
static constexpr float    MAX_DIST_CM  = 600.0f;

// Hands-on-wheel sensor. Print touchRead(TOUCH_PIN) with your wiring to choose these:
// the value with the hand on the wheel and without it must be on opposite sides of the threshold.
static constexpr int      TOUCH_THRESHOLD  = 860;
static constexpr bool     TOUCH_IS_BELOW   = true;   // true: touched = value below the threshold
static constexpr uint32_t TOUCH_TIMEOUT_MS = 2000UL;

static constexpr uint32_t BUZZ_MS = 200UL;


BluetoothSerial SerialBT;
// Bluetooth MAC address of your ELM327 adapter (see README)
static uint8_t elm327MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

enum : uint8_t {
    EQ_RPM = 0,
    EQ_PEDAL,
    EQ_TEMP,
    EQ_VOLT,
    EQ_RAW16,
    EQ_DPF,
    EQ_BRAKE,
    EQ_SPEED,
    EQ_TORQUE,
};


struct PID {
    const char *label;
    const char *cmd;
    const char *header;
    const char *rspSid;
    const char *payload;
    uint8_t     dataBytes;
    uint8_t     eq;
    uint8_t     dec;
    uint16_t    timeoutMs;
};

static const PID PIDS[] = {
    { "RPM",       "222002", "7E0", "62", "2002", 2, EQ_RPM,    0, 900  },
    { "Pedal%",    "22202E", "7E0", "62", "202E", 2, EQ_PEDAL,  1, 900  },
    { "CoolantC",  "222001", "7E0", "62", "2001", 2, EQ_TEMP,   1, 900  },
    { "ECU_V",     "222005", "7E0", "62", "2005", 2, EQ_VOLT,   2, 900  },
    { "FuelTgt",   "222802", "7E0", "62", "2802", 2, EQ_RAW16,  0, 900  },
    { "FuelAct",   "222801", "7E0", "62", "2801", 2, EQ_RAW16,  0, 900  },
    { "Boost",     "222401", "7E0", "62", "2401", 2, EQ_RAW16,  0, 900  },
    { "DPF_mbar",  "222542", "7E0", "62", "2542", 2, EQ_DPF,    0, 900  },
    { "Speed",     "222003", "7E0", "62", "2003", 2, EQ_SPEED,  1, 900  },
    { "IntakeC",   "22240D", "7E0", "62", "240D", 2, EQ_TEMP,   1, 900  },
    { "Torque",    "222004", "7E0", "62", "2004", 2, EQ_TORQUE, 0, 900  },
    { "Brake_bar", "2107",   "740", "61", "07",   1, EQ_BRAKE,  1, 1200 },
};
static constexpr int N_PIDS = (int)(sizeof(PIDS) / sizeof(PIDS[0]));

enum PidIndex : uint8_t {
    I_RPM = 0, I_PEDAL, I_COOLANT, I_ECUV,
    I_FUELTGT, I_FUELACT, I_BOOST, I_DPF,
    I_SPEED, I_INTAKE, I_TORQUE, I_BRAKE,
    I_COUNT
};
static_assert(I_COUNT == N_PIDS, "PID index mismatch");

static SemaphoreHandle_t uartMtx      = NULL;
static volatile bool     lidarReady   = false;
static volatile bool     btReady      = false;
static volatile bool     sysReadySent = false;

static HardwareSerial LidarSerial(2);
static uint8_t        lidarRawBuf[7];

static float medBuf[MEDIAN_WIN];
static int   medFill = 0;
static int   medHead = 0;

static int   vchaussee_degradee = 0;
static int   vralentisseur      = 0;
static int   vNormal            = 0;
static float lastFiltered       = 0.0f;
static float calibBaseline      = 0.0f;
static const float detThreshold = DET_THRESHOLD_CM;


static char lastHeader[8] = {'\0'};

static void checkAndSendReady() {
    if (sysReadySent) return;
    if (!lidarReady || !btReady) return;
    sysReadySent = true;
    xSemaphoreTake(uartMtx, portMAX_DELAY);
    Serial.println("sys: READY");
    xSemaphoreGive(uartMtx);
}


static void waitForStartCommand() {
    Serial.println("sys: WAITING_FOR_START");
    Serial.println("[SYS] En attente de CMD:START depuis le Raspberry Pi...");

    uint32_t lastPing = millis();
    char     lineBuf[64];
    int      lineLen = 0;

    while (true) {
  
        if (millis() - lastPing >= WAIT_PING_MS) {
            lastPing = millis();
            Serial.println("sys: WAITING_FOR_START");
        }

        while (Serial.available()) {
            char c = (char)Serial.read();


            if (c == '\r') continue;

            if (c == '\n') {
                lineBuf[lineLen] = '\0';

                char *trimmed = lineBuf;
                while (*trimmed == ' ') trimmed++;
                int tlen = strlen(trimmed);
                while (tlen > 0 && trimmed[tlen - 1] == ' ') {
                    trimmed[--tlen] = '\0';
                }

                if (strcmp(trimmed, START_CMD) == 0) {
                    Serial.println("sys: START_RECEIVED");
                    Serial.println("[SYS] CMD:START reçu — démarrage du système.");
                    return;   
                }

                if (tlen > 0) {
                    Serial.print("[SYS] Message ignoré : ");
                    Serial.println(trimmed);
                }

                lineLen = 0;  
            } else {

                if (lineLen < (int)(sizeof(lineBuf) - 1)) {
                    lineBuf[lineLen++] = c;
                }
                // Si le buffer déborde sans '\n', on le flush silencieusement
                else {
                    lineLen = 0;
                }
            }
        }

        delay(1);   // Évite de saturer le CPU pendant l'attente
    }
}


static void sortF(float *a, int n) {
    for (int i = 0; i < n - 1; ++i) {
        int m = i;
        for (int j = i + 1; j < n; ++j)
            if (a[j] < a[m]) m = j;
        if (m != i) { float t = a[i]; a[i] = a[m]; a[m] = t; }
    }
}

static float medianOf(const float *src, int n) {
    float tmp[MEDIAN_WIN];
    int cnt = (n < MEDIAN_WIN) ? n : MEDIAN_WIN;
    for (int i = 0; i < cnt; ++i) tmp[i] = src[i];
    sortF(tmp, cnt);
    return (cnt & 1) ? tmp[cnt >> 1]
                     : 0.5f * (tmp[(cnt >> 1) - 1] + tmp[cnt >> 1]);
}

static float pushMedian(float s) {
    if (medFill < MEDIAN_WIN) {
        medBuf[medFill++] = s;
        return medianOf(medBuf, medFill);
    }
    medBuf[medHead] = s;
    medHead = (medHead + 1) % MEDIAN_WIN;
    return medianOf(medBuf, MEDIAN_WIN);
}


// ════════════════════════════════════════════════════════════
//  TF-Luna frame reader
// ════════════════════════════════════════════════════════════
static bool readLidarFrame(float &dist, uint16_t &strength) {
    if (LidarSerial.available() < 9) return false;
    if ((uint8_t)LidarSerial.read() != 0x59) return false;
    if ((uint8_t)LidarSerial.read() != 0x59) return false;
    if (LidarSerial.readBytes(lidarRawBuf, 7) != 7) return false;

    // byte 8 of the frame = low byte of the sum of bytes 0..7
    uint8_t sum = 0x59 + 0x59;
    for (int i = 0; i < 6; ++i) sum += lidarRawBuf[i];
    if (sum != lidarRawBuf[6]) return false;

    const uint16_t d = (uint16_t)lidarRawBuf[0] | ((uint16_t)lidarRawBuf[1] << 8);
    const uint16_t s = (uint16_t)lidarRawBuf[2] | ((uint16_t)lidarRawBuf[3] << 8);

    if (s < MIN_STRENGTH)           return false;
    if (d < (uint16_t)MIN_DIST_CM)  return false;
    if (d > (uint16_t)MAX_DIST_CM)  return false;

    dist     = (float)d;
    strength = s;
    return true;
}


// ════════════════════════════════════════════════════════════
//  LiDAR baseline calibration
// ════════════════════════════════════════════════════════════
static float calibrateBaseline() {
    Serial.println("[LIDAR] Calibration — gardez le véhicule immobile...");

    float lBuf[MEDIAN_WIN] = {};
    int   lFill = 0, lHead = 0;

    static float smp[1200];
    int cnt = 0;

    const uint32_t startMs  = millis();
    const uint32_t endMs    = startMs + CALIB_DURATION_MS;
    const uint32_t giveUpMs = startMs + CALIB_TIMEOUT_MS;

    while (millis() < endMs && millis() < giveUpMs && cnt < 1200) {
        float d; uint16_t s;
        if (readLidarFrame(d, s)) {
            if (lFill < MEDIAN_WIN) {
                lBuf[lFill++] = d;
            } else {
                lBuf[lHead] = d;
                lHead = (lHead + 1) % MEDIAN_WIN;
            }
            smp[cnt++] = medianOf(lBuf, lFill < MEDIAN_WIN ? lFill : MEDIAN_WIN);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (cnt == 0) {
        Serial.println("[LIDAR] ERREUR: aucun échantillon — vérifier le câblage du TF-Luna.");
        return -1.0f;
    }

    static float tmp[1200];
    for (int i = 0; i < cnt; ++i) tmp[i] = smp[i];
    sortF(tmp, cnt);
    float baseline = (cnt & 1) ? tmp[cnt >> 1]
                               : 0.5f * (tmp[(cnt >> 1) - 1] + tmp[cnt >> 1]);

    Serial.printf("[LIDAR] Calibration OK — %d échantillons, baseline = %.2f cm\n",
                  cnt, baseline);
    return baseline;
}


static float applyEq(uint8_t id, uint32_t A, uint32_t B) {
    const uint32_t raw = A * 256UL + B;
    switch (id) {
        case EQ_RPM:    return raw / 4.0f;
        case EQ_PEDAL:  return raw / 8.0f;
        case EQ_TEMP:   return ((float)raw - 2730.0f) / 10.0f;
        case EQ_VOLT:   return raw / 100.0f;
        case EQ_RAW16:  return (float)raw;
        case EQ_DPF:    return (float)raw - 32768.0f;
        case EQ_BRAKE:  return 0.5f * (float)A;
        case EQ_SPEED:  return raw / 100.0f;
        case EQ_TORQUE: return ((float)raw - 12800.0f) / 32.0f;
        default:        return NAN;
    }
}

static void btFlush() {
    while (SerialBT.available()) SerialBT.read();
}

static bool containsBadText(const char *s) {
    return strstr(s, "NODATA")  != nullptr ||
           strstr(s, "NO DATA") != nullptr ||
           strstr(s, "UNABLE")  != nullptr ||
           strstr(s, "ERROR")   != nullptr ||
           strstr(s, "STOPPED") != nullptr ||
           strstr(s, "BUSINIT") != nullptr ||
           strstr(s, "?")       != nullptr;
}

static void toUpperHexOnly(const char *in, char *out, int outLen) {
    int w = 0;
    for (int i = 0; in[i] != '\0' && w < outLen - 1; ++i) {
        const unsigned char c = (unsigned char)in[i];
        if (isxdigit(c)) out[w++] = (char)toupper(c);
    }
    out[w] = '\0';
}

static int btReadLine(char *buf, int bufLen, uint32_t timeoutMs) {
    int      n        = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        while (SerialBT.available()) {
            char c = (char)SerialBT.read();
            if (c == '>') { buf[n] = '\0'; return n; }
            if (n < bufLen - 1 && c != '\r' && c != '\n' && c >= 0x20)
                buf[n++] = c;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    buf[n] = '\0';
    return -1;
}

static int btSendCmd(const char *cmd, char *buf, int bufLen, uint32_t timeoutMs) {
    btFlush();
    SerialBT.print(cmd);
    SerialBT.print('\r');
    buf[0] = '\0';
    return btReadLine(buf, bufLen, timeoutMs);
}

static void initELM327() {
    char tmp[64];
    vTaskDelay(pdMS_TO_TICKS(500));
    btSendCmd("ATZ",    tmp, sizeof(tmp), 1500);
    btSendCmd("ATE0",   tmp, sizeof(tmp),  500);
    btSendCmd("ATL0",   tmp, sizeof(tmp),  500);
    btSendCmd("ATS0",   tmp, sizeof(tmp),  500);
    btSendCmd("ATH0",   tmp, sizeof(tmp),  500);
    btSendCmd("ATAT1",  tmp, sizeof(tmp),  500);
    btSendCmd("ATSTFF", tmp, sizeof(tmp),  500);
    btSendCmd("ATSP6",  tmp, sizeof(tmp),  500);
    memset(lastHeader, 0, sizeof(lastHeader));
}

static void setHeader(const char *hdr) {
    if (strcmp(lastHeader, hdr) == 0) return;
    char cmd[16], tmp[64];
    snprintf(cmd, sizeof(cmd), "ATSH%s", hdr);
    btSendCmd(cmd, tmp, sizeof(tmp), 300);
    strncpy(lastHeader, hdr, sizeof(lastHeader) - 1);
    lastHeader[sizeof(lastHeader) - 1] = '\0';
}

static bool requestPID(const PID &pid, uint32_t &A, uint32_t &B) {
    setHeader(pid.header);
    vTaskDelay(pdMS_TO_TICKS(10));
    char raw[128];

    for (int attempt = 0; attempt < 2; ++attempt) {
        int n = btSendCmd(pid.cmd, raw, sizeof(raw), pid.timeoutMs);
        if (n < 0 || n < 2)      { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if (containsBadText(raw)) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        char clean[128];
        toUpperHexOnly(raw, clean, sizeof(clean));

        const size_t sidLen     = 2;
        const size_t payloadLen = strlen(pid.payload);
        const char *best = nullptr, *p = clean;

        while ((p = strstr(p, pid.rspSid)) != nullptr) {
            const char *after = p + sidLen;
            if (strlen(after) >= payloadLen + (size_t)(pid.dataBytes * 2) &&
                strncmp(after, pid.payload, payloadLen) == 0) {
                best = after + payloadLen;
                break;
            }
            ++p;
        }
        if (!best) {
            p = clean;
            while ((p = strstr(p, pid.rspSid)) != nullptr) {
                const char *after = p + sidLen;
                if (strlen(after) >= payloadLen + (size_t)(pid.dataBytes * 2)) {
                    best = after + payloadLen;
                    break;
                }
                ++p;
            }
        }
        if (!best || strlen(best) < (size_t)(pid.dataBytes * 2)) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        char byteStr[3] = {best[0], best[1], '\0'};
        A = (uint32_t)strtoul(byteStr, nullptr, 16);
        B = 0;
        if (pid.dataBytes >= 2) {
            byteStr[0] = best[2]; byteStr[1] = best[3];
            B = (uint32_t)strtoul(byteStr, nullptr, 16);
        }
        return true;
    }
    return false;
}


static void buzzerTask(void *) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(BUZZ_MS));
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelete(nullptr);
}

static void triggerBuzzer() {
    xTaskCreatePinnedToCore(buzzerTask, "BUZ", 1024, nullptr, 1, nullptr, 1);
}


// ════════════════════════════════════════════════════════════
//  TOUCH TASK  (Core 0, highest priority)
// ════════════════════════════════════════════════════════════
void touchTask(void *) {
    bool     onWheel     = true;
    bool     alertActive = false;
    bool     timerOn     = false;
    uint32_t liftedAt    = 0;

    for (;;) {
        const int  v       = touchRead(TOUCH_PIN);
        const bool touched = TOUCH_IS_BELOW ? (v < TOUCH_THRESHOLD) : (v > TOUCH_THRESHOLD);

        if (touched) {
            if (!onWheel) {
                onWheel = true;
                timerOn = false;
                if (alertActive) {
                    alertActive = false;
                    xSemaphoreTake(uartMtx, portMAX_DELAY);
                    Serial.println("driver: HANDS_ON");
                    xSemaphoreGive(uartMtx);
                }
            }
        } else {
            if (onWheel) {
                onWheel  = false;
                timerOn  = true;
                liftedAt = millis();
            }
            if (timerOn && !alertActive &&
                (millis() - liftedAt >= TOUCH_TIMEOUT_MS)) {
                alertActive = true;
                timerOn     = false;
                xSemaphoreTake(uartMtx, portMAX_DELAY);
                Serial.println("driver: NO_TOUCH");
                xSemaphoreGive(uartMtx);
                triggerBuzzer();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lidarTask(void *) {
    lastFiltered = calibBaseline;
    uint32_t nextDecision = millis() + DECISION_MS;

    for (;;) {
        float d; uint16_t s;
        while (readLidarFrame(d, s)) {
            float f = pushMedian(d);
            lastFiltered = f;
            float delta = f - calibBaseline;
            if      (delta >  detThreshold) ++vchaussee_degradee;
            else if (delta < -detThreshold) ++vralentisseur;
            else                            ++vNormal;
        }

        uint32_t now = millis();
        if (now >= nextDecision) {
            nextDecision = now + DECISION_MS;
            int total = vchaussee_degradee + vralentisseur + vNormal;
            if (total > 0) {
                int cm = (int)roundf(lastFiltered);
                float majority = VOTE_MAJORITY * total;
                xSemaphoreTake(uartMtx, portMAX_DELAY);
                if (vchaussee_degradee >= majority) {
                    Serial.print("infrastructure: TYPE:chaussee_degradee DIST:");
                    Serial.print(cm);
                    Serial.println("cm");
                } else if (vralentisseur >= majority) {
                    Serial.print("infrastructure: TYPE:ralentisseur DIST:");
                    Serial.print(cm);
                    Serial.println("cm");
                }
                xSemaphoreGive(uartMtx);
            }
            vchaussee_degradee = vralentisseur = vNormal = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


// ════════════════════════════════════════════════════════════
//  OBD TASK  (Core 1, priority 2)
// ════════════════════════════════════════════════════════════
void obdTask(void *) {
    static char report[512];

    xSemaphoreTake(uartMtx, portMAX_DELAY);
    Serial.println("[OBD] Connexion à l'ELM327...");
    xSemaphoreGive(uartMtx);

    while (!SerialBT.connect(elm327MAC))
        vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreTake(uartMtx, portMAX_DELAY);
    Serial.println("sys: BT_CONNECTED");
    xSemaphoreGive(uartMtx);

    btReady = true;
    checkAndSendReady();

    initELM327();

    xSemaphoreTake(uartMtx, portMAX_DELAY);
    Serial.println("[OBD] Adaptateur initialisé — polling démarré.");
    xSemaphoreGive(uartMtx);

    float values[N_PIDS];
    bool  valid[N_PIDS];

    for (;;) {
        if (!SerialBT.connected()) {
            xSemaphoreTake(uartMtx, portMAX_DELAY);
            Serial.println("sys: BT_LOST");
            xSemaphoreGive(uartMtx);

            while (!SerialBT.connect(elm327MAC))
                vTaskDelay(pdMS_TO_TICKS(2000));

            xSemaphoreTake(uartMtx, portMAX_DELAY);
            Serial.println("sys: BT_RECONNECTED");
            xSemaphoreGive(uartMtx);

            initELM327();
        }

        for (int i = 0; i < N_PIDS; ++i) {
            values[i] = NAN;
            valid[i]  = false;
            uint32_t A = 0, B = 0;
            bool ok = requestPID(PIDS[i], A, B);
            valid[i] = ok;
            if (ok) values[i] = applyEq(PIDS[i].eq, A, B);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        int pos = snprintf(report, sizeof(report), "car_OBD: ");
        for (int i = 0; i < N_PIDS && pos < (int)sizeof(report) - 32; ++i) {
            pos += snprintf(report + pos, sizeof(report) - pos,
                            "%s=", PIDS[i].label);
            if (valid[i] && !isnan(values[i])) {
                char nb[24];
                dtostrf(values[i], 1, PIDS[i].dec, nb);
                pos += snprintf(report + pos, sizeof(report) - pos, "%s", nb);
            } else {
                pos += snprintf(report + pos, sizeof(report) - pos, "NA");
            }
            if (i < N_PIDS - 1)
                pos += snprintf(report + pos, sizeof(report) - pos, ", ");
        }

        xSemaphoreTake(uartMtx, portMAX_DELAY);
        Serial.println(report);
        xSemaphoreGive(uartMtx);
    }
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(BAUD);
    delay(200);   // Laisse le port USB se stabiliser

    Serial.println();
    Serial.println("==============================================");
    Serial.println("  ESP32 Road Security System  v8");
    Serial.println("==============================================");

    // ── Mutex ──────────────────────────────────────────────
    uartMtx = xSemaphoreCreateMutex();
    if (!uartMtx) {
        Serial.println("FATAL: mutex creation failed.");
        for (;;) delay(1000);
    }

    // ── Buzzer ─────────────────────────────────────────────
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[BUZZ] GPIO 25 prêt.");

    // ── Bluetooth (master mode) ────────────────────────────
    SerialBT.begin("ESP32_RoadSec", true);
    Serial.println("[BT]  Bluetooth démarré en mode maître.");

    // ── LiDAR UART ────────────────────────────────────────
    LidarSerial.begin(BAUD, SERIAL_8N1, RXD2, TXD2);
    delay(50);


    waitForStartCommand();

    // ── Calibration LiDAR (après réception de CMD:START) ──
    calibBaseline = calibrateBaseline();
    const bool lidarOK = calibBaseline > 0.0f;

    if (lidarOK) {
        for (int i = 0; i < MEDIAN_WIN; ++i) medBuf[i] = calibBaseline;
        medFill      = MEDIAN_WIN;
        medHead      = 0;
        lastFiltered = calibBaseline;

        Serial.printf("[LIDAR] Baseline = %.2f cm | Seuil = %.2f cm\n",
                      calibBaseline, detThreshold);

        // Signaler au Raspberry Pi que le LiDAR est calibré
        Serial.println("sys: LIDAR_CALIBRATED");
        lidarReady = true;
    } else {
        // Without a baseline every reading would look like a pothole: no LiDAR task.
        Serial.println("sys: LIDAR_ERROR");
    }

    // ── Touch sensor self-check ────────────────────────────
    bool touchOK = false;
    for (int i = 0; i < 10 && !touchOK; ++i) {
        int v = touchRead(TOUCH_PIN);
        if (v > 0 && v < 4096) touchOK = true;
        else delay(10);
    }
    Serial.println(touchOK ? "[TOUCH] Capteur OK."
                           : "[TOUCH] AVERTISSEMENT — vérifier le câblage.");

    // ── Démarrage des tâches FreeRTOS ─────────────────────
    Serial.println("[SYS] Démarrage des tâches FreeRTOS...");
    BaseType_t rc;

    rc = xTaskCreatePinnedToCore(obdTask,   "OBD",   8192, NULL, 2,
                                 NULL, 1);
    Serial.println(rc == pdPASS ? "[SYS] Tâche OBD OK (Core 1, priorité 2)."
                                : "FATAL: tâche OBD échouée.");

    rc = xTaskCreatePinnedToCore(touchTask, "TOUCH", 3072, NULL,
                                 configMAX_PRIORITIES - 1, NULL, 0);
    Serial.println(rc == pdPASS ? "[SYS] Tâche TOUCH OK (Core 0, max priorité)."
                                : "FATAL: tâche TOUCH échouée.");

    if (lidarOK) {
        rc = xTaskCreatePinnedToCore(lidarTask, "LIDAR", 6144, NULL, 3,
                                     NULL, 0);
        Serial.println(rc == pdPASS ? "[SYS] Tâche LIDAR OK (Core 0, priorité 3)."
                                    : "FATAL: tâche LIDAR échouée.");
    }

    Serial.println("[SYS] Toutes les tâches démarrées.");
    Serial.println("==============================================");
}


// ════════════════════════════════════════════════════════════
//  LOOP — inactif (tout le travail est dans les tâches)
// ════════════════════════════════════════════════════════════
void loop() {
    vTaskDelay(portMAX_DELAY);
}

#define BLYNK_TEMPLATE_ID   "TMPL3tHkiwbjU"
#define BLYNK_TEMPLATE_NAME "Robotic Arm"
#define BLYNK_AUTH_TOKEN    "54hl-vgooPEeF6NUHLhdeM9rUtQ5dWvK"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

char ssid[] = "Prodyumna";
char pass[] = "pro12345";

// ── Servo pin map ──────────────────────────────────────────────────────────────
#define PIN_V1 21   // Base
#define PIN_V2 22   // Shoulder
#define PIN_V3 23   // Elbow
#define PIN_V4 18   // Wrist
#define PIN_V5 19   // Claw

// ── Home / default positions ───────────────────────────────────────────────────
// Applied on power-up AND whenever Blynk disconnects / reconnects.
static constexpr float HOME[5] = { 0.0f, 177.0f, 180.0f, 0.0f, 100.0f };

// ── Tuning constants ───────────────────────────────────────────────────────────
// EMA alpha — higher = snappier tracking, lower = smoother.
// 0.20 at 50 Hz gives ~80 ms settling (good balance for a robotic arm).
static constexpr float EMA_ALPHA        = 0.20f;

// Max degrees the servo is allowed to travel per control loop tick.
// At 50 Hz, 5.0°/tick → max slew rate = 250°/s — fast enough for crisp motion
// without mechanical shock. Reduce if a servo sounds strained.
static constexpr float MAX_DEG_PER_TICK = 5.0f;

// Dead-band: ignore target updates smaller than this (kills WiFi-induced jitter).
// 0.5° is tight but effective; increase to 1.0° if jitter persists on a servo.
static constexpr float DEAD_BAND_DEG   = 0.5f;

// Control loop interval in ms (20 ms = 50 Hz)
static constexpr unsigned long LOOP_MS = 20;

// ── Per-servo state ────────────────────────────────────────────────────────────
struct ServoState {
    Servo   hw;
    float   target   = 90.0f;   // Latest angle commanded by Blynk
    float   smoothed = 90.0f;   // EMA-filtered setpoint
    float   current  = 90.0f;   // Velocity-limited position sent to servo
    int     pin      = 0;
    int     lastWritten = -1;   // Avoid redundant servo writes

    void attach(int p, float homeAngle) {
        pin      = p;
        target   = homeAngle;
        smoothed = homeAngle;
        current  = homeAngle;
        hw.attach(p);
        hw.write((int)homeAngle);
        lastWritten = (int)homeAngle;
    }

    // Snap all internal state to a new angle and write immediately.
    // Use for home / disconnect recovery — bypasses EMA and slew.
    void snapTo(float angle) {
        angle    = constrain(angle, 0.0f, 180.0f);
        target   = angle;
        smoothed = angle;
        current  = angle;
        int pos  = (int)roundf(angle);
        hw.write(pos);
        lastWritten = pos;
    }

    // Called by Blynk handlers — just store the target, never touch the servo here.
    void setTarget(int raw) {
        float angle = constrain((float)raw, 0.0f, 180.0f);
        if (fabsf(angle - target) >= DEAD_BAND_DEG) {
            target = angle;
        }
    }

    // Called every LOOP_MS ms by the control-loop timer.
    void step() {
        // 1. EMA: pull smoothed setpoint toward target
        smoothed += EMA_ALPHA * (target - smoothed);

        // 2. Velocity limiter: move current toward smoothed, capped per tick
        float delta = constrain(smoothed - current, -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);
        current    += delta;
        current     = constrain(current, 0.0f, 180.0f);

        // 3. Only write to servo if integer position changed (saves PWM churn)
        int pos = (int)roundf(current);
        if (pos != lastWritten) {
            hw.write(pos);
            lastWritten = pos;
        }
    }
};

// ── Servo array ────────────────────────────────────────────────────────────────
static ServoState servos[5];

// ── Home helper ───────────────────────────────────────────────────────────────
void goHome() {
    for (int i = 0; i < 5; i++) servos[i].snapTo(HOME[i]);
    Serial.println("Servos moved to home position");
}

// ── Blynk connection callbacks ────────────────────────────────────────────────
BLYNK_CONNECTED() {
    // Sync the app sliders so the UI reflects the current servo positions
    Blynk.syncAll();
    Serial.println("Blynk connected");
}

BLYNK_DISCONNECTED() {
    goHome();
    Serial.println("Blynk disconnected — homing servos");
}

// ── Blynk handlers — set target only ─────────────────────────────────────────
BLYNK_WRITE(V1) { servos[0].setTarget(param.asInt()); }
BLYNK_WRITE(V2) { servos[1].setTarget(param.asInt()); }
BLYNK_WRITE(V3) { servos[2].setTarget(param.asInt()); }
BLYNK_WRITE(V4) { servos[3].setTarget(180 - param.asInt()); }
BLYNK_WRITE(V5) { servos[4].setTarget(param.asInt()); }

// ── Control loop ──────────────────────────────────────────────────────────────
BlynkTimer timer;

void controlLoop() {
    for (auto& s : servos) s.step();

    // Debug serial output — comment out in production to save CPU
    Serial.print("<");
    for (int i = 0; i < 5; i++) {
        Serial.print((int)roundf(servos[i].current));
        if (i < 4) Serial.print(",");
    }
    Serial.println(">");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    const int pins[5] = { PIN_V1, PIN_V2, PIN_V3, PIN_V4, PIN_V5 };
    for (int i = 0; i < 5; i++) servos[i].attach(pins[i], HOME[i]);

    Serial.println("Servos attached at home position");

    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    timer.setInterval(LOOP_MS, controlLoop);
    Serial.println("Robotic arm ready — smoothing active");
}

void loop() {
    Blynk.run();
    timer.run();
}

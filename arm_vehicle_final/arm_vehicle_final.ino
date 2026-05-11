#include <ESP32Servo.h>
#include <Wire.h>

// ================================================================
//  ARM PINS
// ================================================================
#define PIN_J0   13
#define PIN_J1   32
#define PIN_J2   14
#define PIN_J3   27
#define PIN_J4   26
#define PIN_J5   25

// ================================================================
//  VEHICLE PINS
// ================================================================
#define IN1  18
#define IN2  19
#define ENA   4
#define IN3  23
#define IN4   5
#define ENB  17

#define PWM_FREQ  1000
#define PWM_BITS     8

// ================================================================
//  VEHICLE PID
// ================================================================
#define BASE_SPEED   110
#define RUN_TIME_MS 6000


#define DRIVE_CORR_MAX 40

float vKp = 3.0f, vKi = 0.05f, vKd = 1.2f;

// ================================================================
//  MPU6050
// ================================================================
#define MPU_ADDR      0x68
#define REG_PWR       0x6B
#define REG_GYRO_CFG  0x1B
#define REG_GYRO_Z    0x47
const float GYRO_SCALE = 131.0f;

// ================================================================
//  ARM JOINT LIMITS
// ================================================================
const float J_MIN[6] = {  0,  20,  20,   0,   0,   0 };
const float J_MAX[6] = {180, 160, 180, 180,  90,  90 };
const float J_HOM[6] = { 90,  90,  90,  90,  45,   0 };
#define TRAVEL_J1  60
#define TRAVEL_J2  160

// ================================================================
//  GRIPPER
// ================================================================
#define GRIP_L_OPEN    0.0f
#define GRIP_L_CLOSE  45.0f
#define GRIP_R_OPEN   45.0f
#define GRIP_R_CLOSE   0.0f

// ================================================================
//  ARM MOTION PARAMETERS
// ================================================================
float VEL_LIMIT[6] = { 12.0f, 12.0f, 15.0f, 12.0f, 35.0f, 35.0f };
float KP[6]   = { 0.8f, 0.9f, 0.9f, 0.8f, 2.0f,  2.0f  };
float KI_A[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.01f };
float KD_A[6] = { 0.5f, 0.6f, 0.6f, 0.5f, 0.6f,  0.6f  };

#define PID_HZ    50
#define PID_DT    (1.0f / PID_HZ)
#define DEAD_ZONE 1.5f
#define I_CLAMP   15.0f

// ================================================================
//  PICK & DROP POSITIONS
// ================================================================
#define PICK_YAW        90
#define PICK_HOVER_J1   35
#define PICK_HOVER_J2  140
#define PICK_GRASP_J1   20
#define PICK_GRASP_J2  160
#define PICK_J3          0

#define DROP_YAW         0
#define DROP_HOVER_J1   50
#define DROP_HOVER_J2  160
#define DROP_GRASP_J1   60
#define DROP_GRASP_J2  180
#define DROP_J3          0

#define LIFT_J1         90
#define LIFT_J2         90

// ================================================================
//  ARM STATE
// ================================================================
Servo srv[6];
float cur[6], tgt[6], wpt[6], itg_a[6], prv[6];

// ================================================================
//  POSE STRUCTURE
// ================================================================
struct Pose {
  float         j[6];
  unsigned long hold_ms;
  const char*   label;
};

// ================================================================
//  SEQUENCE A — PICK → DROP
// ================================================================
const Pose SEQ_A[] = {
  {{ 90, LIFT_J1, LIFT_J2, 90, GRIP_L_CLOSE, GRIP_R_CLOSE },
    1500, "A: Safety lift" },
  {{ PICK_YAW, LIFT_J1, LIFT_J2, 90, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2000, "A: Yaw to pick side" },
  {{ PICK_YAW, PICK_HOVER_J1, PICK_HOVER_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2500, "A: Hover above pick" },
  {{ PICK_YAW, PICK_HOVER_J1, PICK_HOVER_J2, PICK_J3, GRIP_L_OPEN, GRIP_R_OPEN },
    1000, "A: Open gripper" },
  {{ PICK_YAW, PICK_GRASP_J1, PICK_GRASP_J2, PICK_J3, GRIP_L_OPEN, GRIP_R_OPEN },
    2000, "A: Descend to pick" },
  {{ PICK_YAW, PICK_GRASP_J1, PICK_GRASP_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    1500, "A: Grasp object" },
  {{ PICK_YAW, TRAVEL_J1, TRAVEL_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2000, "A: Lift to vehicle height" },
  {{ DROP_YAW, TRAVEL_J1, TRAVEL_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    3000, "A: Yaw to drop side" },
  {{ DROP_YAW, DROP_HOVER_J1, DROP_HOVER_J2, DROP_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2000, "A: Hover above drop" },
  {{ DROP_YAW, DROP_HOVER_J1, DROP_HOVER_J2, DROP_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    1500, "A: Confirm settle at drop hover" },
};
const int SEQ_A_LEN = sizeof(SEQ_A) / sizeof(SEQ_A[0]);

// ================================================================
//  SEQUENCE B — DROP → PICK  (reverse trip after vehicle stops)
// ================================================================
const Pose SEQ_B[] = {
  //{{ DROP_YAW, TRAVEL_J1, TRAVEL_J2, DROP_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    //2000, "B: Lift to vehicle height at drop side" },
  {{ PICK_YAW, TRAVEL_J1, TRAVEL_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    3000, "B: Yaw to pick side at vehicle height" },
  {{ PICK_YAW, PICK_HOVER_J1, PICK_HOVER_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2000, "B: Hover above place position" },
  {{ PICK_YAW, PICK_GRASP_J1, PICK_GRASP_J2, PICK_J3, GRIP_L_CLOSE, GRIP_R_CLOSE },
    2000, "B: Descend to place position" },
  {{ PICK_YAW, PICK_GRASP_J1, PICK_GRASP_J2, PICK_J3, GRIP_L_OPEN, GRIP_R_OPEN },
    800,  "B: Release gently" },
  
  {{ PICK_YAW, PICK_HOVER_J1, PICK_HOVER_J2, 90, GRIP_L_OPEN, GRIP_R_OPEN },
    1500, "B: Lift away" },
  {{ PICK_YAW, PICK_HOVER_J1, PICK_HOVER_J2, 90, GRIP_L_OPEN, GRIP_R_OPEN },
    1000, "B: Wrist neutral" },
  {{ 90, 90, 90, 90, GRIP_L_CLOSE, GRIP_R_CLOSE },
    3000, "B: Return home" }
};
const int SEQ_B_LEN = sizeof(SEQ_B) / sizeof(SEQ_B[0]);

// ================================================================
//  SEQUENCER STATE
// ================================================================
const Pose*   active_seq     = nullptr;
int           active_seq_len = 0;
int           seq_step       = -1;
bool          seq_running    = false;
unsigned long seq_timer      = 0;
bool          seq_waiting    = false;

bool          seq_holding    = false;
bool          seq_done       = false;

// ================================================================
//  MISSION FSM
// ================================================================

enum MissionPhase {
  MP_ARM_A,
  MP_ARM_A_WAIT,
  MP_PAUSE_BEFORE_DRIVE,
  MP_DRIVE,
  MP_DRIVE_WAIT,
  MP_PAUSE_BEFORE_ARM_B,
  MP_ARM_B,
  MP_ARM_B_WAIT,
  MP_DONE
};
MissionPhase  mission            = MP_ARM_A;
unsigned long missionPauseUntil  = 0;  

// ================================================================
//  VEHICLE STATE
// ================================================================
float         gyroZoffset  = 0;
float         yaw          = 0;
float         v_integral   = 0;
float         v_prevError  = 0;
unsigned long driveStart   = 0;
unsigned long lastLoopTime = 0;
bool          driveRunning = false;
bool          driveDone    = false;

// ================================================================
//  HELPERS
// ================================================================
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

// ================================================================
//  ARM: SET TARGET
// ================================================================
void setTarget(int j, float deg) {
  if (j < 0 || j > 5) return;
  tgt[j]   = clampf(deg, J_MIN[j], J_MAX[j]);
  itg_a[j] = 0;
  prv[j]   = tgt[j] - cur[j];
}

void setPose(const Pose& p) {
  for (int j = 0; j < 6; j++) setTarget(j, p.j[j]);
}

void goHome() {
  for (int j = 0; j < 6; j++) setTarget(j, J_HOM[j]);
}

void openGripper() {
  setTarget(4, GRIP_L_OPEN);
  setTarget(5, GRIP_R_OPEN);
}

void closeGripper() {
  setTarget(4, GRIP_L_CLOSE);
  setTarget(5, GRIP_R_CLOSE);
}

// ================================================================
//  ARM: MOTION UPDATE — 50 Hz
// ================================================================
void motionUpdate() {
  for (int j = 0; j < 6; j++) {

    float to_tgt   = tgt[j] - wpt[j];
    float max_step = VEL_LIMIT[j] * PID_DT;

    if (fabsf(to_tgt) <= max_step)
      wpt[j] = tgt[j];
    else
      wpt[j] += (to_tgt > 0 ? 1.0f : -1.0f) * max_step;

    float error = wpt[j] - cur[j];

    if (fabsf(error) < DEAD_ZONE) {
      itg_a[j] *= 0.5f;
      prv[j]    = 0;
      srv[j].write((int)roundf(cur[j]));
      continue;
    }

    if (fabsf(error) < 5.0f) itg_a[j] = 0;

    itg_a[j] += error * PID_DT;
    itg_a[j]  = clampf(itg_a[j], -I_CLAMP, I_CLAMP);

    float deriv = clampf((error - prv[j]) / PID_DT, -30.0f, 30.0f);
    prv[j] = error;

    float scale  = clampf(fabsf(error) / 10.0f, 0.2f, 1.0f);
    float output = scale * (KP[j]*error + KI_A[j]*itg_a[j] + KD_A[j]*deriv);
    output = clampf(output, -max_step * 4.0f, max_step * 4.0f);

    cur[j] += output;
    cur[j]  = clampf(cur[j], J_MIN[j], J_MAX[j]);
    srv[j].write((int)roundf(cur[j]));
  }
}

// ================================================================
//  ARM: ALL AT TARGET
// ================================================================
bool allAtTarget() {
  for (int j = 0; j < 6; j++) {
    if (fabsf(wpt[j] - tgt[j]) > 0.5f)              return false; // profiler still moving
    if (fabsf(cur[j] - tgt[j]) > DEAD_ZONE * 3.0f)  return false; // servo not there yet
  }
  return true;
}

// ================================================================
//  ARM: START SEQUENCE
// ================================================================
void startSequence(const Pose* seq, int len) {
  active_seq     = seq;
  active_seq_len = len;
  seq_step       = 0;
  seq_running    = true;
  seq_waiting    = false;
  seq_holding    = false;   
  seq_timer      = 0;
  seq_done       = false;   
}

// ================================================================
//  ARM: SEQUENCER — non-blocking FSM
// ================================================================
void runSequencer() {
  if (!seq_running || active_seq == nullptr) return;

  if (seq_step >= active_seq_len) {
    Serial.println("[SEQ] Sequence COMPLETE.");
    seq_running = false;
    seq_step    = -1;
    seq_done    = true;   
    return;
  }

  if (!seq_waiting) {
    Serial.printf("[SEQ] Step %d/%d — %s\n",
                  seq_step + 1, active_seq_len, active_seq[seq_step].label);
    setPose(active_seq[seq_step]);
    seq_waiting = true;
    seq_holding = false;
    seq_timer   = 0;
  } else {
    if (!seq_holding) {
      // Wait until the arm has physically reached the target
      if (allAtTarget()) {
        seq_holding = true;
        seq_timer   = millis() + active_seq[seq_step].hold_ms;
        Serial.printf("[SEQ] Holding %lu ms...\n", active_seq[seq_step].hold_ms);
      }
    } else {
      // Holding — advance when timer expires
      if (millis() >= seq_timer) {
        seq_step++;
        seq_waiting = false;
        seq_holding = false;
        seq_timer   = 0;
      }
    }
  }
}

// ================================================================
//  MPU6050
// ================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

int16_t readRawGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_Z);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void calibrateGyro(int samples = 500) {
  Serial.print("[GYRO] Calibrating — keep still");
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readRawGyroZ();
    if (i % 100 == 0) Serial.print('.');
    delay(2);
  }
  gyroZoffset = (float)sum / samples;
  Serial.printf("\n[GYRO] Offset=%.2f\n", gyroZoffset);
}

// ================================================================
//  VEHICLE
// ================================================================
void motorsForward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}


void motorsBrake() {
  ledcWrite(ENA, 255); ledcWrite(ENB, 255);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  delay(80);   
  ledcWrite(ENA, 0);   ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setSpeeds(int L, int R) {
  ledcWrite(ENA, constrain(L, 0, 255));
  ledcWrite(ENB, constrain(R, 0, 255));
}

float vehiclePID(float error, float dt) {
  v_integral += error * dt;
  v_integral  = constrain(v_integral, -50.0f, 50.0f);
  float deriv = (dt > 0) ? (error - v_prevError) / dt : 0;
  v_prevError = error;
  return vKp * error + vKi * v_integral + vKd * deriv;
}

void startDrive() {
  Serial.println("[DRIVE] Starting vehicle...");
  yaw          = 0;
  v_integral   = 0;
  v_prevError  = 0;
  driveRunning = true;
  driveDone    = false;
  lastLoopTime = millis();
  driveStart   = millis();
  motorsForward();
  setSpeeds(BASE_SPEED, BASE_SPEED);
}


void updateDrive() {
  if (!driveRunning) return;
  unsigned long now = millis();

  if (now - driveStart >= (unsigned long)RUN_TIME_MS) {
    motorsBrake();
    driveRunning = false;
    driveDone    = true;
    Serial.printf("[DRIVE] Stopped. Yaw=%.2f° ran=%lu ms\n",
                  yaw, now - driveStart);
    return;
  }

  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;   
  if (dt <= 0.001f) return;

  float gz = (readRawGyroZ() - gyroZoffset) / GYRO_SCALE;
  yaw      += gz * dt;

  float corr = vehiclePID(yaw, dt);
  
  corr = clampf(corr, -DRIVE_CORR_MAX, DRIVE_CORR_MAX);
  setSpeeds(BASE_SPEED + (int)corr, BASE_SPEED - (int)corr);
}

// ================================================================
//  MISSION FSM
// ================================================================
void runMission() {
  switch (mission) {

    case MP_ARM_A:
      Serial.println("\n[MISSION] Phase 1 — Pick → Drop");
      startSequence(SEQ_A, SEQ_A_LEN);
      mission = MP_ARM_A_WAIT;
      break;

    case MP_ARM_A_WAIT:
      if (seq_done) {
        Serial.println("[MISSION] Arm done. Vehicle starts in 1s...");
        missionPauseUntil = millis() + 1000;
        mission = MP_PAUSE_BEFORE_DRIVE;
      }
      break;

    case MP_PAUSE_BEFORE_DRIVE:
      if (millis() >= missionPauseUntil)
        mission = MP_DRIVE;
      break;

    case MP_DRIVE:
      startDrive();
      mission = MP_DRIVE_WAIT;
      break;

    case MP_DRIVE_WAIT:
      updateDrive();
      if (driveDone) {
        Serial.println("[MISSION] Vehicle stopped. Arm returns in 1s...");
        missionPauseUntil = millis() + 1000;
        mission = MP_PAUSE_BEFORE_ARM_B;
      }
      break;

    case MP_PAUSE_BEFORE_ARM_B:
      if (millis() >= missionPauseUntil)
        mission = MP_ARM_B;
      break;

    case MP_ARM_B:
      Serial.println("\n[MISSION] Phase 3 — Drop → Pick (return)");
      startSequence(SEQ_B, SEQ_B_LEN);
      mission = MP_ARM_B_WAIT;
      break;

    case MP_ARM_B_WAIT:
      if (seq_done) mission = MP_DONE;
      break;

    case MP_DONE: {
      static bool printed = false;
      if (!printed) {
        Serial.println("\n================================================");
        Serial.println("  [MISSION] COMPLETE");
        Serial.println("  Object returned to original pick position.");
        Serial.println("  Vehicle travelled 3-4 m.");
        Serial.println("  Arm is HOME.");
        Serial.println("================================================\n");
        printed = true;
      }
      break;
    }
  }
}

// ================================================================
//  SERIAL COMMAND PARSER
// ================================================================
String cmd_buf = "";

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;   
    if (c == '\n') {
      cmd_buf.trim();
      cmd_buf.toLowerCase();
      if (cmd_buf.length() == 0) { cmd_buf = ""; return; }

      if (cmd_buf.length() >= 4
          && cmd_buf[0] == 'j'
          && cmd_buf[1] >= '0' && cmd_buf[1] <= '5'
          && cmd_buf[2] == ':') {
        int   ji  = cmd_buf[1] - '0';
        float ang = cmd_buf.substring(3).toFloat();
        setTarget(ji, ang);
        Serial.printf("[CMD] J%d → %.1f°\n", ji, tgt[ji]);
      }
      else if (cmd_buf == "home") {
        seq_running  = false;
        driveRunning = false;
        goHome();
        Serial.println("[CMD] HOME");
      }
      else if (cmd_buf == "open") {
        openGripper();
        Serial.println("[CMD] Gripper OPEN");
      }
      else if (cmd_buf == "close") {
        closeGripper();
        Serial.println("[CMD] Gripper CLOSE");
      }
      else if (cmd_buf == "stop") {
        seq_running  = false;
        driveRunning = false;
        motorsBrake();
        goHome();
        Serial.println("[CMD] EMERGENCY STOP → HOME");
      }
      else if (cmd_buf == "status") {
        const char* N[6] = {"J0 Yaw ","J1 Shldr","J2 Elbow","J3 Wrst","J4 GrpL ","J5 GrpR "};
        Serial.println("\n  Joint    Cur    Target  Waypoint");
        Serial.println("  ─────────────────────────────────");
        for (int j = 0; j < 6; j++)
          Serial.printf("  %s  %5.1f  %5.1f  %5.1f\n", N[j], cur[j], tgt[j], wpt[j]);
        Serial.printf("  Mission phase: %d\n\n", (int)mission);
      }
      else {
        Serial.println("[ERR] Commands: j0:90  home  open  close  stop  status");
      }
      cmd_buf = "";
    } else {
      cmd_buf += c;
    }
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  // ── STEP 1: Motor GPIO & LEDC FIRST (grabs timers 0 & 1) ──
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  // ── STEP 2: MPU6050 ──
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpuWrite(REG_PWR,      0x00); delay(100);
  mpuWrite(REG_GYRO_CFG, 0x00); delay(50);
  calibrateGyro(500);

  // ── STEP 3: Servo timers AFTER motor LEDC ──
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  const int   PINS[6]  = {PIN_J0,PIN_J1,PIN_J2,PIN_J3,PIN_J4,PIN_J5};
  const char* NAMES[6] = {
    "J0 Yaw      (GPIO13)", "J1 Shoulder (GPIO32)", "J2 Elbow    (GPIO14)",
    "J3 Wrist    (GPIO27)", "J4 Gripper-L(GPIO26)", "J5 Gripper-R(GPIO25)"
  };

  Serial.println("\n================================================");
  Serial.println("  COMBINED ARM + VEHICLE — Full Mission");
  Serial.println("  PICK: J0=90 J1=20 J2=160 J3=0");
  Serial.println("  DROP: J0=0  J1=60 J2=180 J3=0");
  Serial.println("  J1 moved to GPIO32 (was GPIO12)");
  Serial.println("================================================");

  for (int j = 0; j < 6; j++) {
    srv[j].attach(PINS[j], 500, 2500);
    cur[j] = wpt[j] = tgt[j] = J_HOM[j];
    itg_a[j] = prv[j] = 0;
    srv[j].write((int)J_HOM[j]);
    Serial.printf("  OK  %s  HOME=%.0f°\n", NAMES[j], J_HOM[j]);
  }

  Serial.println("================================================");
  Serial.println("  Mission starts in 2 seconds...");
  Serial.println("  Emergency: type 'stop' anytime");
  Serial.println("================================================\n");
  delay(2000);
}

// ================================================================
//  LOOP
// ================================================================
unsigned long last_pid = 0;

void loop() {
  unsigned long now = millis();

  // ARM PID at 50 Hz — always running
  if (now - last_pid >= (1000UL / PID_HZ)) {
    last_pid = now;
    motionUpdate();
  }

  runSequencer();
  runMission();
  parseSerial();
}

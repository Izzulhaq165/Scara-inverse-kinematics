#include <AccelStepper.h>
#include <math.h>

// ================== KONFIGURASI PIN ==================
// Z Axis (Lead Screw / slider vertical)
#define Z_STEP_PIN       4
#define Z_DIR_PIN        5
#define Z_LIMIT_SWITCH   12

// Link1 (rotasi lengan 1)
#define LINK1_STEP_PIN   2
#define LINK1_DIR_PIN    3
#define LINK1_LIMIT_SWITCH 11

// Link2 (rotasi lengan 2)
#define LINK2_STEP_PIN   6
#define LINK2_DIR_PIN    7
#define LINK2_LIMIT_SWITCH 13

// ================== KONSTANTA STEP/MOTOR ==================
const int STEPS_PER_REV = 200;     // langkah per putaran motor
const float DEG_PER_STEP = 1.8;    // derajat per step motor

// Rasio gear & lead screw
const float RATIO_LINK1 = 6.0;   // gear 1:5
const float RATIO_LINK2 = 3.0;   // gear 1:3
const float RATIO_Z     = 3.0;   // gear 1:3
const float LEAD_PITCH  = 8.0;   // mm per putaran screw

// ================== PARAMETER LENGAN SCARA ==================
#define L1 330.0f   // panjang link1 dalam mm
#define L2 250.0f   // panjang link2 dalam mm
const float L1_2 = L1 * L1;
const float L2_2 = L2 * L2;

// Axis index
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// Offset SCARA
const float SCARA_OFFSET_X = 0.0f;
const float SCARA_OFFSET_Y = 0.0f;

// Mode input (false = Cartesian -> IK, true = langsung derajat)
bool angle_mode = false;

// Pilihan elbow (true = elbow up, false = elbow down)
bool elbow_up = true;

// ================== OBJEK STEPPER ==================
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperLink1(AccelStepper::DRIVER, LINK1_STEP_PIN, LINK1_DIR_PIN);
AccelStepper stepperLink2(AccelStepper::DRIVER, LINK2_STEP_PIN, LINK2_DIR_PIN);

// ================== HOMING OFFSET ==================
const float Z_OFFSET_MM      = 0.0;   // posisi Z setelah homing
const float LINK1_OFFSET_DEG = 90.0;  // posisi Link1 setelah homing
const float LINK2_OFFSET_DEG = 0.0;   // posisi Link2 setelah homing

// ================== WAYPOINT CARTESIAN ==================
float cart_waypoints[][3] = {
  {0.0, 250.0,  0.0},  // WP1 (X, Y, Z)
  {0.0, 300.0,  0.0},  // WP2
  {0.0, 350.0,  0.0},  // WP3
  {0.0, 400.0,  0.0},  // WP1 (X, Y, Z)
  {0.0, 450.0,  0.0},  // WP2
  {0.0, 500.0,  0.0},  // WP3
};
const int numCartWaypoints = sizeof(cart_waypoints) / sizeof(cart_waypoints[0]);

// ================== PROTOTYPE ==================
void doHoming(AccelStepper &motor, int limitPin, int dir, long offsetSteps);
long degreeToStep(float degree, float ratio);
long mmToStep(float mm, float ratio);
void moveAllTo(float z_mm, float link1_deg, float link2_deg);
void inverse_kinematics(float const *cartesian, float *f_scara);

// ================== IMPLEMENTASI ==================

// Homing dengan offset
void doHoming(AccelStepper &motor, int limitPin, int dir, long offsetSteps) {
  motor.setMaxSpeed(800);
  motor.setAcceleration(600);
  motor.setSpeed(dir * 400);

  unsigned long start = millis();
  while (digitalRead(limitPin) == HIGH) {
    motor.runSpeed();
    if (millis() - start > 5000UL) break; // timeout
  }
  // set posisi sesuai offset
  motor.setCurrentPosition(offsetSteps);
}

// Konversi derajat -> step
long degreeToStep(float degree, float ratio) {
  float motorDegree = degree * ratio;
  float steps = motorDegree / DEG_PER_STEP;
  return lround(steps);
}

// Konversi mm -> step
long mmToStep(float mm, float ratio) {
  float rev = mm / LEAD_PITCH;
  float steps = rev * STEPS_PER_REV;
  return lround(steps * ratio);
}

// Gerak sinkron semua motor
void moveAllTo(float z_mm, float link1_deg, float link2_deg) {
  long targetZ     = mmToStep(z_mm, RATIO_Z);
  long targetLink1 = degreeToStep(link1_deg, RATIO_LINK1);
  long targetLink2 = degreeToStep(link2_deg, RATIO_LINK2);

  stepperZ.moveTo(targetZ);
  stepperLink1.moveTo(targetLink1);
  stepperLink2.moveTo(targetLink2);

  while (stepperZ.distanceToGo() != 0 ||
         stepperLink1.distanceToGo() != 0 ||
         stepperLink2.distanceToGo() != 0) {
    stepperZ.run();
    stepperLink1.run();
    stepperLink2.run();
  }
}

// Inverse kinematics SCARA -> derajat L1 & L2
void inverse_kinematics(float const *cartesian, float *f_scara) {
  float X = -cartesian[X_AXIS] - SCARA_OFFSET_X;
  float Y =  cartesian[Y_AXIS] + SCARA_OFFSET_Y;

  float r2 = X*X + Y*Y;
  float r  = sqrtf(r2);

  // Cek jangkauan
  if (r > (L1 + L2) + 0.001f || r < fabsf(L1 - L2) - 0.001f) {
    f_scara[X_AXIS] = NAN;
    f_scara[Y_AXIS] = NAN;
    f_scara[Z_AXIS] = cartesian[Z_AXIS];
    Serial.println("IK: Unreachable target!");
    return;
  }

  float cos2 = (r2 - L1_2 - L2_2) / (2.0f * L1 * L2);
  if (cos2 > 1.0f) cos2 = 1.0f;
  if (cos2 < -1.0f) cos2 = -1.0f;

  float sin2 = sqrtf(fmaxf(0.0f, 1.0f - cos2*cos2));
  if (!elbow_up) sin2 = -sin2;

  float theta2 = atan2f(sin2, cos2);

  float phi = atan2f(Y, X);
  float k   = atan2f(L2*sin2, L1 + L2*cos2);
  float theta1 = phi - k;

  float theta1_deg = theta1 * 180.0f / M_PI;
  float theta2_deg = theta2 * 180.0f / M_PI;

  if (!angle_mode) {
    f_scara[X_AXIS] = theta1_deg;
    f_scara[Y_AXIS] = theta2_deg;
    f_scara[Z_AXIS] = cartesian[Z_AXIS];
  } else {
    f_scara[X_AXIS] = cartesian[X_AXIS];
    f_scara[Y_AXIS] = cartesian[Y_AXIS];
    f_scara[Z_AXIS] = cartesian[Z_AXIS];
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(Z_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(LINK1_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(LINK2_LIMIT_SWITCH, INPUT_PULLUP);

  stepperZ.setMaxSpeed(1000);     stepperZ.setAcceleration(800);
  stepperLink1.setMaxSpeed(800); stepperLink1.setAcceleration(600);
  stepperLink2.setMaxSpeed(800); stepperLink2.setAcceleration(600);

  Serial.println("Homing Z...");
  doHoming(stepperZ, Z_LIMIT_SWITCH, +1, mmToStep(Z_OFFSET_MM, RATIO_Z));

  Serial.println("Homing Link1...");
  doHoming(stepperLink1, LINK1_LIMIT_SWITCH, +1, degreeToStep(LINK1_OFFSET_DEG, RATIO_LINK1));

  Serial.println("Homing Link2...");
  doHoming(stepperLink2, LINK2_LIMIT_SWITCH, +1, degreeToStep(LINK2_OFFSET_DEG, RATIO_LINK2));

  Serial.println("Homing selesai dengan offset.");
}

// ================== LOOP utama ==================
void loop() {
  for (int i = 0; i < numCartWaypoints; i++) {
    float cart[3], out_angles[3];
    cart[X_AXIS] = cart_waypoints[i][0];
    cart[Y_AXIS] = cart_waypoints[i][1];
    cart[Z_AXIS] = cart_waypoints[i][2];

    inverse_kinematics(cart, out_angles);

    float z_mm      = cart[Z_AXIS];
    float link1_deg = out_angles[X_AXIS];
    float link2_deg = out_angles[Y_AXIS];

    Serial.print("WP "); Serial.print(i);
    Serial.print(" X="); Serial.print(cart[X_AXIS]);
    Serial.print(" Y="); Serial.print(cart[Y_AXIS]);
    Serial.print(" Z="); Serial.print(cart[Z_AXIS]);
    Serial.print(" => L1="); Serial.print(link1_deg);
    Serial.print(" L2="); Serial.print(link2_deg);
    Serial.println();

    if (!isfinite(link1_deg) || !isfinite(link2_deg)) {
      Serial.println("Skip waypoint (IK error).");
      continue;
    }

    moveAllTo(z_mm, link1_deg, link2_deg);
    delay(500);
  }
  delay(1000);
}
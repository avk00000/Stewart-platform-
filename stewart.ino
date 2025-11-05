/*
  MPU-6050 Controlled 3-DOF Stewart Platform
  
  This code reads the Roll and Pitch from an MPU-6050 sensor
  and uses them to control the platform's tilt (Roll/Pitch)
  and height (Raise/Lower).

  CONTROL LOGIC:
  - If MPU is held flat (stationary): Platform stays flat at Z_FLAT.
  - If MPU is tilted left/right (Roll): Platform mimics the roll.
  - If MPU is tilted forward (Pitch > 3 deg): Platform pitches forward AND moves to Z_LOWERED.
  - If MPU is tilted backward (Pitch < -3 deg): Platform pitches backward AND moves to Z_RAISED.

  PRACTICAL WIRING:
  - Servos MUST be powered by an external 7.4V battery.
  - Arduino and Servos MUST share a common GND.
  - MPU can be powered from the Arduino 5V pin.
  
  PINOUT:
  - Servo 1: Pin 9
  - Servo 2: Pin 10
  - Servo 3: Pin 11
  - MPU SCL: Pin A5
  - MPU SDA: Pin A4
*/

// --- Libraries ---
#include <Servo.h>        // For Servos
#include <math.h>         // For IK math
#include <Wire.h>         // For I2C communication
#include <Adafruit_MPU6050.h> // For the sensor
#include <Adafruit_Sensor.h>  // Adafruit's sensor library

// --- Objects ---
Servo servos[3];
Adafruit_MPU6050 mpu;

// --- Pin configuration ----
const int SERVO_PIN[3] = {9, 10, 11};

// --- IK Geometry (CRITICAL: Match your real build!) ---
// All units are in millimeters (mm)
const float Rb = 80.0;   // base radius (distance from center to servo pivot)
const float Rp = 60.0;   // platform radius (distance from center to platform attachment)
const float la = 40.0;   // servo arm length (horn length)
const float lr = 100.0;  // connecting rod length
const float baseZ = 0.0; // z level of servo pivots (ground)

// Offsets and direction (for calibration)
const float servoOffset[3] = {0.0, 0.0, 0.0};
const int servoDir[3] = {1, 1, 1};

// Servo range limits (in degrees)
const float SERVO_MIN = 0.0;
const float SERVO_MAX = 180.0;

// 3D vector struct
struct Vec3 { float x, y, z; };
Vec3 baseAnchor[3];
Vec3 platformLocal[3];

// --- Control Logic Variables ---
static float targetZ = 80.0;      // Target height (tz)

// Define the heights for each mode
const float Z_FLAT = 80.0;
const float Z_RAISED = 95.0;
const float Z_LOWERED = 65.0;

// Define the max tilt of the platform (in degrees)
const float MAX_PLATFORM_TILT = 15.0; 
// Define the MPU tilt that maps to the max platform tilt
// A bigger number makes the controls *less sensitive*
const float MPU_MAP_RANGE = 30.0; 


// --- Utility (from previous code) ---
float toRad(float d){ return d * M_PI / 180.0; }
float toDeg(float r){ return r * 180.0 / M_PI; }


// ==========================================================
//                     SETUP FUNCTION
// ==========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Stewart Platform MPU Controller Initializing...");

  // 1. Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); } // Freeze if MPU not found
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Smooths out jitter

  // 2. Initialize Servos
  setupAnchors(); // Calculate IK anchor points
  for (int i=0;i<3;i++) {
    servos[i].attach(SERVO_PIN[i]);
  }

  Serial.println("System Ready. Moving to start position.");
  setPose(0, 0, Z_FLAT, 0, 0, 0); // Start at neutral
  delay(1000);
}

// ==========================================================
//                      MAIN LOOP
// ==========================================================
void loop() {
  
  // 1. Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 2. Calculate Roll and Pitch from Accelerometer data
  // This gives us the "static" tilt of the sensor
  float roll_rad = atan2(a.acceleration.y, a.acceleration.z);
  float pitch_rad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

  // Convert to degrees
  float mpu_roll = roll_rad * 180.0 / M_PI;
  float mpu_pitch = pitch_rad * 180.0 / M_PI;

  // 3. Define a "deadzone" for when the controller is stationary
  const float DEADZONE_DEGREES = 3.0;
  float targetRoll = 0;
  float targetPitch = 0;
  
  if (abs(mpu_roll) < DEADZONE_DEGREES && abs(mpu_pitch) < DEADZONE_DEGREES) {
    // ---- CONTROLLER IS STATIONARY ----
    // Go to the flat, neutral position
    targetRoll = 0;
    targetPitch = 0;
    targetZ = Z_FLAT;
    
  } else {
    // ---- CONTROLLER IS MOVING ----
    
    // 4. Map and Constrain the angles for the platform
    // Map the MPU's tilt range to the platform's desired tilt range
    targetRoll = map(mpu_roll, 
                           -MPU_MAP_RANGE, MPU_MAP_RANGE, 
                           -MAX_PLATFORM_TILT, MAX_PLATFORM_TILT);
                           
    targetPitch = map(mpu_pitch, 
                            -MPU_MAP_RANGE, MPU_MAP_RANGE, 
                            -MAX_PLATFORM_TILT, MAX_PLATFORM_TILT);

    // Constrain to prevent impossible angles
    targetRoll = constrain(targetRoll, -MAX_PLATFORM_TILT, MAX_PLATFORM_TILT);
    targetPitch = constrain(targetPitch, -MAX_PLATFORM_TILT, MAX_PLATFORM_TILT);

    // 5. Link Z-Height (Raise/Lower) to Pitch motion
    // This is now a smooth analog mapping, not a digital jump.
    // Tilting backward (-MPU_MAP_RANGE) maps to Z_RAISED.
    // Tilting forward (+MPU_MAP_RANGE) maps to Z_LOWERED.
    targetZ = map(mpu_pitch, 
                  -MPU_MAP_RANGE, MPU_MAP_RANGE, 
                  Z_RAISED, Z_LOWERED);
                      
    // Constrain the height to its max/min values
    targetZ = constrain(targetZ, Z_LOWERED, Z_RAISED);
  }

  // 6. Send the final pose to the IK solver
  // We use 0 for tx, ty, and yaw, as we only control z, roll, and pitch
  setPose(0, 0, targetZ, targetRoll, targetPitch, 0);

  // Optional: Print to Serial Monitor for debugging
  // Serial.print("Roll: "); Serial.print(targetRoll);
  // Serial.print("\t Pitch: "); Serial.print(targetPitch);
  // Serial.print("\t Height: "); Serial.println(targetZ);

  // Delay to keep a steady update rate
  delay(20); 
}


// ==========================================================
//            INVERSE KINEMATICS FUNCTIONS
// ==========================================================

// Pre-compute anchor coordinates
void setupAnchors() {
  for (int i = 0; i < 3; i++) {
    float ang = toRad(i * 120.0);      // base @ 0°,120°,240°
    baseAnchor[i] = { Rb*cos(ang), Rb*sin(ang), baseZ };
    platformLocal[i] = { Rp*cos(ang), Rp*sin(ang), 0.0 };
  }
}

// Apply roll-pitch-yaw rotation
void rotateRPY(const Vec3 &in, Vec3 &out, float roll, float pitch, float yaw) {
  float cr = cos(roll), sr = sin(roll);
  float cp = cos(pitch), sp = sin(pitch);
  float cy = cos(yaw), sy = sin(yaw);
  out.x = cy*cp*in.x + (cy*sp*sr - sy*cr)*in.y + (cy*sp*cr + sy*sr)*in.z;
  out.y = sy*cp*in.x + (sy*sp*sr + cy*cr)*in.y + (sy*sp*cr - cy*sr)*in.z;
  out.z = -sp*in.x   + cp*sr*in.y                 + cp*cr*in.z;
}

// Compute one servo angle
bool computeAngle(int i, float tx, float ty, float tz,
                  float roll, float pitch, float yaw, float &angleOut) {
  Vec3 pl = platformLocal[i], pr;
  rotateRPY(pl, pr, toRad(roll), toRad(pitch), toRad(yaw));
  Vec3 pw = { pr.x + tx, pr.y + ty, pr.z + tz };

  float dx = pw.x - baseAnchor[i].x;
  float dy = pw.y - baseAnchor[i].y;
  float dz = pw.z - baseAnchor[i].z;

  float h = sqrt(dx*dx + dy*dy);
  float d = sqrt(dx*dx + dy*dy + dz*dz);
  if (d < 1e-6) return false;

  float cosb = (la*la + d*d - lr*lr) / (2.0 * la * d);
  cosb = constrain(cosb, -1.0, 1.0);
  float beta = acos(cosb);
  float phi  = atan2(dz, h);
  float theta = phi + beta;
  float deg = toDeg(theta);
  deg = servoDir[i]*deg + servoOffset[i];
  deg = constrain(deg, SERVO_MIN, SERVO_MAX);
  angleOut = deg;
  return true;
}

// Command all three servos for a pose
bool setPose(float tx, float ty, float tz, float roll, float pitch, float yaw) {
  float a[3];
  for (int i=0;i<3;i++){
    if(!computeAngle(i,tx,ty,tz,roll,pitch,yaw,a[i])) return false;
    servos[i].write((int)round(a[i]));
  }

  // Print the final output to Serial Monitor for debugging
  Serial.print("Pose Z="); Serial.print(tz);
  Serial.print(" R="); Serial.print(roll);
  Serial.print(" P="); Serial.print(pitch);
  Serial.print(" -> Angles: ");
  for(int i=0;i<3;i++){ Serial.print((int)a[i]); if(i<2) Serial.print(","); }
  Serial.println();
  
  return true;
}




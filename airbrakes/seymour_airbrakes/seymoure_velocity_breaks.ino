#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
 
/* --- Hardware Objects --- */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo airbrakeServo;
 
/* --- Physical Constants --- */
const float rho_0  = 1.225;   // Sea level air density (kg/m^3)
const float mass   = 20.5251; // kg (no fuel); review before flight
const float area   = 0.01929; // m^2
const float Cd_off = 0.569;   // Drag coefficient
const float g      = 9.80665; // Gravity
 
/* --- Mission Configuration --- */
const float targetHeight  = 3048.0;
const float motor_burnout = 7.6;    
const float safety_stop   = 20.0;  
const float maxBuffer     = 152.4;  
const float v2Max         = 40000.0;
const float hysteresis    = 120.0;  
 
/* --- Servo Limits --- */
const int BRAKE_IN  = 90;
const int BRAKE_OUT = 137;
const int servoPin  = 4;
 
/* --- Global State --- */
bool LaunchDetected    = false;
unsigned long lastMicros = 0;
float timeSinceLaunch  = 0.0;
float velocity         = 0.0;
float prevVelocity     = 0.0; // Added for Trapezoidal integration
float height           = 0.0;
float accelFiltered    = 0.0;
 
// Prediction Simulation with Barometric Density Model
float predictFinalHeight(float v, float h) {
  float tempV = v;
  float tempH = h;
  float stepDT = 0.1;
 
  while (tempV > 0) {
    // Dynamic density calculation: rho = rho_0 * e^(-h / scale_height)
    // 8500m is the standard tropospheric scale height
    float currentRho = rho_0 * exp(-tempH / 8500.0);
   
    float drag = (0.5 * currentRho * tempV * tempV * Cd_off * area) / mass;
    float acceleration = -g - drag;
   
    tempH += (tempV * stepDT);
    tempV += (acceleration * stepDT);
   
    if (tempH > 4500) break;
  }
  return tempH;
}
 
void setup() {
  Serial.begin(115200);
  airbrakeServo.attach(servoPin);
  airbrakeServo.write(BRAKE_IN);
 
  if (!bno.begin()) {
    Serial.println("IMU_FAIL");
    while (1);
  }
  bno.setExtCrystalUse(true);
 
  delay(1000);
  lastMicros = micros();
  Serial.println("SYSTEM_READY");
}
 
void loop() {
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;
 
  if (dt <= 0 || dt > 0.1) return;
 
  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float acceleration = la.z(); // CHANGE to .x() if mounting is vertical
 
  if (!LaunchDetected) {
    // Keep everything zeroed until launch
    velocity = 0;
    height = 0;
    prevVelocity = 0;
    accelFiltered = 0;
 
    static int accelCount = 0;
    if (acceleration > 15.0) accelCount++;
    else accelCount = 0;
 
    if (accelCount > 5) {
      LaunchDetected = true;
      timeSinceLaunch = 0;
      Serial.println("LAUNCH_DETECTED");
    }
    return;
  }
 
  // 4. State Estimation
  timeSinceLaunch += dt;
 
  float alpha = 0.2;
  accelFiltered = (alpha * acceleration) + (1.0 - alpha) * accelFiltered;
 
  prevVelocity = velocity;
  velocity += accelFiltered * dt;
  // Trapezoidal Integration for better accuracy
  height += (prevVelocity + velocity) / 2.0 * dt;
 
  // 5. Deployment Logic
  if (timeSinceLaunch < motor_burnout) {
    airbrakeServo.write(BRAKE_IN);
    return;
  }
 
  static float lastPredTime = 0;
  static float predApogee = 0;
  if (timeSinceLaunch - lastPredTime > 0.05) {
    predApogee = predictFinalHeight(velocity, height);
    lastPredTime = timeSinceLaunch;
  }
 
  float predEnergy = g * predApogee;
  float buffer = maxBuffer * (velocity * velocity) / v2Max;
  if (buffer > maxBuffer) buffer = maxBuffer;
  float targetEnergy = g * (targetHeight + buffer);
 
  // 6. Final Actuation with Safety Shutdowns
  if (velocity < 5.0 || timeSinceLaunch > safety_stop) {
    airbrakeServo.write(BRAKE_IN);
  }
  else if (predEnergy > (targetEnergy + hysteresis)) {
    airbrakeServo.write(BRAKE_OUT);
  }
  else if (predEnergy < (targetEnergy - hysteresis)) {
    airbrakeServo.write(BRAKE_IN);
  }
}

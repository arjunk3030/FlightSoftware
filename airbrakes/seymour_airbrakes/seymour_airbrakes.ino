#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

/* --- Hardware Objects --- */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo airbrakeServo; 

const int servoPin = 4; 

const float g            = 9.80665;
const float targetHeight = 3048.0; 

const float maxBuffer       = 152.4;  // 500 ft
const float v2Max           = 40000.0; // (200 m/s)^2
const float hysteresis      = 120.0;   // ~40 ft energy equivalent
const float lagCompensation = 0.1; 

const int rotation  = 47;
const int BRAKE_IN  = 90;
const int BRAKE_OUT = 90 + rotation; 

bool LaunchDetected    = false;
unsigned long lastTime = 0;
float velocity         = 0.0;
float height           = 0.0;

void setup() {
  Serial.begin(115200);
  airbrakeServo.attach(servoPin);
  airbrakeServo.write(BRAKE_IN); 

  if (!bno.begin()) { while (1); }
  bno.setExtCrystalUse(true);
  delay(1000);
  lastTime = millis();

  Serial.println("SYSTEM_READY");
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;

  // Basic timing guard
  if (dt <= 0 || dt > 0.5) { 
    lastTime = currentTime; 
    return; 
  }
  lastTime = currentTime;

  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float accel = la.x(); 

  if (!LaunchDetected) {
    if (accel >= 1.5) {
      LaunchDetected = true;
      velocity = 0; 
      height = 0;
    }
    return;
  }

  // 3. Physics Integration
  float prevVelocity = velocity;
  velocity += accel * dt;
  height   += ((prevVelocity + velocity) / 2.0) * dt;

  // 4. Predictive Energy Calculation
  float predV = velocity + (accel * lagCompensation);
  float predH = height + (velocity * lagCompensation);
  float currentEnergy = (0.5 * predV * predV) + (g * predH);

  // 5. Dynamic Buffer Calculation
  float buffer = maxBuffer * (velocity * velocity) / v2Max;
  if (buffer > maxBuffer) buffer = maxBuffer;
  if (buffer < 0) buffer = 0;
  
  float targetEnergy = g * (targetHeight + buffer);

  // 6. Control Logic
  if (velocity < 5.0) {
    airbrakeServo.write(BRAKE_IN); 
  } 
  else if (currentEnergy > (targetEnergy + hysteresis)) {
    airbrakeServo.write(BRAKE_OUT); 
  } 
  else if (currentEnergy < (targetEnergy - hysteresis)) {
    airbrakeServo.write(BRAKE_IN); 
  }
}
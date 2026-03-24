#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo airbrake;

const int servoPin = 4;

const float DeployTime = 4.0;
const float OpenDuration = 5.0;   

const int rotation = 47

bool LaunchDetected = false;
unsigned long LaunchTime = 0;

bool AirbrakeOpened = false;
bool AirbrakeClosed = false;

struct State {
  float accel_x;
  float time;
};

State GlobalState;

void setup() {

  Serial.begin(115200);

  airbrake.attach(servoPin);
  airbrake.write(0);   // airbrakes start closed

  if (!bno.begin()) {
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(1000);
}

State GetState() {

  State s;

  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  s.accel_x = la.x();

  s.time = (millis() - LaunchTime) / 1000.0;

  return s;
}

void loop() {

  delay(10);

  if (!LaunchDetected) {

    GlobalState = GetState();

    if (GlobalState.accel_x >= 1.5) {
      LaunchDetected = true;
      LaunchTime = millis();
      Serial.println("Launch Detected");
    }

  } else {

    GlobalState = GetState();

    float t = GlobalState.time;

    if (t >= DeployTime && !AirbrakeOpened) {
      airbrake.write(90+rotation);    // deploy airbrakes
      AirbrakeOpened = true;
      Serial.println("Airbrakes OPEN");
    }

    if (t >= DeployTime + OpenDuration && AirbrakeOpened && !AirbrakeClosed) {
      airbrake.write(90);     // retract airbrakes
      AirbrakeClosed = true;
      Serial.println("Airbrakes CLOSED");
    }

  }
}
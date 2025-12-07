/********** Bismillah tinggal Latihan **********/
#include <Arduino.h>
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include "BluetoothSerial.h"

int LIFT_UP = 20;
int LIFT_DOWN = 75;
int GRIP = 80;
int UNGRIP = 100;

#define SERVO_MIN   500
#define SERVO_MAX   2500

/********** PID **********/
double kP = 32;
double kI = 200;
double kD = 0.4;

String serialInput = "";
double setpoint = 0;
double input = 0;
double output = 0;
double lastError = 0;
double integral = 0;

unsigned long lastTime = 0;
const int LOOP_TIME_MS = 5;

/********** Motor Driver L298N **********/
const int ENA = 4;
const int IN1 = 27;
const int IN2 = 26;
const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;

const int freq = 1000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int resolution = 8;

const int LED_BLUETOOTH = 15;
const int LED_FALL = 2;

const int SERVO_LIFT = 13;
const int SERVO_GRIP = 14;

bool dmpReady = false;

float pitch;
float gyroRate = 0.0;
float accAngle = 0.0;
float rawAngle = 0.0;

volatile bool IMUdataReady = false;

float correctionOffset = -10.8;
float throttleOffset = 0.0;

float angleV = 0;
float turnV = 0;
float deadband = 0;

float motor_min_speed = 60;
float throttle_fwd = 2.0;
float throttle_bwd = -2.0;

float easing = 1;
float steering_gain = 42;

float right_motor_gain = 1.0;
float left_motor_gain = 1.0;

float ax_offset = 0.0;
float velX = 0.0;
float distX = 0.0;
unsigned long lastTimelinear = 0;

bool btnForward = false;
bool btnBackward = false;
bool btnLeft = false;
bool btnRight = false;

bool activateGripCompe = false;

unsigned long GRIP_COMPENSATION_DELAY = 400;
unsigned long gripTimerStartTime = 0;
bool prevActivateGripCompe = false;
float gipCompensation = 10.0;
float gripCompApplied = 0.0;
float gripEasing = 0.01;
bool compeGrow = false;       

hw_timer_t* servoTimer = NULL;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t servoLiftPulse = 1500;  // μs
volatile uint16_t servoGripPulse = 1500;  // μs
volatile bool servoState = false;
volatile uint8_t servoIndex = 0;

Adafruit_MPU6050 mpu;
BluetoothSerial BT;

/********** RESET INPUTS **********/
void resetControllerStates() {
  btnForward = false;
  btnBackward = false;
  btnLeft = false;
  btnRight = false;
}

/********** SERIAL PID UPDATE **********/
void readPIDfromSerial() {
  if (Serial.available()) {
    serialInput = Serial.readStringUntil('\n');
    serialInput.trim();

    int spaceIndex = serialInput.indexOf(' ');
    String cmd = serialInput.substring(0, spaceIndex);
    String val = serialInput.substring(spaceIndex + 1);

    double num = val.toFloat();

    if (cmd.equalsIgnoreCase("KP"))       { kP = num; Serial.println("Updated KP = " + String(kP)); }
    else if (cmd.equalsIgnoreCase("KI")) { kI = num; Serial.println("Updated KI = " + String(kI)); }
    else if (cmd.equalsIgnoreCase("KD")) { kD = num; Serial.println("Updated KD = " + String(kD)); }
    else if (cmd.equalsIgnoreCase("SHOW")) {
      Serial.print("KP = "); Serial.println(kP);
      Serial.print("KI = "); Serial.println(kI);
      Serial.print("KD = "); Serial.println(kD);
    }
    else if (cmd.equalsIgnoreCase("OF"))  { correctionOffset = num; Serial.println("Updated OF = " + String(correctionOffset)); }
    else if (cmd.equalsIgnoreCase("DB"))  { deadband = num; Serial.println("Updated DB = " + String(deadband)); }
    else if (cmd.equalsIgnoreCase("MS"))  { motor_min_speed = num; Serial.println("Updated MS = " + String(motor_min_speed)); }
    else if (cmd.equalsIgnoreCase("TF"))  { throttle_fwd = num; Serial.println("Updated TF = " + String(throttle_fwd)); }
    else if (cmd.equalsIgnoreCase("TB"))  { throttle_bwd = num; Serial.println("Updated TB = " + String(throttle_bwd)); }
    else if (cmd.equalsIgnoreCase("SG"))  { steering_gain = num;  Serial.println("Updated SG = " + String(steering_gain)); }
    else if (cmd.equalsIgnoreCase("RMG")) { right_motor_gain = num; Serial.println("Updated RMG = " + String(right_motor_gain)); }
    else if (cmd.equalsIgnoreCase("LMG")) { left_motor_gain = num; Serial.println("Updated LMG = " + String(left_motor_gain)); }
    else if (cmd.equalsIgnoreCase("GRIP")) { GRIP = num; Serial.println("Set GRIP to " + String(GRIP)); }
    else if (cmd.equalsIgnoreCase("UNGRIP")) { UNGRIP = num; Serial.println("Set UNGRIP to " + String(UNGRIP));}
    else if (cmd.equalsIgnoreCase("LIFTUP")) { LIFT_UP = num; Serial.println("Set LIFT_UP to " + String(LIFT_UP)); }
    else if (cmd.equalsIgnoreCase("LIFTDOWN")) { LIFT_DOWN = num; Serial.println("Set LIFT_DOWN to " + String(LIFT_DOWN)); }
    else if (cmd.equalsIgnoreCase("GIPC")) { gipCompensation = num; Serial.println("Updated GIPC = " + String(gipCompensation)); }
    else if (cmd.equalsIgnoreCase("GRPCD")) { GRIP_COMPENSATION_DELAY = (unsigned long)num; Serial.println("Updated GRPCD = " + String(GRIP_COMPENSATION_DELAY)); }
    else if (cmd.equalsIgnoreCase("hold")) { 
      servoGripPulse = map(GRIP,   0, 180, SERVO_MIN, SERVO_MAX);
      activateGripCompe = true;
    }
    else if (cmd.equalsIgnoreCase("release")) { 
      servoGripPulse = map(UNGRIP, 0, 180, SERVO_MIN, SERVO_MAX); 
      activateGripCompe = false;
    }
    else { Serial.println("Unknown command!"); }
  }
}

/********** MPU6050 READING **********/
void readMPU6050() {
  unsigned long now = micros();
  float dt = (now - lastTimelinear) / 1e6;
  lastTimelinear = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float dt_constant = 0.005;

  gyroRate = g.gyro.y * 57.296;
  accAngle = -atan2(a.acceleration.x, a.acceleration.z) * 57.296;

  rawAngle = 0.98 * (rawAngle + gyroRate * dt_constant) + 0.02 * accAngle;
  pitch = rawAngle;

  float ax = a.acceleration.x - ax_offset;
  velX += ax * dt;
  distX += velX * dt;
  velX *= 0.98;
}

/********** MOTOR CONTROL **********/
void setMotorLeft(double speed, bool forward) {
  int _speed = constrain(abs(speed), 0, 255);

  digitalWrite(IN1, !forward);
  digitalWrite(IN2, forward);

  ledcWrite(pwmChannelLeft, _speed);
}

void setMotorRight(double speed, bool forward) {
  int _speed = constrain(abs(speed), 0, 255);

  digitalWrite(IN3, !forward);
  digitalWrite(IN4, forward);

  ledcWrite(pwmChannelRight, _speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(pwmChannelLeft, 0);
  ledcWrite(pwmChannelRight, 0);
}

/********** CONTROLLER **********/
void readController() {
  if (BT.available()) {
    char c = BT.read();

    switch (c) {
      case 'F': btnForward = true; break;
      case 'B': btnBackward = true; break;
      case 'L': btnLeft = true; break;
      case 'R': btnRight = true; break;
      case 's': resetControllerStates(); break;

      case 'P': servoLiftPulse = map(LIFT_UP,   0, 180, SERVO_MIN, SERVO_MAX); break;
      case 'T': servoLiftPulse = map(LIFT_DOWN, 0, 180, SERVO_MIN, SERVO_MAX); break;

      case 'G': 
        servoGripPulse = map(GRIP,   0, 180, SERVO_MIN, SERVO_MAX);
        activateGripCompe = true;
        break;
      case 'U': 
        servoGripPulse = map(UNGRIP, 0, 180, SERVO_MIN, SERVO_MAX); 
        activateGripCompe = false;
        break;

    }
    Serial.println(c);

    digitalWrite(LED_BLUETOOTH, HIGH);
  }
}

void updateTurn() {
  turnV = 0;
  if (btnLeft)  turnV = -steering_gain;
  if (btnRight) turnV =  steering_gain;
}

void updateTrim() {
  if (!btnForward && !btnBackward) {
    throttleOffset = 0.0;
    return;
  }

  float target = btnForward ? throttle_fwd : throttle_bwd;

  throttleOffset += (target - throttleOffset) * easing;

  throttleOffset = constrain(throttleOffset, throttle_bwd, throttle_fwd);
}

void IRAM_ATTR onServoTimer() {
    static uint8_t state = 0;

    portENTER_CRITICAL_ISR(&servoMux);

    if (state == 0) {
        digitalWrite(SERVO_LIFT, HIGH);
        timerAlarmWrite(servoTimer, servoLiftPulse, true);
        state = 1;
    }
    else if (state == 1) {
        digitalWrite(SERVO_LIFT, LOW);
        timerAlarmWrite(servoTimer, 500, true);
        state = 2;
    }
    else if (state == 2) {
        digitalWrite(SERVO_GRIP, HIGH);
        timerAlarmWrite(servoTimer, servoGripPulse, true);
        state = 3;
    }
    else {
        digitalWrite(SERVO_GRIP, LOW);
        timerAlarmWrite(servoTimer, 20000 - (servoLiftPulse + servoGripPulse + 500), true);
        state = 0;
    }

    portEXIT_CRITICAL_ISR(&servoMux);
}

/********** SETUP **********/
void setup() {
  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(pwmChannelLeft, freq, resolution);
  ledcSetup(pwmChannelRight, freq, resolution);

  ledcAttachPin(ENA, pwmChannelLeft);
  ledcAttachPin(ENB, pwmChannelRight);

  pinMode(LED_FALL, OUTPUT);
  pinMode(LED_BLUETOOTH, OUTPUT);

  BT.begin("hell nah");

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating MPU... Keep robot UPRIGHT");
  delay(3000);

  readMPU6050();

  pinMode(SERVO_LIFT, OUTPUT);
  pinMode(SERVO_GRIP, OUTPUT);

  servoTimer = timerBegin(1, 80, true);      // 80 prescaler → 1 tick = 1µs
  timerAttachInterrupt(servoTimer, &onServoTimer, true);
  timerAlarmWrite(servoTimer, 20000, false); // start at 20ms
  timerAlarmEnable(servoTimer);

  servoLiftPulse = map(LIFT_DOWN, 0, 180, SERVO_MIN, SERVO_MAX);
  servoGripPulse = map(UNGRIP,    0, 180, SERVO_MIN, SERVO_MAX);
}

/********** LOOP **********/
void loop() {
  readPIDfromSerial();
  readController();
  updateTrim();
  updateTurn();

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= LOOP_TIME_MS) {

    readMPU6050();
    lastTime = currentTime;

    if(activateGripCompe && !prevActivateGripCompe) {
        gripTimerStartTime = millis(); 
    }
    prevActivateGripCompe = activateGripCompe;

    input = pitch;
    setpoint = correctionOffset + throttleOffset;

    if (activateGripCompe) {
        if (millis() - gripTimerStartTime >= GRIP_COMPENSATION_DELAY) {
            input += gipCompensation;
        }
    }

    double error = setpoint - input;

    integral += error * (LOOP_TIME_MS / 1000.0);
    integral = constrain(integral, -100, 100);

    double derivative = (error - lastError) / (LOOP_TIME_MS / 1000.0);

    output = kP * error + kI * integral + kD * derivative;

    lastError = error;

    if (abs(output) < deadband) output = 0;

    double speedLeft  = (output * left_motor_gain)  + turnV;
    double speedRight = (output * right_motor_gain) - turnV;

    if (abs(speedRight) < motor_min_speed){
      int sign = (speedRight > 0) ? 1 : -1;
      speedLeft = speedRight * sign;
    }

    if (abs(speedLeft) < motor_min_speed){
      int sign = (speedLeft > 0) ? 1 : -1;
      speedLeft = speedLeft * sign;
    }

    if (input < 12 && input > -38) {

      setMotorLeft(speedLeft, speedLeft >= 0);
      setMotorRight(speedRight, speedRight >= 0);

      digitalWrite(LED_FALL, LOW);

    } else {
      stopMotors();
      integral = 0;
      digitalWrite(LED_FALL, HIGH);
    }
  }

  Serial.print(input);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(distX);
}

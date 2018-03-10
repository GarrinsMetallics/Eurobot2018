#include <avr/interrupt.h>
#include <TimerOne.h>
#include <PID_v1.h>

#define DEV_ID (202)
#define FW_VER (0001)

#define ENCODER_PIN_1 3
#define ENCODER_PIN_2 2
#define OUTPUT_PIN 5
#define DIRECTION_PIN 6

#define NUMPI 3.1415926
#define PPR 663 // total pulses per revolution
#define ANGLE 0.5429864 // degree per pulse
#define DELTA_TIME 30000 // 0.03 s
#define RADIUS 30 // mm

enum MSG_CMD {
  DRIVER_ID = 1,
  FW_VERSION,
  RUNNING_TIME,
  GET_SPEED,
  SET_SPEED,
  GET_STEPS,
  SET_STEPS
};

enum MSG_TYPE {
  REQUEST = 0xA,
  RESPONSE
};

typedef struct {
  uint8_t command;
  uint8_t type;
  int32_t value;
} GarrinsMsg;

static GarrinsMsg req_msg;

volatile double pulses = 0;
volatile double last_pulses = 0;
volatile double delta_distance = 0;
volatile double motor_speed = 0;
volatile double delta_pulses = 0;

double Setpoint;
double Input;
double Output;
double Kp = 0.4;
double Ki = 1.1;
double Kd = 0.002;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  digitalWrite(ENCODER_PIN_1, HIGH);
  attachInterrupt(0, updatePulsesEncoder1, RISING);
  digitalWrite(ENCODER_PIN_2, HIGH);
  attachInterrupt(0, updatePulsesEncoder2, RISING);

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  Timer1.initialize(DELTA_TIME);         // initialize timer1, and set a 0.03 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  Input = motor_speed;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);

  sei();

}

void loop() {
  if (receiveGarrinsMsg(&req_msg)) {
    replyGarrinsMsg(&req_msg);
  }

  digitalWrite(DIRECTION_PIN, HIGH);
  Input = motor_speed;
  myPID.Compute();
  analogWrite(OUTPUT_PIN, Output);

  delay(50);
}

void callback() {
  delta_pulses = pulses - last_pulses;
  delta_distance = (delta_pulses * ANGLE * 2 * NUMPI * RADIUS) / 360;
  motor_speed =  (delta_distance * 1000000)/ DELTA_TIME; //mm/s
  last_pulses = pulses;
}

void updatePulsesEncoder1() {
  pulses++;
}

void updatePulsesEncoder2() {
  pulses++;
}

static bool receiveGarrinsMsg(GarrinsMsg* msg) {
  bool ret_val = false;
  uint8_t *ptr = (uint8_t *)msg;

  while (Serial.available() > 0) {
    *ptr = Serial.read();
    ptr++;
    ret_val = true;
  }

  return ret_val;
}

static void replyGarrinsMsg(GarrinsMsg* msg) {
  GarrinsMsg resp_msg;
  resp_msg.command = msg->command;
  resp_msg.type = RESPONSE;

  switch (msg->command) {
    case DRIVER_ID:
      resp_msg.value = DEV_ID;
      break;
    case FW_VERSION:
      resp_msg.value = FW_VER;
      break;
    case RUNNING_TIME:
      resp_msg.value = millis();
      break;
    case GET_SPEED:
      resp_msg.value = (uint32_t)motor_speed;
      break;
    case SET_SPEED:
      Setpoint = (double)msg->value;
      resp_msg.value = (uint32_t)Setpoint;
      break;
    case GET_STEPS:
      resp_msg.value = pulses;
      break;
    case SET_STEPS:
      pulses = msg->value;
      resp_msg.value = pulses;
      break;
  }

  Serial.write((uint8_t*)&resp_msg, sizeof(GarrinsMsg));
}


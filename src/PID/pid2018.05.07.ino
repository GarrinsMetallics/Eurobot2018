#include <avr/interrupt.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <avr/wdt.h>

#define DEV_ID (303) //A->101  B->202  C->303
#define FW_VER (0001)

#define ENCODER_PIN_1 3
#define ENCODER_PIN_2 2
#define OUTPUT_PIN 5
#define DIRECTION_PIN 6
#define TRIGGER_PIN 8

#define NUMPI 3.1415926
#define PPR 663 // total pulses per revolution
#define ANGLE 0.5429864 // degree per pulse
#define DELTA_TIME 10000 // 0.03 s  -> 30000us
#define RADIUS 30 // mm

enum STATES {
  ROBOT_IDLE = 10,
  ROBOT_MOVING,
  ROBOT_BLOCKED
};

enum MSG_CMD {
  DRIVER_ID = 1,
  FW_VERSION,
  RUNNING_TIME,
  GET_SPEED,
  SET_SPEED,
  GET_STEPS,
  SET_STEPS,
  GET_POSITION,
  SET_POSITION,
  GET_STATE
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
static int robot_state = ROBOT_IDLE;

volatile double pulses = 0;
volatile double last_pulses = 0;
volatile double delta_distance = 0;
volatile double motor_speed = 0;
volatile double delta_pulses = 0;
volatile double current_distance = 0; //mm
volatile double setpoint_distance = 0;  //mm

double Setpoint_speed;
double Input;
double Output;
double Kp = 0.17;//0.2;
double Ki = 1.3;//1;
double Kd = 0.0008;

double Setpoint_speed_orig;

PID myPID(&Input, &Output, &Setpoint_speed, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  digitalWrite(ENCODER_PIN_1, HIGH);
  attachInterrupt(0, updatePulsesEncoder1, RISING);
  digitalWrite(ENCODER_PIN_2, HIGH);
  attachInterrupt(0, updatePulsesEncoder2, RISING);

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  pinMode(TRIGGER_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  Timer1.initialize(DELTA_TIME);         // initialize timer1, and set a 0.03 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  Input = motor_speed;
  Setpoint_speed = 0;
  myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(30);

  sei();
  wdt_enable(WDTO_500MS);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (digitalRead(TRIGGER_PIN) == HIGH) {
    myPID.SetMode(AUTOMATIC);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (receiveGarrinsMsg(&req_msg)) {
    replyGarrinsMsg(&req_msg);
  }

  Setpoint_speed = map(current_distance, 0, setpoint_distance, Setpoint_speed_orig, Setpoint_speed_orig / 4);

  if (current_distance >= setpoint_distance) {
    Setpoint_speed = 0;
    pulses = 0;
    last_pulses = 0;
    delta_distance = 0;
    motor_speed = 0;
    setpoint_distance = 0;
    current_distance = 0;
    robot_state = ROBOT_IDLE;
    myPID.SetMode(MANUAL);
    digitalWrite(LED_BUILTIN, LOW);
  }

  Input = motor_speed;
  myPID.Compute();
  if (Setpoint_speed == 0 || setpoint_distance == 0 || digitalRead(TRIGGER_PIN) == LOW) {
    Output = 0;
  }

  analogWrite(OUTPUT_PIN, Output);
  
  wdt_reset();

  delay(50);
}

void callback() {
  delta_pulses = pulses - last_pulses;
  delta_distance = (delta_pulses * ANGLE * 2 * NUMPI * RADIUS) / 360;
  motor_speed =  (delta_distance * 1000000) / DELTA_TIME; //mm/s
  last_pulses = pulses;
  current_distance = (2 * NUMPI * RADIUS * pulses) / PPR;
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
      resp_msg.value = (int32_t)motor_speed;
      break;
    case SET_SPEED:
      robot_state = ROBOT_MOVING;
      Setpoint_speed_orig = (double)msg->value;
      Setpoint_speed_orig = abs(Setpoint_speed_orig);
      resp_msg.value = (int32_t)Setpoint_speed_orig;
      break;
    case GET_STEPS:
      resp_msg.value = pulses;
      break;
    case SET_STEPS:
      robot_state = ROBOT_MOVING;
      pulses = msg->value;
      resp_msg.value = pulses;
      break;
    case GET_POSITION:
      resp_msg.value = current_distance;
      break;
    case SET_POSITION:
      robot_state = ROBOT_MOVING;
      setpoint_distance = (double)msg->value;
      resp_msg.value = (int32_t)setpoint_distance;
      if (setpoint_distance >= 0) {
        digitalWrite(DIRECTION_PIN, HIGH);
      }
      else {
        digitalWrite(DIRECTION_PIN, LOW);
        setpoint_distance = abs(setpoint_distance);
      }
      break;
    case GET_STATE:
      resp_msg.value = robot_state;
      break;
  }

  Serial.write((uint8_t*)&resp_msg, sizeof(GarrinsMsg));
}

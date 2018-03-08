#define DEV_ID (101)
#define FW_VER (0001)

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

static long int motor_speed;
static long int motor_steps;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  motor_speed = 0;
  motor_steps = 0;

}

void loop() {
 if (receiveGarrinsMsg(&req_msg)) {
  replyGarrinsMsg(&req_msg);
 }

  delay(50);
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
    resp_msg.value = motor_speed;
    break;
    case SET_SPEED:
    motor_speed = msg->value;
    resp_msg.value = motor_speed;
    break;
    case GET_STEPS:
    resp_msg.value = motor_steps;
    break;
    case SET_STEPS:
    motor_steps = msg->value;
    resp_msg.value = motor_steps;
    break;
  }
  
  Serial.write((uint8_t*)&resp_msg, sizeof(GarrinsMsg));
}


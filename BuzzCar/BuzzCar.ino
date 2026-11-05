#include <Wire.h>

const int LEFT_SENSOR_PIN   = 1;
const int MIDDLE_SENSOR_PIN = 0;
const int RIGHT_SENSOR_PIN  = 2;

const int LEFT_MOTOR_PIN_A  = 4;
const int LEFT_MOTOR_PIN_B  = 6;
const int RIGHT_MOTOR_PIN_A = 7;
const int RIGHT_MOTOR_PIN_B = 14;
const int MOTOR_FAULT_PIN   = 3;

const int SPEAKER_PIN = 21;

const int RED_LED_PIN    = 18;
const int YELLOW_LED_PIN = 19;
const int GREEN_LED_PIN  = 20;
const int OLED_SDA_PIN   = 23;
const int OLED_SCL_PIN   = 22;

const int BUTTON_PIN      = 9;
const int LOOP_TIMING_PIN = 5;

const bool HARDWARE_CHECK_MODE = false;

const int LEFT_SENSOR_WHITE   = 520;
const int LEFT_SENSOR_BLACK   = 225;
const int MIDDLE_SENSOR_WHITE = 520;
const int MIDDLE_SENSOR_BLACK = 225;
const int RIGHT_SENSOR_WHITE  = 520;
const int RIGHT_SENSOR_BLACK  = 225;

const int CRUISE_SPEED      = 45;
const int CORNER_SPEED      = 30;
const int MAX_REVERSE_SPEED = 20;

const float STEERING_STRENGTH = 1.2;
const float STEERING_DAMPING  = 1.0;

const int MOTOR_MIN_PWM = 100;
const float LEFT_MOTOR_TRIM  = 1.00;
const float RIGHT_MOTOR_TRIM = 1.00;

const int CONTROL_PERIOD_MS    = 20;
const int LINE_LOST_TIMEOUT_MS = 700;
const int SPEED_RAMP_TIME_MS   = 500;

const float SEES_LINE_DARKNESS = 0.5;

const float TURNING_STARTS_ABOVE  = 0.30;
const float STRAIGHT_AGAIN_BELOW  = 0.15;

const float NOTHING_UNDER_CAR_DARKNESS = 1.5;
const int   LIFTED_TIMEOUT_MS          = 300;

const int MUSIC_SWITCH_DELAY_MS = 200;

const int MOTOR_PWM_FREQUENCY = 20000;
const int OLED_I2C_ADDRESS    = 0x3C;
const int OLED_HEIGHT         = 64;
const bool OLED_UPSIDE_DOWN   = false;

const int REST    = 0;
const int NOTE_G4 = 392;
const int NOTE_A4 = 440;
const int NOTE_B4 = 494;
const int NOTE_C5 = 523;
const int NOTE_D5 = 587;
const int NOTE_E5 = 659;
const int NOTE_F5 = 698;
const int NOTE_G5 = 784;
const int NOTE_A5 = 880;
const int NOTE_B5 = 988;
const int NOTE_C6 = 1047;
const int NOTE_D6 = 1175;
const int NOTE_E6 = 1319;

struct Note {
  int frequency;
  int length;
};

const Note RACE_THEME_NOTES[] = {
  {NOTE_C5, 2}, {NOTE_E5, 2}, {NOTE_G5, 2}, {NOTE_C6, 2}, {NOTE_G5, 2}, {NOTE_E5, 2}, {NOTE_G5, 4},
  {NOTE_A4, 2}, {NOTE_C5, 2}, {NOTE_F5, 2}, {NOTE_A5, 2}, {NOTE_F5, 2}, {NOTE_C5, 2}, {NOTE_F5, 4},
  {NOTE_G4, 2}, {NOTE_B4, 2}, {NOTE_D5, 2}, {NOTE_G5, 2}, {NOTE_F5, 2}, {NOTE_D5, 2}, {NOTE_B4, 2}, {NOTE_D5, 2},
  {NOTE_C5, 2}, {NOTE_E5, 2}, {NOTE_G5, 2}, {NOTE_E5, 2}, {NOTE_C6, 4}, {REST, 4},
  {NOTE_E5, 2}, {NOTE_E5, 2}, {NOTE_G5, 2}, {NOTE_E5, 2}, {NOTE_D5, 2}, {NOTE_C5, 2}, {NOTE_D5, 4},
  {NOTE_E5, 2}, {NOTE_E5, 2}, {NOTE_G5, 2}, {NOTE_A5, 2}, {NOTE_G5, 4}, {NOTE_E5, 4},
  {NOTE_F5, 2}, {NOTE_F5, 2}, {NOTE_A5, 2}, {NOTE_F5, 2}, {NOTE_E5, 2}, {NOTE_D5, 2}, {NOTE_E5, 4},
  {NOTE_D5, 2}, {NOTE_B4, 2}, {NOTE_G4, 2}, {NOTE_B4, 2}, {NOTE_C5, 4}, {REST, 4},
};

const Note DRIFT_TUNE_NOTES[] = {
  {NOTE_E6, 1}, {NOTE_C6, 1}, {NOTE_E6, 1}, {NOTE_C6, 1},
  {NOTE_D6, 1}, {NOTE_B5, 1}, {NOTE_D6, 1}, {NOTE_B5, 1},
};

const Note STOP_JINGLE_NOTES[] = {
  {NOTE_G5, 2}, {NOTE_E5, 2}, {NOTE_C5, 4},
};

enum Tune { NO_TUNE, RACE_THEME, DRIFT_TUNE, STOP_JINGLE };

struct LineSensor {
  int pin;
  int whiteReading;
  int blackReading;
  int rawReading;
  float darkness;
  bool seesLine;
};

LineSensor leftSensor   = {LEFT_SENSOR_PIN,   LEFT_SENSOR_WHITE,   LEFT_SENSOR_BLACK,   0, 0.0, false};
LineSensor middleSensor = {MIDDLE_SENSOR_PIN, MIDDLE_SENSOR_WHITE, MIDDLE_SENSOR_BLACK, 0, 0.0, false};
LineSensor rightSensor  = {RIGHT_SENSOR_PIN,  RIGHT_SENSOR_WHITE,  RIGHT_SENSOR_BLACK,  0, 0.0, false};

enum CarState { STARTING, STRAIGHT, TURNING, STOPPED };
CarState carState = STARTING;
unsigned long stateStartMs = 0;
bool turningLeft = false;
const char *stopReason = "";

float linePosition = 0.0;
float previousLinePosition = 0.0;
float lastSeenLinePosition = 0.0;
unsigned long lineLostStartMs = 0;
unsigned long nothingUnderCarStartMs = 0;
float speedRamp = 0.0;

float currentLeftSpeed = 0;
float currentRightSpeed = 0;
bool motorFault = false;

Tune currentTune = NO_TUNE;
int currentNoteIndex = 0;
unsigned long noteStartMs = 0;
int noteLengthMs = 0;
bool noteIsSounding = false;
int savedRaceThemeIndex = 0;

const uint8_t FONT_5X7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x10,0x08,0x08,0x10,0x08},
};

bool oledConnected = false;
unsigned long lastOledUpdateMs = 0;
int nextOledRow = 0;
char textOnOled[2][17];

bool buttonWasDown = false;
unsigned long lastButtonChangeMs = 0;

unsigned long lastControlCycleMs = 0;
bool loopTimingPinHigh = false;

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(LOOP_TIMING_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FAULT_PIN, INPUT_PULLUP);

  ledcAttach(LEFT_MOTOR_PIN_A, MOTOR_PWM_FREQUENCY, 8);
  ledcAttach(LEFT_MOTOR_PIN_B, MOTOR_PWM_FREQUENCY, 8);
  ledcAttach(RIGHT_MOTOR_PIN_A, MOTOR_PWM_FREQUENCY, 8);
  ledcAttach(RIGHT_MOTOR_PIN_B, MOTOR_PWM_FREQUENCY, 8);
  setMotorSpeeds(0, 0);

  ledcAttach(SPEAKER_PIN, 2000, 10);
  ledcWriteTone(SPEAKER_PIN, 0);

  setupOled();

  if (HARDWARE_CHECK_MODE) {
    runHardwareCheck();
  }

  startDriving();
}

void loop() {
  checkButton();

  unsigned long now = millis();
  if (now - lastControlCycleMs >= CONTROL_PERIOD_MS) {
    lastControlCycleMs = now;
    runControlCycle();
  }

  updateMusic();
  updateOled();
}

void runControlCycle() {
  loopTimingPinHigh = !loopTimingPinHigh;
  digitalWrite(LOOP_TIMING_PIN, loopTimingPinHigh);

  readSensor(leftSensor);
  readSensor(middleSensor);
  readSensor(rightSensor);
  checkMotorFault();

  if (carState != STRAIGHT && carState != TURNING) {
    return;
  }

  bool nothingUnderCar = leftSensor.darkness > NOTHING_UNDER_CAR_DARKNESS
                      && middleSensor.darkness > NOTHING_UNDER_CAR_DARKNESS
                      && rightSensor.darkness > NOTHING_UNDER_CAR_DARKNESS;
  if (nothingUnderCar) {
    if (nothingUnderCarStartMs == 0) {
      nothingUnderCarStartMs = millis();
    }
    if (millis() - nothingUnderCarStartMs >= LIFTED_TIMEOUT_MS) {
      stopCar("LIFTED");
      return;
    }
  } else {
    nothingUnderCarStartMs = 0;
  }

  bool anySensorSeesLine = leftSensor.seesLine || middleSensor.seesLine || rightSensor.seesLine;

  if (anySensorSeesLine) {
    linePosition = findLinePosition();
    lastSeenLinePosition = linePosition;
    lineLostStartMs = 0;
  } else {
    if (lineLostStartMs == 0) {
      lineLostStartMs = millis();
    }
    if (millis() - lineLostStartMs >= LINE_LOST_TIMEOUT_MS) {
      stopCar("NO LINE");
      return;
    }

    if (lastSeenLinePosition > 0.2) {
      linePosition = 1.0;
    } else if (lastSeenLinePosition < -0.2) {
      linePosition = -1.0;
    } else {
      linePosition = 0.0;
    }
  }

  if (fabs(linePosition) > TURNING_STARTS_ABOVE) {
    changeState(TURNING);
  } else if (fabs(linePosition) < STRAIGHT_AGAIN_BELOW) {
    changeState(STRAIGHT);
  }

  if (linePosition < 0) {
    turningLeft = true;
  } else if (linePosition > 0) {
    turningLeft = false;
  }

  float positionChange = linePosition - previousLinePosition;
  previousLinePosition = linePosition;
  float steering = STEERING_STRENGTH * linePosition + STEERING_DAMPING * positionChange;

  float curveAmount = fabs(linePosition);
  if (curveAmount > 1.0) {
    curveAmount = 1.0;
  }
  float baseSpeed = CRUISE_SPEED - (CRUISE_SPEED - CORNER_SPEED) * curveAmount;

  if (speedRamp < 1.0) {
    speedRamp = speedRamp + (float)CONTROL_PERIOD_MS / SPEED_RAMP_TIME_MS;
    if (speedRamp > 1.0) {
      speedRamp = 1.0;
    }
  }
  baseSpeed = baseSpeed * speedRamp;

  float leftSpeed = baseSpeed;
  float rightSpeed = baseSpeed;
  if (steering > 0) {
    rightSpeed = baseSpeed * (1.0 - steering);
  } else {
    leftSpeed = baseSpeed * (1.0 + steering);
  }
  setMotorSpeeds(leftSpeed, rightSpeed);

  if (carState == STRAIGHT) {
    setStatusLeds(false, false, true);
  } else {
    setStatusLeds(false, true, false);
  }
  chooseMusicForState();
}

void changeState(CarState newState) {
  if (newState == carState) {
    return;
  }
  carState = newState;
  stateStartMs = millis();
}

void stopCar(const char *reason) {
  setMotorSpeeds(0, 0);
  stopReason = reason;
  changeState(STOPPED);
  setStatusLeds(true, false, false);
  playTune(STOP_JINGLE);
}

void startDriving() {
  changeState(STARTING);
  setMotorSpeeds(0, 0);
  runCountdown();

  speedRamp = 0.0;
  linePosition = 0.0;
  previousLinePosition = 0.0;
  lastSeenLinePosition = 0.0;
  lineLostStartMs = 0;
  nothingUnderCarStartMs = 0;
  changeState(STRAIGHT);
  lastControlCycleMs = millis();
}

void runCountdown() {
  playTune(NO_TUNE);
  oledWriteLine(0, "BuzzCar Team C02");

  oledWriteLine(1, "Ready...  3");
  setStatusLeds(true, false, false);
  beep(NOTE_A4, 250);
  delay(450);

  oledWriteLine(1, "Ready...  2");
  setStatusLeds(true, true, false);
  beep(NOTE_A4, 250);
  delay(450);

  oledWriteLine(1, "Ready...  1");
  setStatusLeds(true, true, true);
  beep(NOTE_A4, 250);
  delay(450);

  oledWriteLine(1, "GO!");
  setStatusLeds(false, false, true);
  beep(NOTE_A5, 400);
}

void checkButton() {
  bool buttonIsDown = (digitalRead(BUTTON_PIN) == LOW);
  if (buttonIsDown == buttonWasDown) {
    return;
  }

  if (millis() - lastButtonChangeMs > 50) {
    lastButtonChangeMs = millis();
    if (buttonIsDown) {
      if (carState == STOPPED) {
        startDriving();
      } else {
        stopCar("BUTTON");
      }
    }
  }
  buttonWasDown = buttonIsDown;
}

void readSensor(LineSensor &sensor) {
  int total = 0;
  for (int i = 0; i < 4; i++) {
    total = total + analogRead(sensor.pin);
  }
  sensor.rawReading = total / 4;

  float range = sensor.whiteReading - sensor.blackReading;
  if (range == 0) {
    range = 1;
  }
  sensor.darkness = (sensor.whiteReading - sensor.rawReading) / range;
  sensor.seesLine = (sensor.darkness > SEES_LINE_DARKNESS);
}

float findLinePosition() {
  float left = constrain(leftSensor.darkness, 0.0, 1.0);
  float middle = constrain(middleSensor.darkness, 0.0, 1.0);
  float right = constrain(rightSensor.darkness, 0.0, 1.0);

  float totalDarkness = left + middle + right;
  if (totalDarkness == 0) {
    return 0.0;
  }
  return (-1.0 * left + 0.0 * middle + 1.0 * right) / totalDarkness;
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_REVERSE_SPEED, 100);
  rightSpeed = constrain(rightSpeed, -MAX_REVERSE_SPEED, 100);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  driveOneMotor(LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, leftSpeed * LEFT_MOTOR_TRIM);
  driveOneMotor(RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B, rightSpeed * RIGHT_MOTOR_TRIM);
}

void driveOneMotor(int pinA, int pinB, float speed) {
  int pwm = speedToPwm(fabs(speed));

  if (speed > 0) {
    ledcWrite(pinA, 255);
    ledcWrite(pinB, 255 - pwm);
  } else if (speed < 0) {
    ledcWrite(pinA, 255 - pwm);
    ledcWrite(pinB, 255);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, 0);
  }
}

int speedToPwm(float speedPercent) {
  if (speedPercent < 1) {
    return 0;
  }
  if (speedPercent > 100) {
    speedPercent = 100;
  }
  return MOTOR_MIN_PWM + (255 - MOTOR_MIN_PWM) * speedPercent / 100;
}

void checkMotorFault() {
  motorFault = (digitalRead(MOTOR_FAULT_PIN) == LOW);
}

void setStatusLeds(bool redOn, bool yellowOn, bool greenOn) {
  digitalWrite(RED_LED_PIN, redOn);
  digitalWrite(YELLOW_LED_PIN, yellowOn);
  digitalWrite(GREEN_LED_PIN, greenOn);
}

void setupOled() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(OLED_I2C_ADDRESS);
  oledConnected = (Wire.endTransmission() == 0);
  if (!oledConnected) {
    return;
  }

  uint8_t comPinsSetting = 0x12;
  if (OLED_HEIGHT == 32) {
    comPinsSetting = 0x02;
  }

  uint8_t columnOrder = 0xA1;
  uint8_t rowOrder = 0xC8;
  if (OLED_UPSIDE_DOWN) {
    columnOrder = 0xA0;
    rowOrder = 0xC0;
  }

  uint8_t setupCommands[] = {
    0xAE,
    0xD5, 0x80,
    0xA8, OLED_HEIGHT - 1,
    0xD3, 0x00,
    0x40,
    0x8D, 0x14,
    0x20, 0x02,
    columnOrder,
    rowOrder,
    0xDA, comPinsSetting,
    0x81, 0xCF,
    0xD9, 0xF1,
    0xDB, 0x40,
    0xA4,
    0xA6,
    0x2E,
  };
  int commandCount = sizeof(setupCommands);
  for (int i = 0; i < commandCount; i++) {
    sendOledCommand(setupCommands[i]);
  }

  for (int page = 0; page < OLED_HEIGHT / 8; page++) {
    setOledPosition(page, 0);
    for (int column = 0; column < 128; column = column + 32) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write(0x40);
      for (int i = 0; i < 32; i++) {
        Wire.write(0);
      }
      Wire.endTransmission();
    }
  }

  sendOledCommand(0xAF);
}

void sendOledCommand(uint8_t command) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

void setOledPosition(int page, int column) {
  sendOledCommand(0xB0 + page);
  sendOledCommand(column & 0x0F);
  sendOledCommand(0x10 + (column >> 4));
}

void oledWriteLine(int row, const char *text) {
  if (!oledConnected) {
    return;
  }

  char paddedText[17];
  snprintf(paddedText, sizeof(paddedText), "%-16s", text);
  if (strcmp(paddedText, textOnOled[row]) == 0) {
    return;
  }
  strcpy(textOnOled[row], paddedText);

  int topPage = row * 2;
  if (OLED_HEIGHT == 64) {
    topPage = 1 + row * 4;
  }

  for (int half = 0; half < 2; half++) {
    setOledPosition(topPage + half, 16);
    Wire.beginTransmission(OLED_I2C_ADDRESS);
    Wire.write(0x40);
    for (int i = 0; i < 16; i++) {
      for (int column = 0; column < 6; column++) {
        uint16_t tallColumn = makeTallColumn(fontColumn(paddedText[i], column));
        if (half == 0) {
          Wire.write(tallColumn & 0xFF);
        } else {
          Wire.write(tallColumn >> 8);
        }
      }
    }
    Wire.endTransmission();
  }
}

uint8_t fontColumn(char letter, int column) {
  if (column >= 5) {
    return 0;
  }
  if (letter < ' ' || letter > '~') {
    letter = '?';
  }
  return FONT_5X7[letter - ' '][column];
}

uint16_t makeTallColumn(uint8_t column) {
  uint16_t tallColumn = 0;
  for (int bit = 0; bit < 8; bit++) {
    if (column & (1 << bit)) {
      tallColumn = tallColumn | (3 << (bit * 2));
    }
  }
  return tallColumn;
}

void updateOled() {
  if (millis() - lastOledUpdateMs < 100) {
    return;
  }
  lastOledUpdateMs = millis();

  char text[17];
  if (nextOledRow == 0) {
    makeTopOledLine(text);
    oledWriteLine(0, text);
    nextOledRow = 1;
  } else {
    makeBottomOledLine(text);
    oledWriteLine(1, text);
    nextOledRow = 0;
  }
}

void makeTopOledLine(char *text) {
  int averageSpeed = (currentLeftSpeed + currentRightSpeed) / 2;

  if (motorFault) {
    snprintf(text, 17, "MOTOR FAULT!");
  } else if (carState == STRAIGHT) {
    snprintf(text, 17, "STRAIGHT    %3d%%", averageSpeed);
  } else if (carState == TURNING && turningLeft) {
    snprintf(text, 17, "TURN LEFT   %3d%%", averageSpeed);
  } else if (carState == TURNING) {
    snprintf(text, 17, "TURN RIGHT  %3d%%", averageSpeed);
  } else if (carState == STOPPED) {
    bool showReason = (millis() / 2000) % 2 == 0;
    if (showReason) {
      snprintf(text, 17, "STOPPED:%s", stopReason);
    } else {
      snprintf(text, 17, "BUTTON = RESTART");
    }
  } else {
    snprintf(text, 17, "BuzzCar Team C02");
  }
}

void makeBottomOledLine(char *text) {
  snprintf(text, 17, "L:%s M:%s R:%s",
           onOrOff(leftSensor.seesLine),
           onOrOff(middleSensor.seesLine),
           onOrOff(rightSensor.seesLine));
}

const char *onOrOff(bool seesLine) {
  if (seesLine) {
    return "ON";
  }
  return "--";
}

void chooseMusicForState() {
  if (millis() - stateStartMs < MUSIC_SWITCH_DELAY_MS) {
    return;
  }

  if (carState == STRAIGHT) {
    playTune(RACE_THEME);
  } else if (carState == TURNING) {
    playTune(DRIFT_TUNE);
  }
}

void playTune(Tune newTune) {
  if (newTune == currentTune) {
    return;
  }

  if (currentTune == RACE_THEME) {
    savedRaceThemeIndex = currentNoteIndex;
  }

  currentTune = newTune;
  if (newTune == NO_TUNE) {
    ledcWriteTone(SPEAKER_PIN, 0);
    noteIsSounding = false;
    return;
  }

  if (newTune == RACE_THEME) {
    currentNoteIndex = savedRaceThemeIndex;
  } else {
    currentNoteIndex = 0;
  }
  startCurrentNote();
}

void updateMusic() {
  if (currentTune == NO_TUNE) {
    return;
  }

  unsigned long timeIntoNote = millis() - noteStartMs;

  if (noteIsSounding && timeIntoNote >= noteLengthMs * 85 / 100) {
    ledcWriteTone(SPEAKER_PIN, 0);
    noteIsSounding = false;
  }

  if (timeIntoNote >= noteLengthMs) {
    currentNoteIndex = currentNoteIndex + 1;

    if (currentNoteIndex >= tuneLength(currentTune)) {
      if (currentTune == STOP_JINGLE) {
        playTune(NO_TUNE);
        return;
      }
      currentNoteIndex = 0;
    }
    startCurrentNote();
  }
}

void startCurrentNote() {
  Note note = tuneNote(currentTune, currentNoteIndex);
  noteLengthMs = note.length * sixteenthNoteMs();
  noteStartMs = millis();
  ledcWriteTone(SPEAKER_PIN, note.frequency);
  noteIsSounding = (note.frequency > 0);
}

int sixteenthNoteMs() {
  int averageSpeed = (currentLeftSpeed + currentRightSpeed) / 2;
  averageSpeed = constrain(averageSpeed, 0, 100);
  return map(averageSpeed, 0, 100, 110, 50);
}

int tuneLength(Tune tune) {
  if (tune == RACE_THEME) {
    return sizeof(RACE_THEME_NOTES) / sizeof(RACE_THEME_NOTES[0]);
  } else if (tune == DRIFT_TUNE) {
    return sizeof(DRIFT_TUNE_NOTES) / sizeof(DRIFT_TUNE_NOTES[0]);
  } else if (tune == STOP_JINGLE) {
    return sizeof(STOP_JINGLE_NOTES) / sizeof(STOP_JINGLE_NOTES[0]);
  }
  return 0;
}

Note tuneNote(Tune tune, int index) {
  if (tune == RACE_THEME) {
    return RACE_THEME_NOTES[index];
  } else if (tune == DRIFT_TUNE) {
    return DRIFT_TUNE_NOTES[index];
  }
  return STOP_JINGLE_NOTES[index];
}

void beep(int frequency, int lengthMs) {
  ledcWriteTone(SPEAKER_PIN, frequency);
  delay(lengthMs);
  ledcWriteTone(SPEAKER_PIN, 0);
}

void runHardwareCheck() {
  oledWriteLine(0, "HARDWARE CHECK");

  oledWriteLine(1, "LEDs + speaker");
  setStatusLeds(true, false, false);
  beep(NOTE_C5, 300);
  delay(200);
  setStatusLeds(false, true, false);
  beep(NOTE_E5, 300);
  delay(200);
  setStatusLeds(false, false, true);
  beep(NOTE_G5, 300);
  delay(200);
  setStatusLeds(false, false, false);

  oledWriteLine(1, "LEFT wheel FWD");
  setMotorSpeeds(50, 0);
  delay(1500);
  setMotorSpeeds(0, 0);
  delay(500);

  oledWriteLine(1, "LEFT wheel REV");
  setMotorSpeeds(-MAX_REVERSE_SPEED, 0);
  delay(1500);
  setMotorSpeeds(0, 0);
  delay(500);

  oledWriteLine(1, "RIGHT wheel FWD");
  setMotorSpeeds(0, 50);
  delay(1500);
  setMotorSpeeds(0, 0);
  delay(500);

  oledWriteLine(1, "RIGHT wheel REV");
  setMotorSpeeds(0, -MAX_REVERSE_SPEED);
  delay(1500);
  setMotorSpeeds(0, 0);

  while (true) {
    readSensor(leftSensor);
    readSensor(middleSensor);
    readSensor(rightSensor);

    char text[17];
    snprintf(text, sizeof(text), "%4d %4d %4d",
             leftSensor.rawReading, middleSensor.rawReading, rightSensor.rawReading);
    oledWriteLine(0, text);
    makeBottomOledLine(text);
    oledWriteLine(1, text);

    Serial.printf("raw readings:  L=%4d  M=%4d  R=%4d     sees line:  L:%s  M:%s  R:%s\n",
                  leftSensor.rawReading, middleSensor.rawReading, rightSensor.rawReading,
                  onOrOff(leftSensor.seesLine), onOrOff(middleSensor.seesLine), onOrOff(rightSensor.seesLine));
    delay(250);
  }
}

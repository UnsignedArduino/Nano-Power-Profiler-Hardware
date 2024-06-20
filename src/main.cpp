#include <Adafruit_INA260.h>
#include <Arduino.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

enum State { ERROR = -1, READY = 0, ACTIVE = 1 };
enum State state = READY;

const size_t MAX_COMMAND_BUF = 2;
char commandBuf[MAX_COMMAND_BUF];

float currVoltage = 0;
float currCurrent = 0;
float currPower = 0;

void printUint64(uint64_t value) {
  if (value >= 10) {
    printUint64(value / 10);
  }

  Serial.print((byte)value % 10);
}

void sample() {
  currVoltage = ina260.readBusVoltage();
  currCurrent = ina260.readCurrent();
  currPower = ina260.readPower();
}

void printState() {
  switch (state) {
    case ERROR:
      Serial.println("error");
      break;
    case READY:
      Serial.println("ready");
      break;
    case ACTIVE:
      Serial.println("active");
      break;
  }
}

void printVolts() {
  Serial.print(currVoltage);
  Serial.println(" mV");
}

void printCurrent() {
  Serial.print(currCurrent);
  Serial.println(" mA");
}

void printPower() {
  Serial.print(currPower);
  Serial.println(" mW");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!ina260.begin()) {
    state = ERROR;
  }
}

void loop() {
  if (state != ERROR) {
    sample();
  }

  if (Serial.available()) {
    size_t commandLen =
      Serial.readBytesUntil('\n', commandBuf, MAX_COMMAND_BUF);
    commandBuf[commandLen] = '\0';
  }

  const char command = toLowerCase(commandBuf[0]);
  if (command != '\0') {
    if (command == 's') {
      printState();
    } else if (command == 'v') {
      printVolts();
    } else if (command == 'c') {
      printCurrent();
    } else if (command == 'p') {
      printPower();
    } else if (command == 'a') {
      printVolts();
      printCurrent();
      printPower();
    } else if (command == 'i') {
      state = ACTIVE;
    } else if (command == 't') {
      state = READY;
    }
    Serial.println("ok");
    memset(commandBuf, 0, MAX_COMMAND_BUF);
  }

  static int16_t blinkPeriod = 0;
  static uint32_t lastBlink = 0;
  if (state == ERROR) {
    blinkPeriod = 100;
  } else if (state == READY) {
    blinkPeriod = -1;
  } else {
    blinkPeriod = 0;
  }

  if (blinkPeriod == -1) {
    digitalWrite(LED_BUILTIN, LOW);
  } else if (blinkPeriod == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    if (millis() > lastBlink + blinkPeriod) {
      lastBlink = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

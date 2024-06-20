#include <Adafruit_INA260.h>
#include <Arduino.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

enum State { ERROR = -1, READY = 0, ACTIVE = 1 };
enum State state = READY;

const size_t MAX_COMMAND_BUF = 2;
char commandBuf[MAX_COMMAND_BUF];

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!ina260.begin()) {
    state = ERROR;
  }
}

void loop() {
  if (Serial.available()) {
    size_t commandLen =
      Serial.readBytesUntil('\n', commandBuf, MAX_COMMAND_BUF);
    commandBuf[commandLen] = '\0';

    const char command = toLowerCase(commandBuf[0]);

    if (command == 's') {
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
    } else if (command == 'c') {
      Serial.print(ina260.readCurrent());
      Serial.println(" mA");
    } else if (command == 'v') {
      Serial.print(ina260.readBusVoltage());
      Serial.println(" mV");
    } else if (command == 'p') {
      Serial.print(ina260.readPower());
      Serial.println(" mW");
    }
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

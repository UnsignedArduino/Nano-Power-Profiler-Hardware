#include <Adafruit_INA260.h>
#include <Arduino.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

enum State { ERROR = -1, READY = 0, ACTIVE = 1 };
enum State state = READY;

float currVoltage = 0;
float currCurrent = 0;
float currPower = 0;

uint32_t measureStartTime = 0;
uint32_t measureEndTime = 0;
uint64_t measureSamples = 0;
uint32_t lastSampleTime = 0;

float minVoltage = 0;
float maxVoltage = 0;
float voltageSum = 0;

float minCurrent = 0;
float maxCurrent = 0;
float currentSum = 0;

float minPower = 0;
float maxPower = 0;
float powerSum = 0;

void printUint64(uint64_t value) {
  if (value >= 10) {
    printUint64(value / 10);
  }

  Serial.print((byte)value % 10);
}

void initiateMeasure() {
  if (state == ACTIVE) {
    Serial.println("error: already measuring");
    return;
  }
  state = ACTIVE;
  measureStartTime = millis();
  measureEndTime = measureStartTime;
  measureSamples = 1;
  minVoltage = currVoltage;
  maxVoltage = currVoltage;
  voltageSum = currVoltage;
  minCurrent = currCurrent;
  maxCurrent = currCurrent;
  currentSum = currCurrent;
  minPower = currPower;
  maxPower = currPower;
  powerSum = currPower;
  Serial.println("initiate measure");
}

void printMeasureStats() {
  Serial.print("samples: ");
  printUint64(measureSamples);
  Serial.print("   time: ");
  const uint32_t measureTime = measureEndTime - measureStartTime;
  Serial.print(measureTime);
  Serial.print(" ms   ");
  Serial.print((float)measureSamples / ((float)measureTime / 1000));
  Serial.println(" hz");
}

void terminateMeasure() {
  if (state != ACTIVE) {
    Serial.println("error: must be measuring");
    return;
  }
  state = READY;
  measureEndTime = millis();
  Serial.println("terminate measure");
  printMeasureStats();
}

void sample() {
  currVoltage = ina260.readBusVoltage();
  currCurrent = ina260.readCurrent();
  currPower = ina260.readPower();
  lastSampleTime = millis();
  if (state == ACTIVE) {
    measureEndTime = millis();
    measureSamples++;

    if (currVoltage > maxVoltage) {
      maxVoltage = currVoltage;
    }
    if (currVoltage < minVoltage) {
      minVoltage = currVoltage;
    }
    voltageSum += currVoltage;

    if (currCurrent > maxCurrent) {
      maxCurrent = currCurrent;
    }
    if (currCurrent < minCurrent) {
      minCurrent = currCurrent;
    }
    currentSum += currCurrent;

    if (currPower > maxPower) {
      maxPower = currPower;
    }
    if (currPower < minPower) {
      minPower = currPower;
    }
    powerSum += currPower;
  }
}

void printState() {
  Serial.print("state: ");
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
  Serial.print("volts: ");
  Serial.print(currVoltage, 0);
  if (state == ACTIVE) {
    Serial.print(" mV   min: ");
    Serial.print(minVoltage, 0);
    Serial.print(" mV   max: ");
    Serial.print(maxVoltage, 0);
    Serial.print(" mV   avg: ");
    Serial.print(voltageSum / (float)measureSamples, 0);
  }
  Serial.println(" mV");
}

void printCurrent() {
  Serial.print("current: ");
  Serial.print(currCurrent, 0);
  if (state == ACTIVE) {
    Serial.print(" mA   min: ");
    Serial.print(minCurrent, 0);
    Serial.print(" mA   max: ");
    Serial.print(maxCurrent, 0);
    Serial.print(" mA   avg: ");
    const float average = currentSum / (float)measureSamples;
    Serial.print(average, 0);
    Serial.print(" mA   total: ");
    const uint32_t measureTime = measureEndTime - measureStartTime;
    const float milliAmpMilliSeconds = average * (float)measureTime;
    Serial.print(milliAmpMilliSeconds / 3600000);
    Serial.print(" mAh   rate: ");
    Serial.print(milliAmpMilliSeconds / (float)measureTime);
    Serial.println(" mA/h");
  } else {
    Serial.println(" mA");
  }
}

void printPower() {
  Serial.print("power: ");
  Serial.print(currPower, 0);
  if (state == ACTIVE) {
    Serial.print(" mW   min: ");
    Serial.print(minPower, 0);
    Serial.print(" mW   max: ");
    Serial.print(maxPower, 0);
    Serial.print(" mW   avg: ");
    const float average = powerSum / (float)measureSamples;
    Serial.print(average, 0);
    const uint32_t measureTime = measureEndTime - measureStartTime;
    const float milliWattMilliSeconds = average * (float)measureTime;
    Serial.print(" mW   total: ");
    Serial.print(milliWattMilliSeconds / 3600000);
    Serial.print(" mWh   rate: ");
    Serial.print(milliWattMilliSeconds / (float)measureTime);
    Serial.println(" mW/h");
  } else {
    Serial.println(" mW");
  }
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

  char command = '\0';

  if (Serial.available()) {
    command = toLowerCase(Serial.read());
  }

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
      initiateMeasure();
    } else if (command == 't') {
      terminateMeasure();
    } else if (command == 'h') {
      printMeasureStats();
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

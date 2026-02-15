#include "sScreen.h"
#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SD.h>
TFT_eSPI tft = TFT_eSPI();

#include "hardware/pio.h"
#include "quadrature.pio.h"
#define QUADRATURE_A_PIN 9
#define QUADRATURE_B_PIN 10
PIO pio = pio0;
uint offset, sm;
int32_t encoder_value = 0;
int32_t last_encoder_value = 0;
volatile bool swPressed = false;

#define IWIDTH  480
#define IHEIGHT 320

#define tftRotation 1
#define textColour TFT_WHITE
#define textSize 3
#define menuTextSize 2
#define NUM_OPTIONS 2
#define DRAW_INTERVAL 250

#include "Adafruit_MAX31855.h"
#define MAXDO   5
#define MAXCS   6
#define MAXCLK  7
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

int esc = 11;
int relay = 18;
int ledpin = 25;
const int pot = 26;
char analog[12];

const int numReadings = 20;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

#define SD_CS 13
#define SD_CD 19

File logFile;
unsigned long lastLogTime = 0;
const unsigned long logInterval = 10000;
char logFileName[20];
bool loggingEnabled = false;
bool readProfileEnabled = false;
bool sdCardPresent = false;
bool sdInitialized = false;
int selectedOption = 0;

struct RampStep {
  float targetTemp;
  unsigned long duration;
  bool ssrEscState;
  bool ssrRelayState;
};
RampStep steps[10];
int numSteps = 0;
int currentStep = 0;
float startTemp = 0;
unsigned long stepStartTime = 0;
bool rampActive = false;

float currentTempC = 0;
int currentInternalTemp = 0;
char tc_str[10];

unsigned long lastDrawTime = 0;
float lastTempC = 0;
int lastAverage = 0;
int32_t lastEncoderVal = 0;
int lastSelectedOption = -1;
bool lastLogging = false;
bool lastProfile = false;
bool lastSdPresent = false;
bool lastRampActive = false;
int lastCurrentStep = -1;
unsigned long lastRemaining = 0;
char lastRampText[100];
int lastInternalTemp = 0;
char lastTcStr[10];

void setup() {
  Serial.begin(115200);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(8), reSw, RISING);
  tft.init();
  tft.setRotation(tftRotation);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, IWIDTH, IHEIGHT, sScreen);
  delay(2000);
  tft.setSwapBytes(false);
  tft.fillScreen(TFT_BLACK);

  pinMode(esc, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(ledpin, OUTPUT);
  digitalWrite(esc, LOW);
  digitalWrite(relay, LOW);

  pinMode(SD_CD, INPUT_PULLUP);
  checkSDCard();

  analogReadResolution(12);

  if (!thermocouple.begin()) Serial.println("MAX31855 not found!");

  offset = pio_add_program(pio, &quadrature_program);
  sm = pio_claim_unused_sm(pio, true);
  quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  delay(500);

  // Init last values to force full initial draw
  strcpy(lastTcStr, "~");
  strcpy(lastRampText, "");
  lastSelectedOption = -1;
  lastEncoderVal = encoder_value - 1;
  lastAverage = average - 1;
  lastInternalTemp = currentInternalTemp - 1;
  lastSdPresent = !sdCardPresent;
  lastLogging = !loggingEnabled;
  lastProfile = !readProfileEnabled;

  // Force full initial menu draw at boot
  drawMenu(0, 0);

  // Sync last values after initial draw
  lastSdPresent = sdCardPresent;
  lastSelectedOption = selectedOption;
  lastLogging = loggingEnabled;
  lastProfile = readProfileEnabled;
  lastEncoderVal = encoder_value;
  lastAverage = average;
  lastInternalTemp = currentInternalTemp;
  lastDrawTime = millis();
}

void loop() {
  digitalWrite(ledpin, LOW);
  readPicoT();

  currentTempC = thermocouple.readCelsius();
  currentInternalTemp = thermocouple.readInternal();
  if (isnan(currentTempC)) {
    uint8_t fault = thermocouple.readError();
    if (fault & MAX31855_FAULT_OPEN) strcpy(tc_str, "OPEN");
    else if (fault & MAX31855_FAULT_SHORT_GND) strcpy(tc_str, "GND");
    else if (fault & MAX31855_FAULT_SHORT_VCC) strcpy(tc_str, "VCC");
    else strcpy(tc_str, "ERR");
  } else {
    dtostrf(currentTempC, 4, 1, tc_str);
  }

  pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
  int32_t raw = (int32_t)pio_sm_get_blocking(pio, sm);
  int32_t delta = raw - last_encoder_value;
  if (delta != 0) {
    selectedOption = ((selectedOption + delta) % NUM_OPTIONS + NUM_OPTIONS) % NUM_OPTIONS;
  }
  last_encoder_value = raw;
  encoder_value = raw;

  checkSDCard();

  noInterrupts();
  bool pressed = swPressed;
  swPressed = false;
  interrupts();

  if (pressed) {
    if (selectedOption == 0) {
      loggingEnabled = !loggingEnabled;
      Serial.print("Logging: ");
      Serial.println(loggingEnabled ? "ON" : "OFF");
    } else if (selectedOption == 1) {
      readProfileEnabled = !readProfileEnabled;
      Serial.print("Read Profile: ");
      Serial.println(readProfileEnabled ? "ON" : "OFF");
      if (readProfileEnabled && sdCardPresent) {
        readProfile();
        if (numSteps > 0 && !rampActive) {
          currentStep = 0;
          startTemp = isnan(currentTempC) ? 25.0f : currentTempC;
          stepStartTime = millis();
          rampActive = true;
        }
      } else {
        rampActive = false;
        numSteps = 0;
        currentStep = 0;
        startTemp = 0;
        digitalWrite(esc, LOW);
        digitalWrite(relay, LOW);
      }
    }
  }

  // Handle ramp if active
  float setpointTemp = isnan(currentTempC) ? 0 : currentTempC;
  unsigned long remainingTime = 0;
  if (rampActive && currentStep < numSteps) {
    unsigned long elapsedTime = millis() - stepStartTime;
    unsigned long durationMs = steps[currentStep].duration * 1000UL;
    float stepStartTemp = (currentStep == 0) ? startTemp : steps[currentStep - 1].targetTemp;
    float stepTargetTemp = steps[currentStep].targetTemp;

    if (elapsedTime >= durationMs) {
      currentStep++;
      if (currentStep >= numSteps) {
        rampActive = false;
        setpointTemp = stepTargetTemp;
        remainingTime = 0;
        digitalWrite(esc, LOW);
        digitalWrite(relay, LOW);
      } else {
        stepStartTime = millis();
        float newStepStart = stepTargetTemp;
        float newStepTarget = steps[currentStep].targetTemp;
        setpointTemp = newStepStart;
        remainingTime = steps[currentStep].duration;
      }
    } else {
      float progress = (float)elapsedTime / durationMs;
      setpointTemp = stepStartTemp + (stepTargetTemp - stepStartTemp) * progress;
      remainingTime = (durationMs - elapsedTime) / 1000UL;
    }

    if (rampActive && !isnan(currentTempC)) {
      if (currentTempC < setpointTemp) {
        digitalWrite(esc, steps[currentStep].ssrEscState ? HIGH : LOW);
        digitalWrite(relay, steps[currentStep].ssrRelayState ? HIGH : LOW);
      } else {
        digitalWrite(esc, LOW);
        digitalWrite(relay, LOW);
      }
    } else {
      digitalWrite(esc, LOW);
      digitalWrite(relay, LOW);
    }
  }

  // Draw if needed
  bool needsRedraw = (millis() - lastDrawTime >= DRAW_INTERVAL) ||
                     (fabs(currentTempC - lastTempC) > 1.0) ||
                     (abs(average - lastAverage) > 5) ||
                     (abs(encoder_value - lastEncoderVal) != 0) ||
                     (selectedOption != lastSelectedOption) ||
                     (loggingEnabled != lastLogging) ||
                     (readProfileEnabled != lastProfile) ||
                     (sdCardPresent != lastSdPresent) ||
                     (rampActive != lastRampActive) ||
                     (currentStep != lastCurrentStep) ||
                     (remainingTime != lastRemaining) ||
                     (abs(currentInternalTemp - lastInternalTemp) > 2);

  if (needsRedraw) {
    drawMenu(setpointTemp, remainingTime);
    lastDrawTime = millis();
    lastTempC = currentTempC;
    lastAverage = average;
    lastEncoderVal = encoder_value;
    lastSelectedOption = selectedOption;
    lastLogging = loggingEnabled;
    lastProfile = readProfileEnabled;
    lastSdPresent = sdCardPresent;
    lastRampActive = rampActive;
    lastCurrentStep = currentStep;
    lastRemaining = remainingTime;
    lastInternalTemp = currentInternalTemp;
  }

  delay(1);

  unsigned long currentTime = millis();
  if (loggingEnabled && sdCardPresent && sdInitialized && (currentTime - lastLogTime >= logInterval)) {
    logData(setpointTemp);
    lastLogTime = currentTime;
  }
}

void reSw() {
  static unsigned long lastPress = 0;
  unsigned long now = millis();
  if (now - lastPress > 50) {
    swPressed = true;
    digitalWrite(ledpin, HIGH);
  }
  lastPress = now;
}

void readPicoT() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(pot);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) readIndex = 0;
  average = total / numReadings;
}

void checkSDCard() {
  bool currentState = digitalRead(SD_CD) == LOW;
  if (currentState != sdCardPresent) {
    sdCardPresent = currentState;
    if (sdCardPresent) {
      Serial.println("SD card inserted. Initializing...");
      SPI1.begin();
      if (SD.begin(SD_CS, SPI1)) {
        Serial.println("SD card initialized successfully.");
        sdInitialized = true;

        int fileCounter = 0;
        snprintf(logFileName, sizeof(logFileName), "log_%d.csv", fileCounter);
        while (SD.exists(logFileName)) {
          fileCounter++;
          snprintf(logFileName, sizeof(logFileName), "log_%d.csv", fileCounter);
        }

        logFile = SD.open(logFileName, FILE_WRITE);
        if (logFile) {
          Serial.print("Created log file: ");
          Serial.println(logFileName);
          logFile.println("Timestamp,PotValue,EncoderValue,InternalTemp,ThermocoupleTemp,SetpointTemp,SSR_Esc,SSR_Relay,Step");
          logFile.close();
        } else {
          Serial.println("Error creating log file.");
          sdInitialized = false;
        }

        if (readProfileEnabled) {
          readProfile();
          if (numSteps > 0 && !rampActive) {
            currentStep = 0;
            startTemp = isnan(currentTempC) ? 25.0f : currentTempC;
            stepStartTime = millis();
            rampActive = true;
          }
        }
      } else {
        Serial.println("SD card initialization failed.");
        sdInitialized = false;
      }
    } else {
      Serial.println("SD card removed. Disabling logging and profile reading...");
      SD.end();
      SPI1.end();
      sdInitialized = false;
      loggingEnabled = false;
      readProfileEnabled = false;
      rampActive = false;
      numSteps = 0;
      currentStep = 0;
      startTemp = 0;
      digitalWrite(esc, LOW);
      digitalWrite(relay, LOW);
    }
  }
}

void drawPaddedStr(int x, int y, const char* str, int padWidth, uint16_t fg, uint16_t bg) {
  char buf[40];
  int len = strlen(str);
  int pad = padWidth - len;
  if (pad < 0) pad = 0;
  snprintf(buf, sizeof(buf), "%s", str);
  for (int i = len; i < len + pad && i < (int)sizeof(buf) - 1; i++) buf[i] = ' ';
  buf[len + pad] = '\0';
  tft.setTextColor(fg, bg);
  tft.setCursor(x, y);
  tft.print(buf);
}

void drawMenu(float setpointTemp, unsigned long remainingTime) {
  tft.startWrite();

  tft.setTextSize(menuTextSize);

  // SD status
  if (sdCardPresent != lastSdPresent) {
    drawPaddedStr(11, 13, sdCardPresent ? "SD: Present" : "SD: Not Present", 20, textColour, TFT_BLACK);
  }

  // Logging option
  uint16_t logBg = selectedOption == 0 ? TFT_BLUE : TFT_BLACK;
  uint16_t profBg = selectedOption == 1 ? TFT_BLUE : TFT_BLACK;

  if (selectedOption != lastSelectedOption || loggingEnabled != lastLogging) {
    tft.fillRect(10, 40, 280, 25, logBg);
    drawPaddedStr(11, 43, loggingEnabled ? "Logging: ON" : "Logging: OFF", 20, textColour, logBg);
  }

  // Profile option
  if (selectedOption != lastSelectedOption || readProfileEnabled != lastProfile) {
    tft.fillRect(10, 70, 280, 25, profBg);
    drawPaddedStr(11, 73, readProfileEnabled ? "Read Profile: ON" : "Read Profile: OFF", 20, textColour, profBg);
  }

  // Ramp status
  if (rampActive) {
    if (rampActive != lastRampActive || currentStep != lastCurrentStep || remainingTime != lastRemaining) {
      char rampText[50];
      snprintf(rampText, sizeof(rampText), "Step %d/%d: %.1fC target, %lu s left", currentStep + 1, numSteps, setpointTemp, remainingTime);
      drawPaddedStr(11, 103, rampText, 38, textColour, TFT_BLACK);
    }
  } else if (lastRampActive) {
    tft.fillRect(10, 100, 460, 25, TFT_BLACK);
  }

  tft.setTextSize(textSize);

  // Pot average
  if (abs(average - lastAverage) > 0) {
    itoa(average, analog, 10);
    drawPaddedStr(11, 133, analog, 8, textColour, TFT_BLACK);
  }

  // Encoder
  if (encoder_value != lastEncoderVal) {
    itoa(encoder_value, analog, 10);
    drawPaddedStr(11, 173, analog, 8, textColour, TFT_BLACK);
  }

  // Internal temp
  if (abs(currentInternalTemp - lastInternalTemp) > 0) {
    itoa(currentInternalTemp, analog, 10);
    drawPaddedStr(11, 213, analog, 8, textColour, TFT_BLACK);
  }

  // TC temp
  if (strcmp(tc_str, lastTcStr) != 0) {
    drawPaddedStr(11, 253, tc_str, 8, textColour, TFT_BLACK);
    strcpy(lastTcStr, tc_str);
  }

  tft.endWrite();
}

void readProfile() {
  File profileFile = SD.open("profile.csv", FILE_READ);
  if (profileFile) {
    numSteps = 0;
    char buffer[50];

    while (profileFile.available() && profileFile.read() != '\n');

    while (profileFile.available() && numSteps < 10) {
      size_t index = 0;
      while (profileFile.available() && index < sizeof(buffer) - 1) {
        char ch = profileFile.read();
        if (ch == '\n' || ch == '\r') break;
        buffer[index++] = ch;
      }
      buffer[index] = '\0';

      if (index == 0) continue;

      float target;
      unsigned long duration;
      int escState, relayState;
      int matches = sscanf(buffer, "%f,%lu,%d,%d", &target, &duration, &escState, &relayState);
      if (matches == 4 && duration > 0 && !isnan(target)) {
        steps[numSteps].targetTemp = target;
        steps[numSteps].duration = duration;
        steps[numSteps].ssrEscState = (escState != 0);
        steps[numSteps].ssrRelayState = (relayState != 0);
        numSteps++;
      } else {
        Serial.print("Skipped invalid line: ");
        Serial.println(buffer);
      }
    }
    profileFile.close();

    Serial.print("Profile loaded: ");
    Serial.print(numSteps);
    Serial.println(" steps");
  } else {
    Serial.println("Error opening profile.csv");
    readProfileEnabled = false;
    numSteps = 0;
  }
}

void logData(float setpointTemp) {
  logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile) {
    unsigned long timestamp = millis();
    logFile.print(timestamp);
    logFile.print(",");
    logFile.print(average);
    logFile.print(",");
    logFile.print(encoder_value);
    logFile.print(",");
    logFile.print(currentInternalTemp);
    logFile.print(",");
    if (isnan(currentTempC)) logFile.print(tc_str);
    else logFile.print(currentTempC);
    logFile.print(",");
    logFile.print(setpointTemp);
    logFile.print(",");
    logFile.print(digitalRead(esc));
    logFile.print(",");
    logFile.print(digitalRead(relay));
    logFile.print(",");
    int logStep = rampActive ? (currentStep + 1) : 0;
    logFile.print(logStep);
    logFile.println();
    logFile.close();
    Serial.println("Logged data to SD card.");
  } else {
    Serial.println("Error opening log file for writing.");
  }
}

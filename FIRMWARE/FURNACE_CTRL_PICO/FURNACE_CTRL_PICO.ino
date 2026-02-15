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
#define NUM_OPTIONS 3
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

// Profile steps — single-column target temperatures
struct RampStep {
  float targetTemp;
};
RampStep* steps = NULL;
int numSteps = 0;
int currentStep = 0;
float startTemp = 0;
bool rampActive = false;
bool rampHeating = true;

// Learned thermal data per step
struct LearnedStep {
  float targetTemp;
  float prevSettledTemp;
  float timeToReach;
  float overshoot;
  float settleTime;
  float rateOfRise;
  float earlyCutoff;
};
LearnedStep* learned = NULL;
int numLearned = 0;
bool learnedDataLoaded = false;

// Learn mode state
enum LearnPhase { LEARN_IDLE, LEARN_HEATING, LEARN_OVERSHOOT, LEARN_SETTLING, LEARN_COOLING, LEARN_COMPLETE };
bool learnModeEnabled = false;
bool learnModeActive = false;
LearnPhase learnPhase = LEARN_IDLE;
unsigned long learnStepStartTime = 0;
float learnPeakTemp = 0;
unsigned long learnTargetCrossTime = 0;
float learnPrevTemp = 0;
unsigned long learnPrevTime = 0;
float learnRateAtCutoff = 0;
#define LEARN_TIMEOUT_MS 1800000UL

float currentTempC = 0;
int currentInternalTemp = 0;
char tc_str[10];

// Display change tracking
unsigned long lastDrawTime = 0;
float lastTempC = 0;
int lastAverage = 0;
int32_t lastEncoderVal = 0;
int lastSelectedOption = -1;
bool lastLogging = false;
bool lastProfile = false;
bool lastSdPresent = false;
bool lastRampActive = false;
bool lastLearnMode = false;
bool lastLearnActive = false;
LearnPhase lastLearnPhase = LEARN_IDLE;
int lastCurrentStep = -1;
int lastInternalTemp = 0;
char lastTcStr[10];

// Forward declarations
void readProfile();
void loadLearnedData();
void saveLearnedData();
float tickLearnMode();
void drawMenu(float setpointTemp);
void stopAllOutputs();
void freeProfile();
void freeLearned();
void resetLearnState();

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
  stopAllOutputs();

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
  lastSelectedOption = -1;
  lastEncoderVal = encoder_value - 1;
  lastAverage = average - 1;
  lastInternalTemp = currentInternalTemp - 1;
  lastSdPresent = !sdCardPresent;
  lastLogging = !loggingEnabled;
  lastProfile = !readProfileEnabled;
  lastLearnMode = !learnModeEnabled;

  drawMenu(0);

  // Sync after initial draw
  lastSdPresent = sdCardPresent;
  lastSelectedOption = selectedOption;
  lastLogging = loggingEnabled;
  lastProfile = readProfileEnabled;
  lastLearnMode = learnModeEnabled;
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
      // Toggle logging
      loggingEnabled = !loggingEnabled;
      Serial.print("Logging: ");
      Serial.println(loggingEnabled ? "ON" : "OFF");

    } else if (selectedOption == 1) {
      // Toggle profile run
      readProfileEnabled = !readProfileEnabled;
      Serial.print("Profile: ");
      Serial.println(readProfileEnabled ? "ON" : "OFF");
      if (readProfileEnabled) {
        // Deactivate learn mode
        learnModeEnabled = false;
        learnModeActive = false;
        learnPhase = LEARN_IDLE;
        if (sdCardPresent) {
          readProfile();
          loadLearnedData();
          if (numSteps > 0) {
            currentStep = 0;
            startTemp = isnan(currentTempC) ? 25.0f : currentTempC;
            rampActive = true;
            rampHeating = true;
          }
        }
      } else {
        rampActive = false;
        freeProfile();
        stopAllOutputs();
      }

    } else if (selectedOption == 2) {
      // Toggle learn mode
      learnModeEnabled = !learnModeEnabled;
      Serial.print("Learn: ");
      Serial.println(learnModeEnabled ? "ON" : "OFF");
      if (learnModeEnabled) {
        // Deactivate profile run
        readProfileEnabled = false;
        rampActive = false;
        if (sdCardPresent) {
          readProfile();
          if (numSteps > 0) {
            // Load previous learned data for early cutoff
            loadLearnedData();
            // Allocate learned array for this run
            LearnedStep* newLearned = (LearnedStep*)malloc(numSteps * sizeof(LearnedStep));
            if (newLearned) {
              memset(newLearned, 0, numSteps * sizeof(LearnedStep));
              int copyCount = (numLearned < numSteps) ? numLearned : numSteps;
              for (int i = 0; i < copyCount; i++) {
                newLearned[i] = learned[i];
              }
              free(learned);
              learned = newLearned;
              numLearned = copyCount;

              currentStep = 0;
              startTemp = isnan(currentTempC) ? 25.0f : currentTempC;
              learnStepStartTime = millis();
              learnPeakTemp = 0;
              learnTargetCrossTime = 0;
              learnRateAtCutoff = 0;
              learnPrevTemp = isnan(currentTempC) ? 25.0f : currentTempC;
              learnPrevTime = millis();
              learnModeActive = true;

              float target = steps[0].targetTemp;
              if (target < learnPrevTemp) {
                learnPhase = LEARN_COOLING;
              } else {
                learnPhase = LEARN_HEATING;
              }
              Serial.print("Learn mode started: ");
              Serial.print(numSteps);
              Serial.println(" steps");
            } else {
              Serial.println("Failed to allocate learned array");
              learnModeEnabled = false;
            }
          } else {
            learnModeEnabled = false;
          }
        }
      } else {
        learnModeActive = false;
        learnPhase = LEARN_IDLE;
        stopAllOutputs();
      }
    }
  }

  // Handle profile run
  float setpointTemp = isnan(currentTempC) ? 0 : currentTempC;
  if (rampActive && currentStep < numSteps && !isnan(currentTempC)) {
    float target = steps[currentStep].targetTemp;
    setpointTemp = target;

    // Find learned early cutoff for this step
    float cutoff = target;
    if (learnedDataLoaded && currentStep < numLearned &&
        learned[currentStep].targetTemp == target &&
        learned[currentStep].earlyCutoff > 0) {
      cutoff = target - learned[currentStep].earlyCutoff;
    }

    if (target >= currentTempC) {
      // Heating step
      if (rampHeating && currentTempC < cutoff) {
        digitalWrite(esc, HIGH);
        digitalWrite(relay, HIGH);
      } else {
        rampHeating = false;
        digitalWrite(relay, LOW);
        digitalWrite(esc, HIGH);
        if (currentTempC <= target + 1.0f && currentTempC >= target - 1.0f) {
          // Settled — advance
          currentStep++;
          rampHeating = true;
          if (currentStep >= numSteps) {
            rampActive = false;
            stopAllOutputs();
          }
        }
      }
    } else {
      // Cooling step
      digitalWrite(relay, LOW);
      digitalWrite(esc, LOW);
      if (currentTempC <= target + 1.0f) {
        currentStep++;
        rampHeating = true;
        if (currentStep >= numSteps) {
          rampActive = false;
          stopAllOutputs();
        }
      }
    }
  }

  // Handle learn mode
  if (learnModeActive) {
    setpointTemp = tickLearnMode();
  }

  // Draw if needed
  bool needsRedraw = (millis() - lastDrawTime >= DRAW_INTERVAL) ||
                     (fabs(currentTempC - lastTempC) > 1.0) ||
                     (abs(average - lastAverage) > 5) ||
                     (encoder_value != lastEncoderVal) ||
                     (selectedOption != lastSelectedOption) ||
                     (loggingEnabled != lastLogging) ||
                     (readProfileEnabled != lastProfile) ||
                     (learnModeEnabled != lastLearnMode) ||
                     (learnModeActive != lastLearnActive) ||
                     (learnPhase != lastLearnPhase) ||
                     (sdCardPresent != lastSdPresent) ||
                     (rampActive != lastRampActive) ||
                     (currentStep != lastCurrentStep) ||
                     (abs(currentInternalTemp - lastInternalTemp) > 2);

  if (needsRedraw) {
    drawMenu(setpointTemp);
    lastDrawTime = millis();
    lastTempC = currentTempC;
    lastAverage = average;
    lastEncoderVal = encoder_value;
    lastSelectedOption = selectedOption;
    lastLogging = loggingEnabled;
    lastProfile = readProfileEnabled;
    lastLearnMode = learnModeEnabled;
    lastLearnActive = learnModeActive;
    lastLearnPhase = learnPhase;
    lastSdPresent = sdCardPresent;
    lastRampActive = rampActive;
    lastCurrentStep = currentStep;
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

void stopAllOutputs() {
  digitalWrite(esc, LOW);
  digitalWrite(relay, LOW);
}

void freeProfile() {
  free(steps);
  steps = NULL;
  numSteps = 0;
  currentStep = 0;
  startTemp = 0;
}

void freeLearned() {
  free(learned);
  learned = NULL;
  numLearned = 0;
  learnedDataLoaded = false;
}

void resetLearnState() {
  learnModeEnabled = false;
  learnModeActive = false;
  learnPhase = LEARN_IDLE;
  learnPeakTemp = 0;
  learnTargetCrossTime = 0;
  learnRateAtCutoff = 0;
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
          logFile.println("Timestamp,PotValue,EncoderValue,InternalTemp,ThermocoupleTemp,SetpointTemp,SSR_Esc,SSR_Relay,Step,LearnPhase");
          logFile.close();
        } else {
          Serial.println("Error creating log file.");
          sdInitialized = false;
        }

        if (readProfileEnabled) {
          readProfile();
          loadLearnedData();
          if (numSteps > 0 && !rampActive) {
            currentStep = 0;
            startTemp = isnan(currentTempC) ? 25.0f : currentTempC;
            rampActive = true;
            rampHeating = true;
          }
        }
      } else {
        Serial.println("SD card initialization failed.");
        sdInitialized = false;
      }
    } else {
      Serial.println("SD card removed.");
      SD.end();
      SPI1.end();
      sdInitialized = false;
      loggingEnabled = false;
      readProfileEnabled = false;
      rampActive = false;
      freeProfile();
      resetLearnState();
      freeLearned();
      stopAllOutputs();
    }
  }
}

void readProfile() {
  free(steps);
  steps = NULL;
  numSteps = 0;

  File profileFile = SD.open("profile.csv", FILE_READ);
  if (!profileFile) {
    Serial.println("Error opening profile.csv");
    readProfileEnabled = false;
    return;
  }

  // First pass: count valid lines
  int lineCount = 0;
  char buffer[50];
  while (profileFile.available()) {
    size_t index = 0;
    while (profileFile.available() && index < sizeof(buffer) - 1) {
      char ch = profileFile.read();
      if (ch == '\n' || ch == '\r') break;
      buffer[index++] = ch;
    }
    buffer[index] = '\0';
    if (index == 0) continue;
    float temp = atof(buffer);
    if (temp > 0 && !isnan(temp)) lineCount++;
  }

  if (lineCount == 0) {
    Serial.println("No valid steps in profile.csv");
    profileFile.close();
    readProfileEnabled = false;
    return;
  }

  steps = (RampStep*)malloc(lineCount * sizeof(RampStep));
  if (!steps) {
    Serial.println("Failed to allocate memory for profile");
    profileFile.close();
    readProfileEnabled = false;
    return;
  }

  // Second pass: parse
  profileFile.seek(0);
  while (profileFile.available() && numSteps < lineCount) {
    size_t index = 0;
    while (profileFile.available() && index < sizeof(buffer) - 1) {
      char ch = profileFile.read();
      if (ch == '\n' || ch == '\r') break;
      buffer[index++] = ch;
    }
    buffer[index] = '\0';
    if (index == 0) continue;
    float temp = atof(buffer);
    if (temp > 0 && !isnan(temp)) {
      steps[numSteps].targetTemp = temp;
      numSteps++;
    }
  }
  profileFile.close();

  Serial.print("Profile loaded: ");
  Serial.print(numSteps);
  Serial.println(" steps");
}

void loadLearnedData() {
  freeLearned();

  File f = SD.open("learned.csv", FILE_READ);
  if (!f) return;

  // Skip header
  while (f.available() && f.read() != '\n');

  // Count valid lines
  int count = 0;
  long dataStart = f.position();
  char buffer[120];
  while (f.available()) {
    size_t idx = 0;
    while (f.available() && idx < sizeof(buffer) - 1) {
      char ch = f.read();
      if (ch == '\n' || ch == '\r') break;
      buffer[idx++] = ch;
    }
    buffer[idx] = '\0';
    if (idx > 0) count++;
  }

  if (count == 0) { f.close(); return; }

  learned = (LearnedStep*)malloc(count * sizeof(LearnedStep));
  if (!learned) { f.close(); return; }

  f.seek(dataStart);
  while (f.available() && numLearned < count) {
    size_t idx = 0;
    while (f.available() && idx < sizeof(buffer) - 1) {
      char ch = f.read();
      if (ch == '\n' || ch == '\r') break;
      buffer[idx++] = ch;
    }
    buffer[idx] = '\0';
    if (idx == 0) continue;

    LearnedStep ls;
    if (sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f",
        &ls.targetTemp, &ls.prevSettledTemp, &ls.timeToReach,
        &ls.overshoot, &ls.settleTime, &ls.rateOfRise, &ls.earlyCutoff) == 7) {
      learned[numLearned++] = ls;
    }
  }
  f.close();
  learnedDataLoaded = (numLearned > 0);
  Serial.print("Learned data loaded: ");
  Serial.print(numLearned);
  Serial.println(" steps");
}

void saveLearnedData() {
  SD.remove("learned.csv");
  File f = SD.open("learned.csv", FILE_WRITE);
  if (!f) {
    Serial.println("Error saving learned data");
    return;
  }

  f.println("targetTemp,prevSettledTemp,timeToReach,overshoot,settleTime,rateOfRise,earlyCutoff");
  for (int i = 0; i < numLearned; i++) {
    f.print(learned[i].targetTemp, 1);      f.print(",");
    f.print(learned[i].prevSettledTemp, 1);  f.print(",");
    f.print(learned[i].timeToReach, 1);      f.print(",");
    f.print(learned[i].overshoot, 1);        f.print(",");
    f.print(learned[i].settleTime, 1);       f.print(",");
    f.print(learned[i].rateOfRise, 2);       f.print(",");
    f.print(learned[i].earlyCutoff, 1);
    f.println();
  }
  f.close();
  learnedDataLoaded = true;
  Serial.println("Learned data saved.");
}

float tickLearnMode() {
  if (!learnModeActive || currentStep >= numSteps || isnan(currentTempC)) {
    return isnan(currentTempC) ? 0 : currentTempC;
  }

  float target = steps[currentStep].targetTemp;
  unsigned long now = millis();

  // Rate of rise from previous reading
  float rate = 0;
  if (learnPrevTime > 0 && (now - learnPrevTime) > 0) {
    rate = (currentTempC - learnPrevTemp) / ((now - learnPrevTime) / 1000.0f);
  }

  // Check for timeout
  if (learnPhase == LEARN_HEATING && (now - learnStepStartTime) > LEARN_TIMEOUT_MS) {
    Serial.print("Step ");
    Serial.print(currentStep);
    Serial.println(" TIMEOUT — skipping");
    // Record partial data
    learned[currentStep].targetTemp = target;
    learned[currentStep].prevSettledTemp = (currentStep == 0) ? startTemp : steps[currentStep - 1].targetTemp;
    learned[currentStep].timeToReach = -1;
    learned[currentStep].overshoot = 0;
    learned[currentStep].settleTime = 0;
    learned[currentStep].rateOfRise = rate;
    learned[currentStep].earlyCutoff = 0;
    // Advance
    currentStep++;
    if (currentStep >= numSteps) {
      numLearned = numSteps;
      saveLearnedData();
      learnPhase = LEARN_COMPLETE;
      learnModeActive = false;
      stopAllOutputs();
    } else {
      learnStepStartTime = now;
      learnPeakTemp = 0;
      learnTargetCrossTime = 0;
      learnRateAtCutoff = 0;
      float nextTarget = steps[currentStep].targetTemp;
      learnPhase = (nextTarget < currentTempC) ? LEARN_COOLING : LEARN_HEATING;
    }
    learnPrevTemp = currentTempC;
    learnPrevTime = now;
    return target;
  }

  // Determine early cutoff from previous learned data
  float cutoffTemp = target;
  if (currentStep < numLearned &&
      learned[currentStep].targetTemp == target &&
      learned[currentStep].earlyCutoff > 0) {
    cutoffTemp = target - learned[currentStep].earlyCutoff;
  }

  switch (learnPhase) {
    case LEARN_HEATING:
      digitalWrite(esc, HIGH);
      digitalWrite(relay, HIGH);

      if (currentTempC >= cutoffTemp) {
        learnRateAtCutoff = rate;
        learnTargetCrossTime = now;
        learnPeakTemp = currentTempC;
        digitalWrite(relay, LOW);
        learnPhase = LEARN_OVERSHOOT;
        Serial.print("Step ");
        Serial.print(currentStep);
        Serial.print(" cutoff at ");
        Serial.println(currentTempC);
      }
      break;

    case LEARN_OVERSHOOT:
      digitalWrite(esc, HIGH);
      digitalWrite(relay, LOW);

      if (currentTempC > learnPeakTemp) {
        learnPeakTemp = currentTempC;
      }
      // Peak passed when temp drops 0.5C below recorded peak
      if (currentTempC < learnPeakTemp - 0.5f) {
        learnPhase = LEARN_SETTLING;
        Serial.print("Step ");
        Serial.print(currentStep);
        Serial.print(" peak: ");
        Serial.println(learnPeakTemp);
      }
      break;

    case LEARN_SETTLING:
      digitalWrite(esc, HIGH);
      digitalWrite(relay, LOW);

      if (currentTempC <= target + 1.0f) {
        float timeToReach = (learnTargetCrossTime - learnStepStartTime) / 1000.0f;
        float overshoot = learnPeakTemp - target;
        float settleTime = (now - learnTargetCrossTime) / 1000.0f;
        float prevSettled = (currentStep == 0) ? startTemp : steps[currentStep - 1].targetTemp;

        // Early cutoff: 80% of overshoot
        float earlyCut = overshoot * 0.8f;
        if (earlyCut < 0) earlyCut = 0;

        learned[currentStep].targetTemp = target;
        learned[currentStep].prevSettledTemp = prevSettled;
        learned[currentStep].timeToReach = timeToReach;
        learned[currentStep].overshoot = overshoot;
        learned[currentStep].settleTime = settleTime;
        learned[currentStep].rateOfRise = learnRateAtCutoff;
        learned[currentStep].earlyCutoff = earlyCut;

        Serial.print("Step ");
        Serial.print(currentStep);
        Serial.print(" settled. Overshoot=");
        Serial.print(overshoot);
        Serial.print(" earlyCut=");
        Serial.println(earlyCut);

        currentStep++;
        if (currentStep >= numSteps) {
          numLearned = numSteps;
          saveLearnedData();
          learnPhase = LEARN_COMPLETE;
          learnModeActive = false;
          stopAllOutputs();
          Serial.println("Learn mode complete.");
        } else {
          learnStepStartTime = now;
          learnPeakTemp = 0;
          learnTargetCrossTime = 0;
          learnRateAtCutoff = 0;
          float nextTarget = steps[currentStep].targetTemp;
          learnPhase = (nextTarget < currentTempC) ? LEARN_COOLING : LEARN_HEATING;
        }
      }
      break;

    case LEARN_COOLING:
      // Passive cooling — everything off
      stopAllOutputs();

      if (currentTempC <= target + 1.0f) {
        float coolTime = (now - learnStepStartTime) / 1000.0f;
        float prevSettled = (currentStep == 0) ? startTemp : steps[currentStep - 1].targetTemp;

        learned[currentStep].targetTemp = target;
        learned[currentStep].prevSettledTemp = prevSettled;
        learned[currentStep].timeToReach = coolTime;
        learned[currentStep].overshoot = 0;
        learned[currentStep].settleTime = 0;
        learned[currentStep].rateOfRise = rate;
        learned[currentStep].earlyCutoff = 0;

        Serial.print("Step ");
        Serial.print(currentStep);
        Serial.print(" cooled to ");
        Serial.println(currentTempC);

        currentStep++;
        if (currentStep >= numSteps) {
          numLearned = numSteps;
          saveLearnedData();
          learnPhase = LEARN_COMPLETE;
          learnModeActive = false;
          stopAllOutputs();
          Serial.println("Learn mode complete.");
        } else {
          learnStepStartTime = now;
          learnPeakTemp = 0;
          learnTargetCrossTime = 0;
          learnRateAtCutoff = 0;
          float nextTarget = steps[currentStep].targetTemp;
          learnPhase = (nextTarget < currentTempC) ? LEARN_COOLING : LEARN_HEATING;
        }
      }
      break;

    case LEARN_COMPLETE:
    case LEARN_IDLE:
    default:
      stopAllOutputs();
      break;
  }

  learnPrevTemp = currentTempC;
  learnPrevTime = now;
  return target;
}

void drawPaddedStr(int x, int y, const char* str, int padWidth, uint16_t fg, uint16_t bg) {
  char buf[50];
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

void drawMenu(float setpointTemp) {
  tft.startWrite();
  tft.setTextSize(menuTextSize);

  // SD status (y=10)
  if (sdCardPresent != lastSdPresent) {
    drawPaddedStr(11, 13, sdCardPresent ? "SD: Present" : "SD: Not Present", 20, textColour, TFT_BLACK);
  }

  // Logging option (y=35)
  uint16_t logBg   = selectedOption == 0 ? TFT_BLUE : TFT_BLACK;
  uint16_t profBg  = selectedOption == 1 ? TFT_BLUE : TFT_BLACK;
  uint16_t learnBg = selectedOption == 2 ? TFT_BLUE : TFT_BLACK;

  if (selectedOption != lastSelectedOption || loggingEnabled != lastLogging) {
    tft.fillRect(10, 35, 280, 22, logBg);
    drawPaddedStr(11, 37, loggingEnabled ? "Logging: ON" : "Logging: OFF", 20, textColour, logBg);
  }

  // Profile option (y=60)
  if (selectedOption != lastSelectedOption || readProfileEnabled != lastProfile) {
    tft.fillRect(10, 60, 280, 22, profBg);
    drawPaddedStr(11, 62, readProfileEnabled ? "Profile: ON" : "Profile: OFF", 20, textColour, profBg);
  }

  // Learn mode option (y=85)
  if (selectedOption != lastSelectedOption || learnModeEnabled != lastLearnMode) {
    tft.fillRect(10, 85, 280, 22, learnBg);
    drawPaddedStr(11, 87, learnModeEnabled ? "Learn: ON" : "Learn: OFF", 20, textColour, learnBg);
  }

  // Status line (y=112)
  bool statusActive = learnModeActive || rampActive;
  bool lastStatusActive = lastLearnActive || lastRampActive;
  if (statusActive) {
    if (statusActive != lastStatusActive || currentStep != lastCurrentStep ||
        learnPhase != lastLearnPhase || learnModeActive != lastLearnActive) {
      char statusText[50];
      if (learnModeActive && currentStep < numSteps) {
        const char* phaseStr = "IDLE";
        switch (learnPhase) {
          case LEARN_HEATING:   phaseStr = "HEAT";    break;
          case LEARN_OVERSHOOT: phaseStr = "OVRSHT";  break;
          case LEARN_SETTLING:  phaseStr = "SETTLE";  break;
          case LEARN_COOLING:   phaseStr = "COOL";    break;
          case LEARN_COMPLETE:  phaseStr = "DONE";    break;
          default: break;
        }
        snprintf(statusText, sizeof(statusText), "Learn %d/%d: %.0fC [%s]",
                 currentStep + 1, numSteps, steps[currentStep].targetTemp, phaseStr);
      } else if (rampActive && currentStep < numSteps) {
        snprintf(statusText, sizeof(statusText), "Run %d/%d: %.0fC target",
                 currentStep + 1, numSteps, setpointTemp);
      }
      drawPaddedStr(11, 114, statusText, 38, textColour, TFT_BLACK);
    }
  } else if (lastStatusActive) {
    tft.fillRect(10, 112, 460, 22, TFT_BLACK);
  }

  tft.setTextSize(textSize);

  // Pot average (y=140)
  if (abs(average - lastAverage) > 0) {
    itoa(average, analog, 10);
    drawPaddedStr(11, 142, analog, 8, textColour, TFT_BLACK);
  }

  // Encoder (y=172)
  if (encoder_value != lastEncoderVal) {
    itoa(encoder_value, analog, 10);
    drawPaddedStr(11, 174, analog, 8, textColour, TFT_BLACK);
  }

  // Internal temp (y=204)
  if (abs(currentInternalTemp - lastInternalTemp) > 0) {
    itoa(currentInternalTemp, analog, 10);
    drawPaddedStr(11, 206, analog, 8, textColour, TFT_BLACK);
  }

  // TC temp (y=236)
  if (strcmp(tc_str, lastTcStr) != 0) {
    drawPaddedStr(11, 238, tc_str, 8, textColour, TFT_BLACK);
    strcpy(lastTcStr, tc_str);
  }

  tft.endWrite();
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
    int logStep = (rampActive || learnModeActive) ? (currentStep + 1) : 0;
    logFile.print(logStep);
    logFile.print(",");
    logFile.print((int)learnPhase);
    logFile.println();
    logFile.close();
  } else {
    Serial.println("Error opening log file.");
  }
}

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
#define NUM_OPTIONS 4
#define NUM_SETTINGS 6
#define MAX_MEDIAN_WINDOW 128
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

// Settings
float tcSmoothing = 0.3f;
float tcOffset = 0.0f;
int tcMedianWindow = 7;
float smoothedTempC = 0;
bool smoothedSeeded = false;
bool inSettings = false;
int settingsItem = 0;
bool settingsEditing = false;
bool testEsc = false;
bool testRelay = false;
float lastTcSmoothing = -1;
float lastTcOffset = -99;
int lastTcMedianWindow = -1;
int lastSettingsItem = -1;
bool lastSettingsEditing = false;
bool lastInSettings = false;

// Median filter circular buffer
float tcBuffer[MAX_MEDIAN_WINDOW];
int tcBufferIdx = 0;
int tcBufferCount = 0;

// Profile steps — target temperatures with optional dwell
struct RampStep {
  float targetTemp;
  unsigned long dwellMs;  // 0 = no dwell
};
RampStep* steps = NULL;
int numSteps = 0;
int currentStep = 0;
float startTemp = 0;
bool rampActive = false;
bool rampHeating = true;

// Dwell state
bool dwelling = false;
unsigned long dwellStartTime = 0;

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
bool lastDwelling = false;
int lastInternalTemp = 0;
char lastTcStr[10];

// Forward declarations
void readProfile();
void loadLearnedData();
void saveLearnedData();
void loadSettings();
void saveSettings();
float tickLearnMode();
void drawMenu(float setpointTemp);
void drawSettings();
void stopAllOutputs();
void freeProfile();
void freeLearned();
void resetLearnState();

float medianFilter(float newVal) {
  tcBuffer[tcBufferIdx] = newVal;
  tcBufferIdx = (tcBufferIdx + 1) % tcMedianWindow;
  if (tcBufferCount < tcMedianWindow) tcBufferCount++;

  // Copy active samples and sort
  float sorted[MAX_MEDIAN_WINDOW];
  int n = tcBufferCount;
  for (int i = 0; i < n; i++) sorted[i] = tcBuffer[i];
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (sorted[j] < sorted[i]) {
        float tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  return sorted[n / 2];
}

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

  // Seed smoothed temp and median buffer with first valid reading
  float initTemp = thermocouple.readCelsius();
  if (!isnan(initTemp)) {
    float adj = initTemp + tcOffset;
    for (int i = 0; i < tcMedianWindow; i++) tcBuffer[i] = adj;
    tcBufferCount = tcMedianWindow;
    tcBufferIdx = 0;
    smoothedTempC = adj;
    smoothedSeeded = true;
  }

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

  float rawTempC = thermocouple.readCelsius();
  currentInternalTemp = thermocouple.readInternal();
  if (isnan(rawTempC)) {
    uint8_t fault = thermocouple.readError();
    if (fault & MAX31855_FAULT_OPEN) strcpy(tc_str, "OPEN");
    else if (fault & MAX31855_FAULT_SHORT_GND) strcpy(tc_str, "GND");
    else if (fault & MAX31855_FAULT_SHORT_VCC) strcpy(tc_str, "VCC");
    else strcpy(tc_str, "ERR");
    currentTempC = rawTempC;
  } else {
    float adjusted = rawTempC + tcOffset;
    float filtered = medianFilter(adjusted);
    if (!smoothedSeeded) {
      smoothedTempC = filtered;
      smoothedSeeded = true;
    } else {
      smoothedTempC = tcSmoothing * filtered + (1.0f - tcSmoothing) * smoothedTempC;
    }
    currentTempC = smoothedTempC;
    dtostrf(currentTempC, 4, 1, tc_str);
  }

  pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
  int32_t raw = (int32_t)pio_sm_get_blocking(pio, sm);
  int32_t delta = raw - last_encoder_value;
  last_encoder_value = raw;
  encoder_value = raw;

  // Encoder routing: settings sub-menu vs main menu
  if (delta != 0) {
    if (inSettings) {
      if (settingsEditing) {
        // Adjust the selected setting value
        if (settingsItem == 0) {
          tcSmoothing += delta * 0.1f;
          if (tcSmoothing < 0.1f) tcSmoothing = 0.1f;
          if (tcSmoothing > 1.0f) tcSmoothing = 1.0f;
        } else if (settingsItem == 1) {
          tcOffset += delta * 0.1f;
          if (tcOffset < -10.0f) tcOffset = -10.0f;
          if (tcOffset > 10.0f) tcOffset = 10.0f;
        } else if (settingsItem == 2) {
          tcMedianWindow += delta;
          if (tcMedianWindow < 1) tcMedianWindow = 1;
          if (tcMedianWindow > MAX_MEDIAN_WINDOW) tcMedianWindow = MAX_MEDIAN_WINDOW;
          // Reset buffer when window size changes
          tcBufferIdx = 0;
          tcBufferCount = 0;
        }
      } else {
        // Navigate settings items
        settingsItem = ((settingsItem + delta) % NUM_SETTINGS + NUM_SETTINGS) % NUM_SETTINGS;
      }
    } else {
      selectedOption = ((selectedOption + delta) % NUM_OPTIONS + NUM_OPTIONS) % NUM_OPTIONS;
    }
  }

  checkSDCard();

  noInterrupts();
  bool pressed = swPressed;
  swPressed = false;
  interrupts();

  if (pressed) {
    if (inSettings) {
      // Settings sub-menu button handling
      if (settingsItem == 5) {
        // "Back" — exit settings, save, stop test outputs
        inSettings = false;
        settingsEditing = false;
        testEsc = false;
        testRelay = false;
        stopAllOutputs();
        if (sdCardPresent && sdInitialized) saveSettings();
        tft.fillScreen(TFT_BLACK);
        // Force full main menu redraw
        lastSelectedOption = -1;
        lastSdPresent = !sdCardPresent;
        lastLogging = !loggingEnabled;
        lastProfile = !readProfileEnabled;
        lastLearnMode = !learnModeEnabled;
        lastRampActive = !rampActive;
        lastLearnActive = !learnModeActive;
        lastCurrentStep = -1;
        strcpy(lastTcStr, "~");
        lastAverage = average - 1;
        lastInternalTemp = currentInternalTemp - 1;
        lastEncoderVal = encoder_value - 1;
      } else if (settingsItem == 3) {
        // Toggle ESC output
        testEsc = !testEsc;
        digitalWrite(esc, testEsc ? HIGH : LOW);
        lastSettingsItem = -1;  // force redraw
      } else if (settingsItem == 4) {
        // Toggle Relay output
        testRelay = !testRelay;
        digitalWrite(relay, testRelay ? HIGH : LOW);
        lastSettingsItem = -1;  // force redraw
      } else {
        // Toggle edit mode for smoothing, offset, median
        settingsEditing = !settingsEditing;
      }

    } else {
      // Main menu button handling
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
          dwelling = false;
          freeProfile();
          stopAllOutputs();
        }

      } else if (selectedOption == 2) {
        // Toggle learn mode
        learnModeEnabled = !learnModeEnabled;
        Serial.print("Learn: ");
        Serial.println(learnModeEnabled ? "ON" : "OFF");
        if (learnModeEnabled) {
          readProfileEnabled = false;
          rampActive = false;
          if (sdCardPresent) {
            readProfile();
            if (numSteps > 0) {
              loadLearnedData();
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
                learnPhase = (target < learnPrevTemp) ? LEARN_COOLING : LEARN_HEATING;
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

      } else if (selectedOption == 3) {
        // Enter settings
        inSettings = true;
        settingsItem = 0;
        settingsEditing = false;
        tft.fillScreen(TFT_BLACK);
        // Force full settings draw
        lastTcSmoothing = -1;
        lastTcOffset = -99;
        lastSettingsItem = -1;
        lastSettingsEditing = false;
        lastInSettings = false;
      }
    }
  }

  // Handle profile run
  float setpointTemp = isnan(currentTempC) ? 0 : currentTempC;
  if (rampActive && currentStep < numSteps && !isnan(currentTempC)) {
    float target = steps[currentStep].targetTemp;
    setpointTemp = target;

    float cutoff = target;
    if (learnedDataLoaded && currentStep < numLearned &&
        learned[currentStep].targetTemp == target &&
        learned[currentStep].earlyCutoff > 0) {
      cutoff = target - learned[currentStep].earlyCutoff;
    }

    if (target >= currentTempC) {
      if (rampHeating && currentTempC < cutoff) {
        digitalWrite(esc, HIGH);
        digitalWrite(relay, HIGH);
      } else {
        rampHeating = false;
        digitalWrite(relay, LOW);
        digitalWrite(esc, HIGH);
        if (currentTempC <= target + 1.0f && currentTempC >= target - 1.0f) {
          // Temperature reached — handle dwell or advance
          if (steps[currentStep].dwellMs > 0 && !dwelling) {
            dwelling = true;
            dwellStartTime = millis();
          } else if (dwelling && (millis() - dwellStartTime) >= steps[currentStep].dwellMs) {
            dwelling = false;
            currentStep++;
            rampHeating = true;
            if (currentStep >= numSteps) {
              rampActive = false;
              stopAllOutputs();
            }
          } else if (steps[currentStep].dwellMs == 0) {
            currentStep++;
            rampHeating = true;
            if (currentStep >= numSteps) {
              rampActive = false;
              stopAllOutputs();
            }
          }
          // During dwell on a heating step, keep esc HIGH (fan) to maintain temp
        }
      }
    } else {
      digitalWrite(relay, LOW);
      digitalWrite(esc, LOW);
      if (currentTempC <= target + 1.0f) {
        // Temperature reached — handle dwell or advance
        if (steps[currentStep].dwellMs > 0 && !dwelling) {
          dwelling = true;
          dwellStartTime = millis();
        } else if (dwelling && (millis() - dwellStartTime) >= steps[currentStep].dwellMs) {
          dwelling = false;
          currentStep++;
          rampHeating = true;
          if (currentStep >= numSteps) {
            rampActive = false;
            stopAllOutputs();
          }
        } else if (steps[currentStep].dwellMs == 0) {
          currentStep++;
          rampHeating = true;
          if (currentStep >= numSteps) {
            rampActive = false;
            stopAllOutputs();
          }
        }
      }
    }
  }

  // Handle learn mode
  if (learnModeActive) {
    setpointTemp = tickLearnMode();
  }

  // Draw
  if (inSettings) {
    bool settingsChanged = (inSettings != lastInSettings) ||
                           (settingsItem != lastSettingsItem) ||
                           (settingsEditing != lastSettingsEditing) ||
                           (fabs(tcSmoothing - lastTcSmoothing) > 0.01f) ||
                           (fabs(tcOffset - lastTcOffset) > 0.01f) ||
                           (tcMedianWindow != lastTcMedianWindow);
    if (settingsChanged) {
      drawSettings();
      lastInSettings = inSettings;
      lastSettingsItem = settingsItem;
      lastSettingsEditing = settingsEditing;
      lastTcSmoothing = tcSmoothing;
      lastTcOffset = tcOffset;
      lastTcMedianWindow = tcMedianWindow;
    }
  } else {
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
                       (dwelling != lastDwelling) ||
                       (dwelling) ||  // redraw during dwell for countdown
                       (currentStep != lastCurrentStep) ||
                       (abs(currentInternalTemp - lastInternalTemp) > 2) ||
                       (inSettings != lastInSettings);

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
      lastDwelling = dwelling;
      lastCurrentStep = currentStep;
      lastInternalTemp = currentInternalTemp;
      lastInSettings = inSettings;
    }
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

void loadSettings() {
  File f = SD.open("settings.csv", FILE_READ);
  if (!f) return;

  // Skip header
  while (f.available() && f.read() != '\n');

  char buffer[30];
  size_t idx = 0;
  while (f.available() && idx < sizeof(buffer) - 1) {
    char ch = f.read();
    if (ch == '\n' || ch == '\r') break;
    buffer[idx++] = ch;
  }
  buffer[idx] = '\0';
  f.close();

  float s, o;
  int m;
  if (sscanf(buffer, "%f,%f,%d", &s, &o, &m) >= 2) {
    if (s >= 0.1f && s <= 1.0f) tcSmoothing = s;
    if (o >= -10.0f && o <= 10.0f) tcOffset = o;
    if (m >= 1 && m <= MAX_MEDIAN_WINDOW) tcMedianWindow = m;
    Serial.print("Settings loaded: smoothing=");
    Serial.print(tcSmoothing);
    Serial.print(" offset=");
    Serial.print(tcOffset);
    Serial.print(" median=");
    Serial.println(tcMedianWindow);
  }
}

void saveSettings() {
  SD.remove("settings.csv");
  File f = SD.open("settings.csv", FILE_WRITE);
  if (!f) {
    Serial.println("Error saving settings");
    return;
  }
  f.println("smoothing,offset,median");
  f.print(tcSmoothing, 1);
  f.print(",");
  f.print(tcOffset, 1);
  f.print(",");
  f.println(tcMedianWindow);
  f.close();
  Serial.println("Settings saved.");
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

        loadSettings();

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
          logFile.println("Timestamp,PotValue,EncoderValue,InternalTemp,ThermocoupleTemp,SetpointTemp,SSR_Esc,SSR_Relay,Step,LearnPhase,Dwelling");
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
      dwelling = false;
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
      // Parse optional dwell_seconds after comma
      char* comma = strchr(buffer, ',');
      if (comma) {
        unsigned long dwellSec = strtoul(comma + 1, NULL, 10);
        steps[numSteps].dwellMs = dwellSec * 1000UL;
      } else {
        steps[numSteps].dwellMs = 0;
      }
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

  while (f.available() && f.read() != '\n');

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

  float rate = 0;
  if (learnPrevTime > 0 && (now - learnPrevTime) > 0) {
    rate = (currentTempC - learnPrevTemp) / ((now - learnPrevTime) / 1000.0f);
  }

  if (learnPhase == LEARN_HEATING && (now - learnStepStartTime) > LEARN_TIMEOUT_MS) {
    Serial.print("Step ");
    Serial.print(currentStep);
    Serial.println(" TIMEOUT");
    learned[currentStep].targetTemp = target;
    learned[currentStep].prevSettledTemp = (currentStep == 0) ? startTemp : steps[currentStep - 1].targetTemp;
    learned[currentStep].timeToReach = -1;
    learned[currentStep].overshoot = 0;
    learned[currentStep].settleTime = 0;
    learned[currentStep].rateOfRise = rate;
    learned[currentStep].earlyCutoff = 0;
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

void drawSettings() {
  tft.startWrite();
  tft.setTextSize(menuTextSize);

  // Title
  drawPaddedStr(11, 13, "SETTINGS", 20, TFT_YELLOW, TFT_BLACK);

  // Smoothing (y=45)
  uint16_t smBg = settingsItem == 0 ? TFT_BLUE : TFT_BLACK;
  uint16_t smFg = (settingsItem == 0 && settingsEditing) ? TFT_GREEN : textColour;
  tft.fillRect(10, 45, 350, 22, smBg);
  char smText[30];
  snprintf(smText, sizeof(smText), "Smoothing: %.1f", tcSmoothing);
  drawPaddedStr(11, 47, smText, 25, smFg, smBg);

  // Offset (y=75)
  uint16_t ofBg = settingsItem == 1 ? TFT_BLUE : TFT_BLACK;
  uint16_t ofFg = (settingsItem == 1 && settingsEditing) ? TFT_GREEN : textColour;
  tft.fillRect(10, 75, 350, 22, ofBg);
  char ofText[30];
  snprintf(ofText, sizeof(ofText), "Offset: %+.1fC", tcOffset);
  drawPaddedStr(11, 77, ofText, 25, ofFg, ofBg);

  // Median window (y=105)
  uint16_t mdBg = settingsItem == 2 ? TFT_BLUE : TFT_BLACK;
  uint16_t mdFg = (settingsItem == 2 && settingsEditing) ? TFT_GREEN : textColour;
  tft.fillRect(10, 105, 350, 22, mdBg);
  char mdText[30];
  snprintf(mdText, sizeof(mdText), "Median: %d samples", tcMedianWindow);
  drawPaddedStr(11, 107, mdText, 25, mdFg, mdBg);

  // ESC output (y=135)
  uint16_t escBg = settingsItem == 3 ? TFT_BLUE : TFT_BLACK;
  tft.fillRect(10, 135, 350, 22, escBg);
  uint16_t escFg = testEsc ? TFT_GREEN : textColour;
  drawPaddedStr(11, 137, testEsc ? "ESC: ON" : "ESC: OFF", 25, escFg, escBg);

  // Relay output (y=165)
  uint16_t relBg = settingsItem == 4 ? TFT_BLUE : TFT_BLACK;
  tft.fillRect(10, 165, 350, 22, relBg);
  uint16_t relFg = testRelay ? TFT_GREEN : textColour;
  drawPaddedStr(11, 167, testRelay ? "Relay: ON" : "Relay: OFF", 25, relFg, relBg);

  // Back (y=195)
  uint16_t bkBg = settingsItem == 5 ? TFT_BLUE : TFT_BLACK;
  tft.fillRect(10, 195, 350, 22, bkBg);
  drawPaddedStr(11, 197, "< Back", 25, textColour, bkBg);

  // Hint
  if (settingsEditing) {
    drawPaddedStr(11, 235, "Rotate to adjust, press to confirm", 38, TFT_DARKGREY, TFT_BLACK);
  } else {
    drawPaddedStr(11, 235, "Press to edit, rotate to navigate  ", 38, TFT_DARKGREY, TFT_BLACK);
  }

  tft.setTextSize(textSize);

  // Still show TC temp at bottom
  drawPaddedStr(11, 270, tc_str, 8, textColour, TFT_BLACK);

  tft.endWrite();
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
  uint16_t settBg  = selectedOption == 3 ? TFT_BLUE : TFT_BLACK;

  if (selectedOption != lastSelectedOption || loggingEnabled != lastLogging) {
    tft.fillRect(10, 35, 280, 22, logBg);
    drawPaddedStr(11, 37, loggingEnabled ? "Logging: ON" : "Logging: OFF", 20, textColour, logBg);
  }

  // Profile option (y=57)
  if (selectedOption != lastSelectedOption || readProfileEnabled != lastProfile) {
    tft.fillRect(10, 57, 280, 22, profBg);
    drawPaddedStr(11, 59, readProfileEnabled ? "Profile: ON" : "Profile: OFF", 20, textColour, profBg);
  }

  // Learn mode option (y=79)
  if (selectedOption != lastSelectedOption || learnModeEnabled != lastLearnMode) {
    tft.fillRect(10, 79, 280, 22, learnBg);
    drawPaddedStr(11, 81, learnModeEnabled ? "Learn: ON" : "Learn: OFF", 20, textColour, learnBg);
  }

  // Settings option (y=101)
  if (selectedOption != lastSelectedOption) {
    tft.fillRect(10, 101, 280, 22, settBg);
    drawPaddedStr(11, 103, "Settings", 20, textColour, settBg);
  }

  // Status line (y=128)
  bool statusActive = learnModeActive || rampActive;
  bool lastStatusActive = lastLearnActive || lastRampActive;
  if (statusActive) {
    if (statusActive != lastStatusActive || currentStep != lastCurrentStep ||
        learnPhase != lastLearnPhase || learnModeActive != lastLearnActive ||
        dwelling != lastDwelling || dwelling) {
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
        if (dwelling) {
          unsigned long elapsed = millis() - dwellStartTime;
          unsigned long remaining = 0;
          if (elapsed < steps[currentStep].dwellMs)
            remaining = (steps[currentStep].dwellMs - elapsed) / 1000UL;
          unsigned long mins = remaining / 60;
          unsigned long secs = remaining % 60;
          snprintf(statusText, sizeof(statusText), "Run %d/%d: %.0fC DWELL %lu:%02lu",
                   currentStep + 1, numSteps, setpointTemp, mins, secs);
        } else {
          snprintf(statusText, sizeof(statusText), "Run %d/%d: %.0fC target",
                   currentStep + 1, numSteps, setpointTemp);
        }
      }
      drawPaddedStr(11, 130, statusText, 38, textColour, TFT_BLACK);
    }
  } else if (lastStatusActive) {
    tft.fillRect(10, 128, 460, 22, TFT_BLACK);
  }

  tft.setTextSize(textSize);

  // Pot average (y=155)
  if (abs(average - lastAverage) > 0) {
    itoa(average, analog, 10);
    drawPaddedStr(11, 157, analog, 8, textColour, TFT_BLACK);
  }

  // Encoder (y=185)
  if (encoder_value != lastEncoderVal) {
    itoa(encoder_value, analog, 10);
    drawPaddedStr(11, 187, analog, 8, textColour, TFT_BLACK);
  }

  // Internal temp (y=215)
  if (abs(currentInternalTemp - lastInternalTemp) > 0) {
    itoa(currentInternalTemp, analog, 10);
    drawPaddedStr(11, 217, analog, 8, textColour, TFT_BLACK);
  }

  // TC temp (y=245)
  if (strcmp(tc_str, lastTcStr) != 0) {
    drawPaddedStr(11, 247, tc_str, 8, textColour, TFT_BLACK);
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
    logFile.print(",");
    logFile.print(dwelling ? 1 : 0);
    logFile.println();
    logFile.close();
  } else {
    Serial.println("Error opening log file.");
  }
}

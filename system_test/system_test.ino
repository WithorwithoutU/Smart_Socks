#include <Arduino.h>
#include <math.h>
#include <ArduinoBLE.h>

// ==================== Parameters ====================

// Sampling interval for flex sensor (2 seconds)
const unsigned long sampleInterval = 2000; // in milliseconds

// State determination: use a window of 5 consecutive samples (10 seconds total)
const int stateWindowSize = 5;
const float stableThreshold = 1.0; // if the difference between max and min in a window is less than this, readings are considered nearly identical

// Long-term trend detection parameters (for swelling)
// For testing we use a 15-second block; in production this may be extended (e.g., 1-2 minutes)
const unsigned long longTermInterval = 15000; // in milliseconds
const float longTermAlpha = 0.01;             // very slow filter for long-term trend

// Threshold for trend slope (in ADC units per second); if slope is lower (more negative) than this, trigger alarm
const float trendSlopeThreshold = -0.005;  // adjust as needed

// EMA filter for short-term (flex sensor) data
const float emaAlpha = 0.1; // smoothing factor for each sample

// Trend detection window size (number of long-term filtered samples)
const int trendWindowSize = 4;

// ==================== Global Variables ====================

// State machine: MOVING state or STATIONARY state
enum State { MOVING, STATIONARY };
State currentState = MOVING;

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long lastBLETime = 0; // for BLE transmission

// Variables for EMA filtered flex sensor reading
float emaValue = 0;
bool emaFirst = true;

// Buffers for state determination
float movingWindow[stateWindowSize];
int movingCount = 0;
float stationaryWindow[stateWindowSize];
int stationaryCount = 0;

// Long-term trend detection variables
float longTermFiltered = 0;
bool longTermFirst = true;
unsigned long lastLongTermUpdateTime = 0;
float previousBlockAvg = -1;
int consecutiveDecreases = 0;

// Trend window for slope calculation
float trendWindow[trendWindowSize];
int trendCount = 0;

// Alarm flag
bool alarmTriggered = false;

// ==================== BLE Setup ====================

BLEService sensorService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Custom service UUID
BLEStringCharacteristic stateCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);
BLEStringCharacteristic alarmCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  
  delay(1000);//wait for 1 sec
  Serial.println("Starting integrated flex sensor monitoring (without IMU) with BLE transmission...");

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  BLE.setLocalName("FlexSensorMonitor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(stateCharacteristic);
  sensorService.addCharacteristic(alarmCharacteristic);
  BLE.addService(sensorService);
  stateCharacteristic.writeValue("MOVING");
  alarmCharacteristic.writeValue("NORMAL");
  BLE.advertise();
  Serial.println("BLE device is now advertising");
}

// ==================== Main Loop ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Sample the flex sensor every sampleInterval (2 seconds)
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
    // Read the raw ADC value from the flex sensor (assumed connected to A4)
    int rawValue = analogRead(A4);
    
    // ----------------- Short-term EMA Filtering -----------------
    if (emaFirst) {
      emaValue = rawValue;
      emaFirst = false;
    } else {
      emaValue = emaAlpha * rawValue + (1 - emaAlpha) * emaValue;
    }
    float currentSample = emaValue;
    
    // ----------------- Long-term EMA Filtering -----------------
    if (longTermFirst) {
      longTermFiltered = currentSample;
      longTermFirst = false;
    } else {
      longTermFiltered = longTermAlpha * currentSample + (1 - longTermAlpha) * longTermFiltered;
    }
    
    // ------------------ State Machine ------------------
    if (currentState == MOVING) {
      // Record current sample into moving window
      movingWindow[movingCount] = currentSample;
      movingCount++;
      Serial.print("MOVING sample: ");
      Serial.println(currentSample);
      
      // When we have 5 consecutive samples, check if nearly identical
      if (movingCount >= stateWindowSize) {
        float minVal = movingWindow[0];
        float maxVal = movingWindow[0];
        for (int i = 1; i < stateWindowSize; i++) {
          if (movingWindow[i] < minVal) minVal = movingWindow[i];
          if (movingWindow[i] > maxVal) maxVal = movingWindow[i];
        }
        if ((maxVal - minVal) < stableThreshold) {
          // 5 similar readings => subject is stationary
          currentState = STATIONARY;
          Serial.println("Transition to STATIONARY state: Subject is stationary.");
          movingCount = 0; // reset moving window
          
          // Initialize long-term trend variables and trend window
          previousBlockAvg = longTermFiltered;
          lastLongTermUpdateTime = currentTime;
          trendCount = 0;
          alarmTriggered = false;
          stationaryCount = 0;
        } else {
          // Not stable: shift moving window
          for (int i = 1; i < stateWindowSize; i++) {
            movingWindow[i - 1] = movingWindow[i];
          }
          movingCount = stateWindowSize - 1;
        }
      }
      
    } else { // STATIONARY state
      // Record current sample into stationary window
      stationaryWindow[stationaryCount] = currentSample;
      stationaryCount++;
      Serial.print("STATIONARY sample: ");
      Serial.println(currentSample);
      
      // Every 5 samples (10 seconds), check if movement resumed
      if (stationaryCount >= stateWindowSize) {
        int distinctCount = 0;
        float distinct[stateWindowSize];
        int dcount = 0;
        for (int i = 0; i < stateWindowSize; i++) {
          bool found = false;
          for (int j = 0; j < dcount; j++) {
            if (fabs(stationaryWindow[i] - distinct[j]) < stableThreshold) {
              found = true;
              break;
            }
          }
          if (!found) {
            distinct[dcount] = stationaryWindow[i];
            dcount++;
          }
        }
        distinctCount = dcount;
        if (distinctCount >= 3) {
          currentState = MOVING;
          Serial.println("Movement detected in stationary state: Transition to MOVING state.");
          stationaryCount = 0;
          movingCount = 0;
          trendCount = 0;
          alarmTriggered = false;
        } else {
          for (int i = 1; i < stateWindowSize; i++) {
            stationaryWindow[i - 1] = stationaryWindow[i];
          }
          stationaryCount = stateWindowSize - 1;
        }
      }
      
      // ----------------- Long-term Trend Detection -----------------
      if (currentTime - lastLongTermUpdateTime >= longTermInterval) {
        lastLongTermUpdateTime = currentTime;
        // Add current longTermFiltered value to trend window
        trendWindow[trendCount] = longTermFiltered;
        trendCount++;
        Serial.print("Long-term filtered value: ");
        Serial.println(longTermFiltered);
        if (trendCount >= trendWindowSize) {
          // Calculate trend slope (assuming constant interval between trend samples)
          float T = longTermInterval / 1000.0; // seconds between trend samples
          float slope = (trendWindow[trendWindowSize - 1] - trendWindow[0]) / ((trendWindowSize - 1) * T);
          Serial.print("Trend slope: ");
          Serial.println(slope);
          if (slope < trendSlopeThreshold) {
            alarmTriggered = true;
            Serial.println("ALARM: Swelling detected based on trend!");
          } else {
            alarmTriggered = false;
          }
          trendCount = 0;
        }
      }
    }
    
    // ------------------ CSV Output to Serial ------------------
    int totalSamples = movingCount + stationaryCount;
    if (totalSamples >= stateWindowSize) {
      float sumADC = 0;
      for (int i = 0; i < movingCount; i++) {
        sumADC += movingWindow[i];
      }
      for (int i = 0; i < stationaryCount; i++) {
        sumADC += stationaryWindow[i];
      }
      float averageADC = sumADC / totalSamples;
      
      // CSV format: averageADC,STATE,longTermFiltered,ALARM/NORMAL
      Serial.print(averageADC);
      Serial.print(",");
      Serial.print((currentState == MOVING) ? "MOVING" : "STATIONARY");
      Serial.print(",");
      Serial.print(longTermFiltered);
      Serial.print(",");
      Serial.println(alarmTriggered ? "ALARM" : "NORMAL");
      
      movingCount = 0;
      stationaryCount = 0;
    }
  }
  
  // ------------------ BLE Transmission ------------------
  if (currentTime - lastBLETime >= 1000) {
    lastBLETime = currentTime;
    String stateStr = (currentState == MOVING) ? "MOVING" : "STATIONARY";
    String alarmStr = alarmTriggered ? "ALARM" : "NORMAL";
    stateCharacteristic.writeValue(stateStr);
    alarmCharacteristic.writeValue(alarmStr);
  }
  
  BLE.poll();
  
  delay(10); // short delay to yield CPU time for USB tasks
}

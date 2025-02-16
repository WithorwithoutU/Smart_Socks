#include <Arduino.h>
#include <math.h>

// ==================== Parameters ====================

// Sampling interval for flex sensor (2 seconds)
const unsigned long sampleInterval = 2000; // in milliseconds

// State determination: use a window of 5 consecutive samples (10 seconds total)
const int stateWindowSize = 5;
const float stableThreshold = 1.0; // if the difference between max and min in a window is less than this, readings are considered identical

// Long-term trend detection parameters (for swelling)
// For testing we use a 15-second block; in production this may be extended (e.g., 1-2 minutes)
const unsigned long longTermInterval = 15000; // in milliseconds
const float longTermAlpha = 0.01;             // very slow filter for long-term trend
const float longTermDecreaseThreshold = 1.0;  // minimum decrease (in ADC units) required to count as a decrease

// EMA filter for short-term (flex sensor) data
const float emaAlpha = 0.1; // smoothing factor for each sample

// ==================== Global Variables ====================

// State machine: MOVING state or STATIONARY state
enum State { MOVING, STATIONARY };
State currentState = MOVING;

// Timing variables
unsigned long lastSampleTime = 0;

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

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for USB Serial connection
  }
  Serial.println("Starting integrated flex sensor monitoring (without IMU)...");
}

// ==================== Main Loop ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Sample the flex sensor every sampleInterval (2 seconds)
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
    // Read the raw ADC value from the flex sensor (assumed connected to A4)
    int rawValue = analogRead(A4);
    
    // Update the short-term EMA filter for the flex sensor
    if (emaFirst) {
      emaValue = rawValue;
      emaFirst = false;
    } else {
      emaValue = emaAlpha * rawValue + (1 - emaAlpha) * emaValue;
    }
    float currentSample = emaValue;
    
    // Update long-term filter (for swelling detection) with a very low alpha
    if (longTermFirst) {
      longTermFiltered = currentSample;
      longTermFirst = false;
    } else {
      longTermFiltered = longTermAlpha * currentSample + (1 - longTermAlpha) * longTermFiltered;
    }
    
    // ------------------ State Machine ------------------
    if (currentState == MOVING) {
      // In MOVING state, record the current sample into the moving window
      movingWindow[movingCount] = currentSample;
      movingCount++;
      Serial.print("MOVING sample: ");
      Serial.println(currentSample);
      
      // When we have 5 consecutive samples, check if they are nearly identical
      if (movingCount >= stateWindowSize) {
        float minVal = movingWindow[0];
        float maxVal = movingWindow[0];
        for (int i = 1; i < stateWindowSize; i++) {
          if (movingWindow[i] < minVal) minVal = movingWindow[i];
          if (movingWindow[i] > maxVal) maxVal = movingWindow[i];
        }
        if ((maxVal - minVal) < stableThreshold) {
          // 5 consecutive similar readings => subject is stationary
          currentState = STATIONARY;
          Serial.println("Transition to STATIONARY state: Subject is stationary.");
          movingCount = 0; // reset moving window
          
          // Initialize long-term trend variables
          previousBlockAvg = longTermFiltered;
          lastLongTermUpdateTime = currentTime;
          consecutiveDecreases = 0;
          stationaryCount = 0;
        } else {
          // Not stable: shift the moving window to drop the oldest sample
          for (int i = 1; i < stateWindowSize; i++) {
            movingWindow[i - 1] = movingWindow[i];
          }
          movingCount = stateWindowSize - 1;
        }
      }
      
    } else { // STATIONARY state
      // In STATIONARY state, record the current sample into the stationary window
      stationaryWindow[stationaryCount] = currentSample;
      stationaryCount++;
      Serial.print("STATIONARY sample: ");
      Serial.println(currentSample);
      
      // Check stationary window every 5 samples (i.e. 10 seconds)
      if (stationaryCount >= stateWindowSize) {
        // Count distinct values in the stationary window
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
        // If there are at least 3 distinct values among 5 samples, consider that movement has resumed
        if (distinctCount >= 3) {
          currentState = MOVING;
          Serial.println("Movement detected in stationary state: Transition to MOVING state.");
          stationaryCount = 0;
          movingCount = 0;
        } else {
          // Otherwise, shift the stationary window for next samples
          for (int i = 1; i < stateWindowSize; i++) {
            stationaryWindow[i - 1] = stationaryWindow[i];
          }
          stationaryCount = stateWindowSize - 1;
        }
      }
      
      // ------------------ Long-term Trend Detection ------------------
      // Every longTermInterval (15 seconds), compare the long-term filtered value
      if (currentTime - lastLongTermUpdateTime >= longTermInterval) {
        lastLongTermUpdateTime = currentTime;
        Serial.print("Long-term filtered value: ");
        Serial.println(longTermFiltered);
        if (previousBlockAvg >= 0) {
          if (longTermFiltered < previousBlockAvg - longTermDecreaseThreshold) {
            consecutiveDecreases++;
            Serial.print("Long-term decrease detected. Consecutive decreases: ");
            Serial.println(consecutiveDecreases);
            if (consecutiveDecreases >= 2) {
              Serial.println("ALARM: Swelling detected!");
            }
          } else {
            consecutiveDecreases = 0;
          }
        }
        previousBlockAvg = longTermFiltered;
      }
    }
  }
  
  delay(10); // short delay to yield CPU time for USB tasks
}

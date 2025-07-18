/*

MIT License

Copyright (c) 2025 Francis Mike John Camogao

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
#include <MS5611.h>
#include "esp_system.h"
#include <SimpleKalmanFilter.h> 
    SimpleKalmanFilter* kaltPtr = nullptr;    // Simple Kalman Filter for altitudeFiltered smoothing
                                              // Invoking and assigning it as nullpointer first incases that if it ever allocates memory 
                                              // or does non-trivial work, we would not run out of stack or flash (initial).
                                              
// Import global variables
#include <GlobalVariables.h>

static constexpr int OUTPUT_PINS[] = { PIN_GREEN_LED, PIN_RED_LED, PIN_BUZZER };    // Dictionary for LED and Buzzer pins

/// PIN ASSIGNMENTS FOR DEPLOYMENT CHARGES
static constexpr int      DEPLOYMENT_PINS[] = { 13, 27, 32, 33 };                                                                                     // Array of deployment pins for charges
static constexpr size_t   NUM_DEPLOY_PINS   = (sizeof(DEPLOYMENT_PINS) + sizeof(OUTPUT_PINS))/(sizeof(DEPLOYMENT_PINS[0]) + sizeof(OUTPUT_PINS[0]));  // Size of deployment and output pins for the loop

/// SPIFFS Filename 
static const char * const SPIFFS_FILENAME = "/data.txt";

/// Holds one timestamped “snapshot” from the MS5611.
struct DataPoint {
  char           message[64];
  float          temperature;
  long           pressure;
  float          altitudeFiltered;
  unsigned long  timestamp;  // in milliseconds from start()
};

/// State‐machine enumeration: 
enum class SystemState : uint8_t {
  BUFFERING,          // collecting into buffer, no saves yet
  GREEN_SAVING,       // just crossed 3 m: flushed buffer + saved current, waiting for RED threshold
  RED_SAVING,         // altitudeFiltered >= 5 m, continuously save every loop
  RESET_COUNTDOWN,    // altitudeFiltered <= 2 m: keep saving for three seconds
  STOPPED             // done saving; red LED stays on
};

MS5611           ms5611;                                          // Assign the class/struct MS5611 to variable name ms5611.

DataPoint        bufferArray[BUFFER_SIZE];                        // Dictate the bufferArray to set its size to BUFFER_SIZE, that is 10 indeces.
int              bufferIndex        = 0;                          // Start from index 0 from the buffer allocated.
bool             bufferIsFull       = false;                      // Set the buffer conditional to false.

float            referencePressure  = 0.0F;                       // Initially set the value for the reference pressure to 0.
unsigned long    redCountdownStart  = 0UL;                        // Start the countdown of the reset timer to 0.

SystemState      currentState       = SystemState::BUFFERING;     // Set the state of the system to always start from BUFFERING (which is the initial)


// INITIALIZE THE FUNCTIONS
void initSPIFFS();
void saveDataPoint(const DataPoint &pt);
void flushBufferToSPIFFS();
void addToBuffer(const DataPoint &pt);

// On every reboot, read and log the hardware reset cause. That tells you if the last run ended due to a watchdog, brown-out, panic, or external reset.
void reportResetReason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("Last reset reason: %d\n", reason);
    // Optionally map codes to human-readable strings:
    // ESP_RST_POWERON, ESP_RST_BROWNOUT, ESP_RST_WDT, ESP_RST_PANIC, etc.
}

void setup() {
  Serial.begin(115200);
  delay(100);  
  reportResetReason();
  kaltPtr = new SimpleKalmanFilter(0.5, 0.5, 0.138);
  initSPIFFS();
  delay(100); 

  // Initialize OUTPUT pins
  // Set all OUTPUT and DEPLOYMENT pins to OUTPUT and OFF initially
  for (size_t i = 0; i < NUM_DEPLOY_PINS; ++i) {
      pinMode(DEPLOYMENT_PINS[i], OUTPUT);  digitalWrite(DEPLOYMENT_PINS[i], 0);
      pinMode(OUTPUT_PINS[i],     OUTPUT);  digitalWrite(OUTPUT_PINS[i],     0);
  }

  // Initialize the MS5611 sensor
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  if (!ms5611.begin(MS5611_HIGH_RES)) {
    Serial.println("MS5611 initialization failed!");
    while (true) {
      errorBlink();
    }
  }

  delay(1000);

  referencePressure = ms5611.readPressure();  // baseline pressure
}

void loop() {
  static unsigned long  lastLoopTime   = 0UL;
  unsigned long         now            = millis();

  // Enforce a fixed 300 ms loop interval (non‐blocking)
  // This is almost the same with the delay(300) syntax, 
  // however, does not interfere with the program response rate
  // LOOP_INTERVAL_MS == 300ms.
  if (now - lastLoopTime < LOOP_INTERVAL_MS) {
    return;
  }
  lastLoopTime = now;

  // Read MS5611 sensor:
  float temperature         = ms5611.readTemperature();
  long  pressure            = ms5611.readPressure();
  float altitude            = ms5611.getAltitude(pressure, referencePressure);
  float altitudeFiltered    = kaltPtr ? kaltPtr->updateEstimate(altitude) : altitude; // Each time we call a method on that pointer, we must ensure it isn’t null, 
                                                                                      // otherwise we'll dereference a null pointer and crash. 

  // Build the current data point and append to variable currentPt
  DataPoint currentPt { 
    temperature, 
    pressure, 
    altitudeFiltered, 
    now 
  };

  // ALWAYS buffer the data to the memory but temporarily
  addToBuffer(currentPt);

  // State‐machine: handle transitions AND actions in for each state.
  // Using a switch–case construct for the flight-state machine offers several advantages over a cascade of "if" statement
  // Hence, utilize and perfect for this scenario. Memory-wise, switch is not always expecting any return than "if" construct therefore is efficient.
  switch (currentState) {
    case SystemState::BUFFERING: // collecting the first ~3 seconds into buffer; no saves yet
      if (altitudeFiltered >= ALT_THRESHOLD_LAUNCH) {
        Serial.println("--> ALT >= 3 m: GREEN LED ON (transition BUFFERING → GREEN_SAVING)");

        digitalWrite(PIN_GREEN_LED, HIGH);            // Turn ON green LED
        flushBufferToSPIFFS();                        // Flush everything we have buffered so far to SPIFFS
        saveDataPoint(currentPt);                     // Also append the current data point (since it’s ≥ 3 m)
        currentState = SystemState::GREEN_SAVING;     // Move to GREEN_SAVING state (no further automatic saving until 5 m)
      }
      break;

    case SystemState::GREEN_SAVING: // we have already turned green ON and saved the buffer & current once
      if (altitudeFiltered >= ALT_THRESHOLD_APOGEE) {
        Serial.println("--> ALT >= 5 m: GREEN→OFF, RED→ON (transition GREEN_SAVING → RED_SAVING)");
        
        digitalWrite(PIN_GREEN_LED, HIGH);  digitalWrite(PIN_RED_LED,   HIGH);    // Transition to RED: turn OFF green, turn ON red
        flushBufferToSPIFFS();                                                    // Flush any leftover buffer entries (should be empty, but safe)
        saveDataPoint(currentPt);                                                 // Save the current sample immediately (first red‐region data)
        currentState = SystemState::RED_SAVING;                                   // Enter RED_SAVING state: will save every loop iteration while altitudeFiltered ≥ 5 m
      }
      break;

    case SystemState::RED_SAVING: // continuously save while altitudeFiltered remains >= 5 m
      if (altitudeFiltered >= ALT_THRESHOLD_APOGEE) {
        saveDataPoint(currentPt); // Append new data to the memory each loop

        // DEPLOY CHARGE PINS ALL AT ONCE
        for (size_t ch = 0; ch < NUM_DEPLOY_PINS; ++ch) {
            digitalWrite(DEPLOYMENT_PINS[ch], HIGH);
        }

        // TURN OFF ALL DEPLOYMENT PORTS
        for (size_t ch = 0; ch < NUM_DEPLOY_PINS; ++ch) {
            digitalWrite(DEPLOYMENT_PINS[ch], LOW);
        }

      }
      else if (altitudeFiltered <= ALT_RESET_THRESHOLD) { // If altitudeFiltered drops to <= 2 m, begin the 3 s “reset countdown”
        Serial.println("--> ALT <= 2 m: GREEN→ON, RED remains ON (entering RESET_COUNTDOWN)");

        digitalWrite(PIN_GREEN_LED, LOW); digitalWrite(PIN_RED_LED,   HIGH);    // Immediately turn OFF green (if somehow still ON) but keep red ON
        redCountdownStart = now;                                                // Mark the start of the 3 s countdown
        flushBufferToSPIFFS();  saveDataPoint(currentPt);                       // Flush any remaining buffer (if any) & save current sample once more
        currentState = SystemState::RESET_COUNTDOWN;                            // Enter the Reset State
      }
      break;

    case SystemState::RESET_COUNTDOWN: // After altitudeFiltered <= 2 m, we save data for exactly ALT_RESET_THRESHOLD (3 000 ms)
      if (now - redCountdownStart < ALT_RESET_THRESHOLD) { 
        saveDataPoint(currentPt); // Keep saving at the same 300 ms cadence
      }
      else { // 3 seconds have passed → STOP saving from now on, but keep RED LED ON
        Serial.println("--> 3 s after ALT <= 2 m: STOP saving (state → STOPPED). RED LED remains ON.");
        currentState = SystemState::STOPPED; // Enter the Stop State
      }
      break;

    case SystemState::STOPPED:
      // In this final state, we do nothing except keep the red LED ON
      // No further data writing to SPIFFS.
      freeKalmanFilter(); // Free the Kalman filter since we won’t need it any more
      break;

    // This helps catch logic bugs in flight and catches any unexpected enum value
    default: { 
      uint8_t stateVal = static_cast<uint8_t>(currentState); // Cast state to integer

      // Format error into a C-string
      char errorBuf[64];
      int len = snprintf(
          errorBuf,
          sizeof(errorBuf),
          "Something went wrong during state %u",
          stateVal
      );

      // Ensure termination
      if (len < 0 || len >= static_cast<int>(sizeof(errorBuf))) {
          // Truncate safely
          errorBuf[sizeof(errorBuf) - 1] = '\0';
      }

      // Populate a DataPoint instance
      DataPoint dp;

      // Copy the formatted message
      strncpy(dp.message, errorBuf, sizeof(dp.message));
      dp.message[sizeof(dp.message) - 1] = '\0';

      // Log and save
      Serial.printf("ERROR: unexpected state %u\n", stateVal);
      saveDataPoint(dp);

      // DEPLOY CHARGE PINS ALL AT ONCE
      for (size_t ch = 0; ch < NUM_DEPLOY_PINS; ++ch) {
          digitalWrite(DEPLOYMENT_PINS[ch], HIGH);
      }

      // TURN OFF ALL DEPLOYMENT PORTS
      for (size_t ch = 0; ch < NUM_DEPLOY_PINS; ++ch) {
          digitalWrite(DEPLOYMENT_PINS[ch], LOW);
      }

      // Transition to STOPPED
      currentState = SystemState::STOPPED;
      break;
  }


}

  // (Optional) Any debug printout for real‐time monitoring:
  Serial.printf(
    "Temp: %.2f °C | Pressure: %ld Pa | Alt: %.2f m | State: %d | Time: %lu ms\n",
    temperature, pressure, altitudeFiltered, static_cast<int>(currentState), now
  );
  

  // Delay enforced by the loop‐interval logic above, so no additional delay() here.
  // aaaaaaaaaah go crazy
}

/// Clean up function of the pointer of the Kalman Filter.
// Whenever we allocate with "new", we reserve heap memory. If the application ever “tears down” that object—say, 
// we reinitialize or shut down the system—we should free that memory with delete to avoid exhausting the heap
static void freeKalmanFilter()
{
    if (kaltPtr)
    {
        delete kaltPtr;
        kaltPtr = nullptr;
    }
}

/// Blink the red LED at a fixed period to indicate a critical error.
/// This function does not return.
static void errorBlink() {
    // Blink forever: 500 ms on, 500 ms off
    for (;;)
    {
        digitalWrite(PIN_RED_LED, HIGH);
        delay(500);

        digitalWrite(PIN_RED_LED, LOW);
        delay(500);
    }
}

#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

/// MS5611 / ALTITUDE THRESHOLDS
static constexpr float            ALT_THRESHOLD_LAUNCH         = 3.0F;               // in meters (simulating liftoff detection)
static constexpr float            ALT_THRESHOLD_APOGE          = 5.0F;               // in meters (simulating APOGEE)
static constexpr float            ALT_RESET_THRESHOLD          = 2.0F;               // in meters (simulating touchdown detection)

/// TIMING / BUFFER 
static constexpr unsigned long    LOOP_INTERVAL_MS            = 300UL;              // main loop interval
static constexpr unsigned long    RESET_SAVE_PERIOD           = 3000UL;             // 3 seconds after ALT_RESET_THRESHOLD
static constexpr int              BUFFER_SIZE                 = 10;                 // ten entries * 300 ms = 3 seconds

/// PIN ASSIGNMENTS FOR LED AND BUZZER
static constexpr int              PIN_GREEN_LED                = 19;                // Pin for Green LED
static constexpr int              PIN_RED_LED                  = 18;                // Pin for Red LED
static constexpr int              PIN_BUZZER                   = 4;                 // (unused for now, but reserved)

#endif 

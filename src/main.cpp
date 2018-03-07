#include <Arduino.h>
#include <EEPROM.h>
#include "RTClib.h"

// == PINS =====================================================================
#define PIN_PWM_HRS  9      // output pin for the hours voltmeter
#define PIN_PWM_MIN 10      // output pin for the minutes voltmeter
#define PIN_PWM_SEC 11      // output pin for the seconds voltmeter
#define PIN_BTN_DEC  3      // input pin for the 'DECREMENT' button
#define PIN_BTN_INC  4      // input pin for the 'INCREMENT' button

// == CONFIGURATION=============================================================
#define DEBOUNCE_MS      20   // button debounce time
#define REPEAT_FIRST_MS 500   // button first repeat delay
#define REPEAT_NEXT_MS  100   // button next repeat delays
#define CALIBRATION_POINTS 7  // Number of calibration points
#define CONFIG_VERSION 2      // Configuration structure version number
#define DEBUG 1

// == TYPES ====================================================================
struct CalibrationData {
    uint8_t pts[CALIBRATION_POINTS];
};
struct ConfigData {
    CalibrationData hours;
    CalibrationData minutes;
    CalibrationData seconds;
};
struct TimeData {
    uint8_t h;
    uint8_t m;
    uint8_t s;
};
struct ButtonData {
    uint32_t timeout:32;
    uint8_t buttons :2;
    bool debouncing : 1;
    bool emited : 1;
    bool allowRepeat : 1;
};

// == GLOBALS ==================================================================
RTC_DS3231 _rtc;
ConfigData _config;
TimeData _curTime;
uint8_t _lastShowSecond;
uint8_t _setCalibrationIdx;
ButtonData  _btnData;
uint8_t _lastState;
uint8_t _curState;

// == HELPER FUNCTIONS =========================================================

/** 
 * @brief  Maps a value from one range to the other while
 *         Constraining the value to the output range.
 * @param  x: The value to map 
 * @param  inMin: The low inclusive input range
 * @param  inMax: The high inclusive input range
 * @param  outMin: The low inclusive output range
 * @param  outMax: The high inclusive output range
 * @retval 
 */
long mapRound(long x, long inMin, long inMax, long outMin, long outMax)
{
    long result =
        ((x - inMin) * (outMax - outMin) + (inMax - inMin)/2) /
        (inMax - inMin) +
        outMin;
    
    return constrain(result, outMin, outMax);
}

/** 
 * @brief  Calculates a multi-point calibrated PWM value
 * @param  x: The zero-based value for which to obtain the calibrated PWM value
 * @param  max: The maximum possible value of X
 * @param  calibration: The calibration points
 * @retval The calculated calibrated PWM value matching x
 */
uint8_t getCalibratedPwm(uint8_t x, uint8_t max, CalibrationData calibration) {
    // Find calibration value divider and low point index
    uint8_t range = max / (CALIBRATION_POINTS-1);
    uint8_t idx = x / range;

    // Find low and high point values & calculate x within the range
    uint8_t ptMin = calibration.pts[idx];
    uint8_t ptMax = calibration.pts[idx+1];
    x = x % range;

    // Do rounded linear interpolation
    return mapRound(x, 0, range, ptMin, ptMax);
}

/** 
 * @brief  Returns an osciliating PWM value based on time.
 * @retval A PWM value osciliating slowly at each call, based on time.
 */
uint8_t getWaitPwmValue() {
  uint8_t value = ((millis() >> 4) & 0xFF);
    if (value & 0x80)
        return 191 - (value & 0x7F);
    return 64 + (value & 0x7F);
}

// == HAL ======================================================================

/** 
 * @brief  Detects button presses including repeats with long presses
 * @retval uint8_t 0 if no button press was registered,
 *                 1 if the button 1 was pressed or repeated,
 *                 2 if the button 2 was pressed or repeated,
 *                 3 if both buttons were pressed
 */
uint8_t readButton() {        
    // Extract previous state and calculate actual state
    uint8_t curButtons = 0;
    if (digitalRead(PIN_BTN_DEC) == LOW)
        curButtons |= 1;
    if (digitalRead(PIN_BTN_INC) == LOW)
        curButtons |= 2;

    // If nothing was pressed and it's still the same,
    if (!(curButtons || _btnData.buttons))
        return 0;

    // If a new button is newly pressed, (re)start full sequence
    if (curButtons > _btnData.buttons) {
        // Skip if release timeout not expired
        if (!_btnData.buttons && (long)(millis() -_btnData.timeout) < 0) {
            return 0;
        }
        // Set new down state and reset emited flag
        _btnData.buttons = curButtons;
        _btnData.emited = false;
        // ALlow repeat only if one single button is pressed
        _btnData.allowRepeat = 
            1 == _btnData.buttons ||
            2 == _btnData.buttons;
        // Start debouncing
        _btnData.debouncing = true;
        _btnData.timeout = millis() + static_cast<uint32_t>(DEBOUNCE_MS);

        // Done for now
        return 0;
    }
    
    // Wait for debounce to complete
    if (_btnData.debouncing) {
        // If debounce complete, set repeat timeout or emit if no repeat
        if ((long)(millis() -_btnData.timeout) >= 0) {
            _btnData.debouncing = false;
            _btnData.timeout += 
                static_cast<uint32_t>(REPEAT_FIRST_MS - DEBOUNCE_MS);            
        }
        else
            return 0;
    }

    // If some but not all buttons were released, we stall (invalid state)
    if (curButtons && curButtons < _btnData.buttons)
        return 0;
    
    // If fully released or repeated, then we emit a button code
    uint8_t emitButtons = 0;
    // All buttons were released, reset and emit if not yet done so
    if (curButtons == 0) {
        if (!_btnData.emited)
            emitButtons = _btnData.buttons;
        // Clear and set debounce on release        
        _btnData.buttons = 0;
        _btnData.timeout = millis() + static_cast<uint32_t>(DEBOUNCE_MS);
    }
    // Non repeat button is pressed and not yet emited, emit
    else if (!_btnData.allowRepeat) {
        if (!_btnData.emited)
            emitButtons = _btnData.buttons;
    }
    // Track for repeats
    else if ((long)(millis() - _btnData.timeout) >= 0) {
        emitButtons = _btnData.buttons;
        _btnData.timeout += static_cast<uint32_t>(REPEAT_NEXT_MS);
    }
    
    // Now emit
    if (emitButtons)
        _btnData.emited = true;
    return emitButtons;
}

/** 
 * @brief  Reads the current time
 * @retval The current time of day.
 */
TimeData readTime() {
    DateTime now = _rtc.now();

    TimeData result;
    result.h = now.hour();
    result.m = now.minute();
    result.s = now.second();
    return result;
}

/** 
 * @brief  Sets the current time
 * @note   
 * @param  hours: The hours to set
 * @param  minutes: The minutes to set
 * @retval None
 */
void writeTime(uint8_t hours, uint8_t minutes) {
    DateTime now = _rtc.now();    
    DateTime set = DateTime(
        now.year(), now.month(), now.day(),
        hours, minutes, 0);
    _rtc.adjust(set);
    #if DEBUG == 1
        Serial.print(F("Time set to "));
        Serial.print(hours);
        Serial.print(':');
        Serial.print(minutes);
    #endif
}

/** 
 * @brief  Dump the content of the EEPROM for test purposes
 * @note   
 * @retval None
 */
void eepromDump() {    
    Serial.println("EEPROM DUMP:");
    uint16_t len = EEPROM.length();
    for(uint16_t addr = 0; addr < len; ++addr) {
        uint8_t value = EEPROM.read(addr);
        if ((addr & 0x7) == 0) {
            Serial.print(addr <= 0xF ? "0x00" : (addr <= 0xFF ? "0x0" : "0x"));
            Serial.print(addr, 16);
        }

        Serial.print(value <= 0xF ? " 0" : " ");
        Serial.print(value, 16);
        if ((addr & 0x7) == 0x7)
            Serial.println();
    }
}

/** 
 * @brief  Dumps the configuration for debug purposes
 * @note   
 * @retval None
 */
void dumpConfig() {
    #if DEBUG == 1
    Serial.println(F("Configuration dump:"));
    Serial.print(F("  hours calibration points  :"));
    for(uint8_t i = 0; i < CALIBRATION_POINTS; i++) {
        uint8_t p = _config.hours.pts[i];
        Serial.print(p > 100 ? F(" ") : (p > 10 ? F("  ") : F("   ")));
        Serial.print(p);
    }
    Serial.println();
    Serial.print(F("  minutes calibration points :"));
    for(uint8_t i = 0; i < CALIBRATION_POINTS; i++) {
        uint8_t p = _config.minutes.pts[i];
        Serial.print(p > 100 ? F(" ") : (p > 10 ? F("  ") : F("   ")));
        Serial.print(p);
    }
    Serial.println();
    Serial.print(F("  seconds calibration points :"));
    for(uint8_t i = 0; i < CALIBRATION_POINTS; i++) {
        uint8_t p = _config.seconds.pts[i];
        Serial.print(p > 100 ? F(" ") : (p > 10 ? F("  ") : F("   ")));
        Serial.print(p);
    }
    Serial.println();
    #endif
}
/** 
 * @brief  Loads the configuration from the EEPROM
 * @retval None
 */
void loadConfig() {
    // Read version
    uint8_t version = EEPROM.read(0);

    // If version matches, load, otherwise, initialize configuration to default
    if (version == CONFIG_VERSION) {
        // Load configuration from EEPROM
        EEPROM.get(1, _config);

        #if DEBUG == 1
            Serial.println(F("Configuration loaded !"));
            dumpConfig();
        #endif
    }
    else {
        // Set defaults        
        for(uint8_t i = 0; i < CALIBRATION_POINTS; ++i) {
            uint8_t pt = mapRound(i, 0, CALIBRATION_POINTS-1, 0, 255);
            _config.hours.pts[i] = pt;
            _config.minutes.pts[i] = pt;
            _config.seconds.pts[i] = pt;
        }
        // Save
        EEPROM.update(0, CONFIG_VERSION);
        EEPROM.put(1, _config);

        #if DEBUG == 1
            Serial.print(F("Configuration version mismatch: expected "));
            Serial.print(CONFIG_VERSION);
            Serial.print(F(" but read "));
            Serial.print(version);
            Serial.println(F(". Defaults values set"));
        #endif
    }
}

/** 
 * @brief  Saves the configuration to the EEPROM
 * @retval None
 */
void saveConfig() {
    // Write version
    EEPROM.update(0, CONFIG_VERSION);
    // Write configuration
    EEPROM.put(1, _config);

    #if DEBUG == 1
        Serial.println(F("Copnfiguration saved !"));
        dumpConfig();
    #endif    
}

/** 
 * @brief  Writes a value to the hour voltmeter display.
 * @param  value: The value to write
 * @param  raw: true if value is a RAW uncalibrated value between 0 and 255
 *              false if value is a calibrated hour value between 0 and 24
 * @retval None
 */
void writeHours(uint8_t value, bool raw) {
    if (!raw)
        value = getCalibratedPwm(value, 24, _config.hours);
    analogWrite(PIN_PWM_HRS, value);
}

/** 
 * @brief  Writes a value to the minute voltmeter display.
 * @param  value: The value to write
 * @param  raw: true if value is a RAW uncalibrated value between 0 and 255
 *              false if value is a calibrated hour value between 0 and 60
 * @retval None
 */
void writeMinutes(uint8_t value, bool raw) {
    if (!raw)
        value = getCalibratedPwm(value, 60, _config.minutes);
    analogWrite(PIN_PWM_MIN, value);
}

/** 
 * @brief  Writes a value to the second voltmeter display.
 * @param  value: The value to write
 * @param  raw: true if value is a RAW uncalibrated value between 0 and 255
 *              false if value is a calibrated hour value between 0 and 24
 * @retval None
 */
void writeSeconds(uint8_t value, bool raw) {
    if (!raw)
        value = getCalibratedPwm(value, 60, _config.seconds);
    analogWrite(PIN_PWM_SEC, value);

}

// == STATE MACHINE ============================================================

#define STATE_NONE 0
#define STATE_SHOW_TIME 1
#define STATE_SET_HOURS 2
#define STATE_SET_MINUTES 3
#define STATE_CALIBRATE_MECHANICAL 4
#define STATE_CALIBRATE_HOURS 5
#define STATE_CALIBRATE_MINUTES 6
#define STATE_CALIBRATE_SECONDS 7

/** 
 * @brief  Implements the state that shows the time
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateShowTime(bool init) {
    #if DEBUG == 1
    if (init)
        Serial.println(F("Entering state 'ShowTime'"));
    #endif

    TimeData t = readTime();
    
    // Only push time if it is not the correctly displayed one
    if (init || _curTime.h != t.h || _curTime.m != t.m || _curTime.s != t.s) {
        writeHours(t.h, false);
        writeMinutes(t.m, false);
        writeSeconds(t.s, false);
        _curTime = t;
        #if DEBUG == 1
        Serial.print(F("Time: "));
        Serial.print(t.h);
        Serial.print(':');
        Serial.print(t.m);
        Serial.print(':');
        Serial.println(t.s);
        #endif
    }
    
    // Check press on the set button
    if (readButton() == 3)
        return STATE_SET_HOURS;

    return STATE_NONE;
}

/** 
 * @brief  Implements the state that allows to set the hours.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateSetHours(bool init) {
    if (init) {
        #if DEBUG == 1
        Serial.println(F("Entering state 'SetHours'"));
        Serial.print(F("Actual hours: "));
        Serial.println(_curTime.h);
        #endif

        writeMinutes(_curTime.m, false);
        writeSeconds(0, true);
        writeHours(0, true);
        delay(300);
        writeHours(255, true);
        delay(300);
        writeHours(_curTime.h, false);
    }

    switch(readButton()) {
        case 1:
            _curTime.h = _curTime.h > 0 ? _curTime.h - 1 : 23;
            break;
        case 2:
            _curTime.h = _curTime.h < 23 ? _curTime.h + 1 : 0;
            break;
        case 3:
            return STATE_SET_MINUTES;
        default:
            writeSeconds(getWaitPwmValue(), true);
            return STATE_NONE;   
        }

    #if DEBUG == 1
    Serial.print(F("Hours set to: "));
    Serial.println(_curTime.h);
    #endif

    writeHours(_curTime.h, false);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that allows to set the minutes.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateSetMinutes(bool init) {
      if (init) {
        #if DEBUG == 1
        Serial.println(F("Entering state 'SetMinutes'"));
        Serial.print(F("Actual minutes: "));
        Serial.println(_curTime.m);
        #endif
        writeHours(_curTime.h, false);
        writeSeconds(0, true);
        writeMinutes(0, true);
        delay(300);
        writeMinutes(255, true);
        delay(300);
        writeMinutes(_curTime.m, false);
    }

    switch(readButton()) {
        case 1:
            _curTime.m = _curTime.m > 0 ? _curTime.m - 1 : 59;
            break;
        case 2:
            _curTime.m = _curTime.m < 59 ? _curTime.m + 1 : 0;
            break;
        case 3:
            #if DEBUG == 1
            Serial.println(F("Time set done"));
            #endif
            writeTime(_curTime.h, _curTime.m);
            return STATE_SHOW_TIME;
        default:
            writeSeconds(getWaitPwmValue(), true);
            return STATE_NONE;
    }

    #if DEBUG == 1
    Serial.print(F("Minutes set to: "));
    Serial.println(_curTime.m);
    #endif

    writeMinutes(_curTime.m, false);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that allows mechanical calibration of the
 *         voltmeter displays.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateMechanical(bool init) {
    if (init) {
        #if DEBUG == 1
        Serial.println("Entering state 'CalibrateMechanical'");
        #endif
        _setCalibrationIdx = 0;
    }
    // Select minimum and maximum at around 2 second interval
    uint8_t pwmValue = ((millis() >> 12) & 0x01) * 0xFF;
    writeHours(pwmValue, true);
    writeMinutes(pwmValue, true);
    writeSeconds(pwmValue, true);

    if (readButton() != 0)
        return STATE_CALIBRATE_HOURS;
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrate the hours voltmeter display.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateHours(bool init) {
    if (init) {
        #if DEBUG == 1
        Serial.println(F("Entering state 'CalibrateHours'"));
        #endif
        _setCalibrationIdx = 0;
    }

    // Wave other displays up and down
    uint8_t waitPwm = getWaitPwmValue();
    writeMinutes(waitPwm, true);
    writeSeconds(waitPwm, true);

    switch(readButton()) {
        case 1:
            _config.hours.pts[_setCalibrationIdx]--;
            break;
        case 2:
            _config.hours.pts[_setCalibrationIdx]++;
            break;
        case 3:
            if (++_setCalibrationIdx >= CALIBRATION_POINTS)
                return STATE_CALIBRATE_MINUTES;
            break;
        default:
            if (!init)
                return STATE_NONE;
            break;
    }

    #if DEBUG == 1
    Serial.print(F("Calibration hour point "));
    Serial.print(_setCalibrationIdx);
    Serial.print(F(" value = "));
    Serial.println(_config.hours.pts[_setCalibrationIdx]);
    #endif

    writeHours(_config.hours.pts[_setCalibrationIdx], true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrate the minutes voltmeter display.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateMinutes(bool init) {
    if (init) {
        #if DEBUG == 1
        Serial.println(F("Entering state 'CalibrateMinutes'"));
        #endif
        _setCalibrationIdx = 0;
    }

    // Wave other displays up and down
    uint8_t waitPwm = getWaitPwmValue();
    writeHours(waitPwm, true);
    writeSeconds(waitPwm, true);

    switch(readButton()) {
        case 1:
            _config.minutes.pts[_setCalibrationIdx]--;
            break;
        case 2:
            _config.minutes.pts[_setCalibrationIdx]++;
            break;
        case 3:
            if (++_setCalibrationIdx >= CALIBRATION_POINTS)
                return STATE_CALIBRATE_SECONDS;
            break;
        default:
            if (!init)
                return STATE_NONE;
            break;
    }

    #if DEBUG == 1
    Serial.print(F("Calibration minute point "));
    Serial.print(_setCalibrationIdx);
    Serial.print(F(" value = "));
    Serial.println(_config.minutes.pts[_setCalibrationIdx]);
    #endif

    writeMinutes(_config.minutes.pts[_setCalibrationIdx], true);
    return STATE_NONE;
}


/** 
 * @brief  Implements the state that calibrate the seconds voltmeter display.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateSeconds(bool init) {
    if (init) {
        #if DEBUG == 1
        Serial.println(F("Entering state 'CalibrateSeconds'"));
        #endif
        _setCalibrationIdx = 0;
    }

    // Wave other displays up and down
    uint8_t waitPwm = getWaitPwmValue();
    writeHours(waitPwm, true);
    writeMinutes(waitPwm, true);

    switch(readButton()) {
        case 1:
            _config.seconds.pts[_setCalibrationIdx]--;
            break;
        case 2:
            _config.seconds.pts[_setCalibrationIdx]++;
            break;
        case 3:
            if (++_setCalibrationIdx >= CALIBRATION_POINTS) {
                #if DEBUG == 1
                Serial.println(F("Calibration done."));
                #endif
                saveConfig();
                return STATE_SHOW_TIME;
            }
            break;
        default:
            if (!init)
                return STATE_NONE;
            break;
    }

    #if DEBUG == 1
    Serial.print(F("Calibration second point "));
    Serial.print(_setCalibrationIdx);
    Serial.print(F(" value = "));
    Serial.println(_config.seconds.pts[_setCalibrationIdx]);
    #endif

    writeSeconds(_config.seconds.pts[_setCalibrationIdx], true);
    return STATE_NONE;
}

// == SETUP AND MAIN LOOP ======================================================
/** 
 * @brief Initializes the program and the peripherials
 * @retval None
 */
void setup() {
    // Initialize the Serial interface
    Serial.begin(9600);

    // Initialize the RTC
    if (!_rtc.begin())
        Serial.print(F("ERROR: RTC module initialization failed."));
    
    // Initialize the output display pins
    pinMode(PIN_PWM_HRS, OUTPUT);
    pinMode(PIN_PWM_MIN, OUTPUT);
    pinMode(PIN_PWM_SEC, OUTPUT);

    // Initiaize the button pins
    pinMode(PIN_BTN_DEC, INPUT_PULLUP);
    pinMode(PIN_BTN_INC, INPUT_PULLUP);

    // Initialize EEPROM access & load configuration
    EEPROM.begin();  
    loadConfig();

    // Initialize the state machie
    _lastState = STATE_NONE;
    _curState = STATE_SHOW_TIME;

    // Wait 5 times the debounce timeout for the "set" key to see if we
    // must enter calibration mode...
    uint32_t timeout = millis() + static_cast<uint32_t>(DEBOUNCE_MS * 5);
    while((long)(millis() - timeout) < 0) {
        if (readButton() == 3) {
            _curState = STATE_CALIBRATE_HOURS;    
            break;
        }
    }
}

/** 
 * @brief  Main program loop
 * @retval None
 */
void loop() {
    bool init = _lastState != _curState;
    _lastState = _curState;
    switch(_curState) {
        case STATE_SHOW_TIME:
            _curState = stateShowTime(init);
            break;
        case STATE_SET_HOURS:
            _curState = stateSetHours(init);
            break;
        case STATE_SET_MINUTES:
            _curState = stateSetMinutes(init);
            break;
        case STATE_CALIBRATE_MECHANICAL:
            _curState = stateCalibrateMechanical(init);
            break;
        case STATE_CALIBRATE_HOURS:
            _curState = stateCalibrateHours(init);
            break;
        case STATE_CALIBRATE_MINUTES:
            _curState = stateCalibrateMinutes(init);
            break;
        case STATE_CALIBRATE_SECONDS:
            _curState = stateCalibrateSeconds(init);
            break;
    #if DEBUG == 1
        default:
            Serial.print(F("Invalid state value: "));
            Serial.println(_curState);
    #endif            
    }

    if (_curState == STATE_NONE)
        _curState = _lastState;
}
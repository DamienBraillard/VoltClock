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
#define DEBUG 1

// == TYPES ====================================================================
struct CalibrationData {
    uint8_t min;
    uint8_t max;
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
ButtonData  _btnData;
uint8_t _lastState;
uint8_t _curState;

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
 * @brief  Loads the configuration from the EEPROM
 * @retval None
 */
void loadConfig() {
    // Read version
    uint8_t version = EEPROM.read(0);

    // If version matches, load, otherwise, initialize configuration to default
    if (version == 1) {
        EEPROM.get(1, _config);
        #if DEBUG == 1
            Serial.println(F("CONFIG LOADED:"));
            Serial.print(F("  hours.min  =")); Serial.println(_config.hours.min);
            Serial.print(F("  hours.max  =")); Serial.println(_config.hours.max);
            Serial.print(F("  minutes.min=")); Serial.println(_config.minutes.min);
            Serial.print(F("  minutes.max=")); Serial.println(_config.minutes.max);
            Serial.print(F("  seconds.min=")); Serial.println(_config.seconds.min);
            Serial.print(F("  seconds.max=")); Serial.println(_config.seconds.max);
        #endif
    }
    else {
        #if DEBUG == 1
            Serial.println(F("CONFIG SET TO DEFAULT !"));
        #endif
        
        _config.hours.min = 0x00;
        _config.hours.max = 0xFF;
        _config.minutes.min = 0x00;
        _config.minutes.max = 0xFF;
        _config.seconds.min = 0x00;
        _config.seconds.max = 0xFF;
    }
}

/** 
 * @brief  Saves the configuration to the EEPROM
 * @retval None
 */
void saveConfig() {
    // Write version
    EEPROM.update(0, 1);
    // Write configuration
    EEPROM.put(1, _config);

    #if DEBUG == 1
        Serial.println(F("CONFIG SAVED:"));
        Serial.print(F("  hours.min  =")); Serial.println(_config.hours.min);
        Serial.print(F("  hours.max  =")); Serial.println(_config.hours.max);
        Serial.print(F("  minutes.min=")); Serial.println(_config.minutes.min);
        Serial.print(F("  minutes.max=")); Serial.println(_config.minutes.max);
        Serial.print(F("  seconds.min=")); Serial.println(_config.seconds.min);
        Serial.print(F("  seconds.max=")); Serial.println(_config.seconds.max);
    #endif    
}

/** 
 * @brief  Returns the best possible PWM value for the given display value
 * @note   
 * @param  value: The zero-based display value
 * @param  range: The display value that must map to pwmMax
 * @param  pwmMin: The PWM value to return when value is 0
 * @param  pwmMax: The PWM value to return when value is range
 * @retval The best matching PWM value for value
 */
uint8_t getPwm(uint16_t value, uint16_t range, uint8_t pwmMin, uint8_t pwmMax) {
    // Constrain value to range
    if (value > range)
        value = range;

    // Add 1/2 divisor so that trunction results ends up to the next int if
    // division fractionalresult is >= .5
    return pwmMin + ((value * (uint16_t)(pwmMax-pwmMin) + (range > 1)) / range);
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
        value = getPwm(value, 24, _config.hours.min, _config.hours.max);
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
        value = getPwm(value, 60, _config.minutes.min, _config.minutes.max);
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
        value = getPwm(value, 60, _config.seconds.min, _config.seconds.max);
    analogWrite(PIN_PWM_SEC, value);

}

// == STATE MACHINE ============================================================

#define STATE_NONE 0
#define STATE_SHOW_TIME 1
#define STATE_SET_HOURS 2
#define STATE_SET_MINUTES 3
#define STATE_CALIBRATE_START 4
#define STATE_CALIBRATE_HOURS_LOW 5
#define STATE_CALIBRATE_HOURS_HIGH 6
#define STATE_CALIBRATE_MINUTES_LOW 7
#define STATE_CALIBRATE_MINUTES_HIGH 8
#define STATE_CALIBRATE_SECONDS_LOW 9
#define STATE_CALIBRATE_SECONDS_HIGH 10

/** 
 * @brief  Implements the state that shows the time
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateShowTime(bool init) {
    #if DEBUG == 1
    if (init)
        Serial.println("Entering state 'ShowTime'");
    #endif

    TimeData time = readTime();
    
    // Only push time if it is not the correctly displayed one
    if (init || _curTime.h != time.h) {
        writeHours(time.h, false);
        _curTime.h = time.h;
    }
    if (init || _curTime.m != time.m) {
        writeMinutes(time.m, false);
        _curTime.m = time.m;
    }
    if (init || _curTime.s != time.s) {
        writeSeconds(time.s, false);
        _curTime.s = time.s;
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
            Serial.println("Entering state 'SetHours'");
        #endif
        writeHours(_curTime.h, false);
        writeMinutes(_curTime.m, false);
        writeSeconds(0, true);
    }

    switch(readButton()) {
        case 1:
            if (_curTime.h > 0)
                _curTime.h--;
            else
                _curTime.h = 23;
            writeHours(_curTime.h, false);
            break;
        case 2:
            if (_curTime.h < 24)
                _curTime.h++;
            else
                _curTime.h = 0;
            writeHours(_curTime.h, false);
            break;
        case 3:
            return STATE_SET_MINUTES;
    }

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
            Serial.println("Entering state 'SetMinutes'");
        #endif
        writeHours(_curTime.h, false);
        writeMinutes(_curTime.m, false);
        writeSeconds(0, true);
    }

    switch(readButton()) {
        case 1:
            if (_curTime.m > 0)
                _curTime.m--;
            else
                _curTime.m = 59;
            writeMinutes(_curTime.m, false);
            break;
        case 2:
            if (_curTime.m < 60)
                _curTime.m++;
            else
                _curTime.m = 0;
            writeMinutes(_curTime.m, false);
            break;
        case 3:
            writeTime(_curTime.h, _curTime.m);
            return STATE_SHOW_TIME;
    }

    return STATE_NONE;

}

/** 
 * @brief  Implements the state that allows to physically calibrate the 
 *         needles ate the center.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateStart(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateStart'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    if (readButton() == 3)
        return STATE_CALIBRATE_HOURS_LOW;
    return STATE_NONE;
}
/** 
 * @brief  Implements the state that calibrates the hours low PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateHoursLow(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateHoursLow'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.hours.min > 0)
                _config.hours.min--;
            break;
        case 2:
            if (_config.hours.min < 255)
                _config.hours.min++;
            break;
        case 3:
            return STATE_CALIBRATE_HOURS_HIGH;
    }

    writeHours(_config.hours.min, true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrates the hours high PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateHoursHigh(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateHoursHigh'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.hours.max > 0)
                _config.hours.max--;
            break;
        case 2:
            if (_config.hours.max < 255)
                _config.hours.max++;
            break;
        case 3:
            return STATE_CALIBRATE_MINUTES_LOW;
    }

    writeHours(_config.hours.max, true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrates the minutes low PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateMinutesLow(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateMinutesLow'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.minutes.min > 0)
                _config.minutes.min--;
            break;
        case 2:
            if (_config.minutes.min < 255)
                _config.minutes.min++;
            break;
        case 3:
            return STATE_CALIBRATE_MINUTES_HIGH;
    }

    writeMinutes(_config.minutes.min, true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrates the minutes high PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateMinutesHigh(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateMinutesHigh'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.minutes.max > 0)
                _config.minutes.max--;
            break;
        case 2:
            if (_config.minutes.max < 255)
                _config.minutes.max++;
            break;
        case 3:
            return STATE_CALIBRATE_SECONDS_LOW;
    }

    writeMinutes(_config.minutes.max, true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrates the seconds low PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateSecondsLow(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateSecondsLow'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.seconds.min > 0)
                _config.seconds.min--;
            break;
        case 2:
            if (_config.seconds.min < 255)
                _config.seconds.min++;
            break;
        case 3:
            return STATE_CALIBRATE_SECONDS_HIGH;
    }

    writeSeconds(_config.seconds.min, true);
    return STATE_NONE;
}

/** 
 * @brief  Implements the state that calibrates the seconds high PWM value.
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the current state.
 */
uint8_t stateCalibrateSecondsHigh(bool init) {
    if (init) {
        #if DEBUG == 1
            Serial.println("Entering state 'CalibrateSecondsHigh'");
        #endif
        writeHours(128, true);
        writeMinutes(128, true);
        writeSeconds(128, true);
    }

    switch(readButton()) {
        case 1:
            if (_config.seconds.max > 0)
                _config.seconds.max--;
            break;
        case 2:
            if (_config.seconds.max < 255)
                _config.seconds.max++;
            break;
        case 3:
            saveConfig();
            return STATE_SHOW_TIME;
    }

    writeSeconds(_config.seconds.max, true);
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
            _curState = STATE_CALIBRATE_START;    
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
        case STATE_CALIBRATE_START:
            _curState = stateCalibrateStart(init);
            break;
        case STATE_CALIBRATE_HOURS_LOW:
            _curState = stateCalibrateHoursLow(init);
            break;
        case STATE_CALIBRATE_HOURS_HIGH:
            _curState = stateCalibrateHoursHigh(init);
            break;
        case STATE_CALIBRATE_MINUTES_LOW:
            _curState = stateCalibrateMinutesLow(init);
            break;
        case STATE_CALIBRATE_MINUTES_HIGH:
            _curState = stateCalibrateMinutesHigh(init);
            break;
        case STATE_CALIBRATE_SECONDS_LOW:
            _curState = stateCalibrateSecondsLow(init);
            break;
        case STATE_CALIBRATE_SECONDS_HIGH:
            _curState = stateCalibrateSecondsHigh(init);
            break;
    #if DEBUG == 1
        default:
            Serial.print("Invalid state value: ");
            Serial.println(_curState);
    #endif            
    }

    if (_curState == STATE_NONE)
        _curState = _lastState;
}
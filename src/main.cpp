#include <Arduino.h>
#include <EEPROM.h>
#include "RTClib.h"

// == PINS =====================================================================
#define PIN_PWM_HRS  9      // output pin for the hours voltmeter
#define PIN_PWM_MIN 10      // output pin for the minutes voltmeter
#define PIN_PWM_SEC 11      // output pin for the seconds voltmeter
#define PIN_BTN_SET  2      // input pin for the 'SET' button
#define PIN_BTN_DEC  3      // input pin for the 'DECREMENT' button
#define PIN_BTN_INC  4      // input pin for the 'INCREMENT' button

// == CONSTANTS ================================================================
#define DEBOUNCE_MS      20   // button debounce time
#define REPEAT_FIRST_MS 500   // button first repeat delay
#define REPEAT_NEXT_MS  100   // button next repeat delays
#define DEBUG 1
// == TYPES ====================================================================
struct calibration_t {
    uint8_t min;
    uint8_t max;
};
struct config_t {
    calibration_t hours;
    calibration_t minutes;
    calibration_t seconds;
};
struct timeofday_t {
    uint8_t h;
    uint8_t m;
    uint8_t s;
};

// == GLOBALS ==================================================================
RTC_DS3231 _rtc;
config_t _config;
timeofday_t _curTime;
uint8_t _btnState = 0;
uint32_t _btnTimeout = 0;
uint8_t _lastState;
uint8_t _curState;

// == HAL ======================================================================

/** 
 * @brief  Detects button presses including repeats with long presses
 * @retval char '\0' if no button press was registered,
 *              'S', '+', '-' if the Set, Inc or Dec button was pressed
 *              or a repeat occurred due to a long press.
 */
char readButton() {    
    const struct { uint8_t pin; char value; bool repeat; } BTN_INFO[3] = {
        {PIN_BTN_SET, 'S', false },
        {PIN_BTN_INC, '+', true },
        {PIN_BTN_DEC, '-', true }
    };
    const uint8_t COUNT = sizeof(BTN_INFO) / sizeof(BTN_INFO[0]);
    const uint8_t FLAG_DEBOUNCE  = 0x80;
    const uint8_t FLAG_REPEATED  = 0x40;
    const uint8_t FLAG_DOWN      = 0x20;
    const uint8_t BUTTON_MASK    = 0x0F;

    // No button currently pressed, check if a new press is being detected
    if (!(_btnState & FLAG_DOWN)) {
        // Detect a press or return now
        for(uint8_t i = 0; i < COUNT; i++)
            if (digitalRead(BTN_INFO[i].pin) == LOW) {
                _btnState = i | FLAG_DOWN;
                break;
            }
        if (!(_btnState & FLAG_DOWN))
            return 0;

        Serial.println(F("BUTTON: Press detected"));

        // Start debounce
        _btnTimeout = millis() + DEBOUNCE_MS;
        _btnState |= FLAG_DEBOUNCE;
    }
    uint8_t idx = _btnState & BUTTON_MASK;

    // If debouncing, wait for the debounce timeout
    if (_btnState & FLAG_DEBOUNCE) {
        // Debounce still pending ?
        if ((long)(millis() -_btnTimeout) < 0)
            return 0;
        
        // Debounce complete !
        _btnState &= ~FLAG_DEBOUNCE;
        if (BTN_INFO[idx].repeat) {
            // Repeating: Set first repeat timeout & remove debounce flag
            Serial.println(F("BUTTON: Debounce complete"));
            _btnTimeout += (uint32_t)(REPEAT_FIRST_MS - DEBOUNCE_MS);
        }
        else {
            // Not repeating, send press result now
            return BTN_INFO[idx].value;
        }
    }

    // If released, reset state and return released button if repeating and not yet repeated
    if (digitalRead(BTN_INFO[idx].pin) == HIGH) {
        Serial.println(F("BUTTON: Released"));
        bool returnValue = BTN_INFO[idx].repeat && !(_btnState & FLAG_REPEATED);
        _btnState = 0;
        return returnValue ? BTN_INFO[idx].value : 0;
    }

    // Handle repeats
    if (BTN_INFO[idx].repeat && (long)(millis() -_btnTimeout) >= 0) {
        Serial.println(F("BUTTON: Repeat detected"));
        _btnState |= FLAG_REPEATED;
        _btnTimeout += (uint32_t)REPEAT_NEXT_MS;
        return BTN_INFO[idx].value;
    }

    // Fallback
    return 0;
}

/** 
 * @brief  Reads the current time
 * @retval The current time of day.
 */
timeofday_t readTime() {
    DateTime now = _rtc.now();

    timeofday_t result;
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

    timeofday_t time = readTime();
    
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
    if (readButton() == 'S')
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
        case 'S':
            return STATE_SET_MINUTES;
        case '+':
            if (_curTime.h < 24)
                _curTime.h++;
            else
                _curTime.h = 0;
            writeHours(_curTime.h, false);
            break;
        case '-':
            if (_curTime.h > 0)
                _curTime.h--;
            else
                _curTime.h = 23;
            writeHours(_curTime.h, false);
            break;
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
        case 'S':
            writeTime(_curTime.h, _curTime.m);
            return STATE_SHOW_TIME;
        case '+':
            if (_curTime.m < 60)
                _curTime.m++;
            else
                _curTime.m = 0;
            writeMinutes(_curTime.m, false);
            break;
        case '-':
            if (_curTime.m > 0)
                _curTime.m--;
            else
                _curTime.m = 59;
            writeMinutes(_curTime.m, false);
            break;
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

    if (readButton() == 'S')
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
        case 'S':
            return STATE_CALIBRATE_HOURS_HIGH;
        case '+':
            if (_config.hours.min < 255)
                _config.hours.min++;
            break;
        case '-':
            if (_config.hours.min > 0)
                _config.hours.min--;
            break;
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
        case 'S':
            return STATE_CALIBRATE_MINUTES_LOW;
        case '+':
            if (_config.hours.max < 255)
                _config.hours.max++;
            break;
        case '-':
            if (_config.hours.max > 0)
                _config.hours.max--;
            break;
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
        case 'S':
            return STATE_CALIBRATE_MINUTES_HIGH;
        case '+':
            if (_config.minutes.min < 255)
                _config.minutes.min++;
            break;
        case '-':
            if (_config.minutes.min > 0)
                _config.minutes.min--;
            break;
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
        case 'S':
            return STATE_CALIBRATE_SECONDS_LOW;
        case '+':
            if (_config.minutes.max < 255)
                _config.minutes.max++;
            break;
        case '-':
            if (_config.minutes.max > 0)
                _config.minutes.max--;
            break;
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
        case 'S':
            return STATE_CALIBRATE_SECONDS_HIGH;
        case '+':
            if (_config.seconds.min < 255)
                _config.seconds.min++;
            break;
        case '-':
            if (_config.seconds.min > 0)
                _config.seconds.min--;
            break;
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
        case 'S':
            saveConfig();
            return STATE_SHOW_TIME;
        case '+':
            if (_config.seconds.max < 255)
                _config.seconds.max++;
            break;
        case '-':
            if (_config.seconds.max > 0)
                _config.seconds.max--;
            break;
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
    pinMode(PIN_BTN_SET, INPUT_PULLUP);
    pinMode(PIN_BTN_DEC, INPUT_PULLUP);
    pinMode(PIN_BTN_INC, INPUT_PULLUP);

    // Initialize EEPROM access & load configuration
    EEPROM.begin();  
    loadConfig();

    // Initialize the state machie
    _lastState = STATE_NONE;
    _curState = STATE_SHOW_TIME;

    // If "S" button is pressed at startup, then we enter calibration mode
    if (digitalRead(PIN_BTN_SET) == LOW) {
        while(readButton() != 'S');
        _curState = STATE_CALIBRATE_START;    
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
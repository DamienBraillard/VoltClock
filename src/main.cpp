#include <Arduino.h>
#include <EEPROM.h>
#include "RTClib.h"

// == PINS =====================================================================
#define PIN_PWM_HRS  9      // output pin for the hours voltmeter
#define PIN_PWM_MIN 10      // output pin for the minutes voltmeter
#define PIN_PWM_SEC 11      // output pin for the seconds voltmeter
#define PIN_BTN_SET  2      // input pin for the 'SET' button
#define PIN_BTN_INC  3      // input pin for the 'INCREMENT' button
#define PIN_BTN_DEC  4      // input pin for the 'DECREMENT' button

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
timeofday_t _displayedTime;
uint8_t _btnState = 0;
uint32_t _btnTimeout = 0;

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
 * @brief  Writes uncalibrated RAW value to the hour voltmeter display.
 * @note   
 * @param  pwm: The raw uncalibrated PWM value (0-255) to write.
 * @retval None
 */
void writeHoursRaw(uint8_t pwm) {
    analogWrite(PIN_PWM_HRS, pwm);
    _displayedTime.h = 0xFF;

    #if DEBUG == 1
    Serial.print("Write hours: ");
    Serial.println(pwm);
    #endif
}

/** 
 * @brief  Writes uncalibrated RAW value to the minute voltmeter display.
 * @note   
 * @param  pwm: The raw uncalibrated PWM value (0-255) to write.
 * @retval None
 */
void writeMinutesRaw(uint8_t pwm) {
    analogWrite(PIN_PWM_MIN, pwm);
    _displayedTime.m = 0xFF;
    #if DEBUG == 1
    Serial.print("Write minutes: ");
    Serial.println(pwm);
    #endif
}

/** 
 * @brief  Writes uncalibrated RAW value to the seconds voltmeter display.
 * @note   
 * @param  pwm: The raw uncalibrated PWM value (0-255) to write.
 * @retval None
 */
void writeSecondsRaw(uint8_t pwm) {
    analogWrite(PIN_PWM_SEC, pwm);
    _displayedTime.s = 0xFF;
    #if DEBUG == 1
    Serial.print("Write seconds: ");
    Serial.println(pwm);
    #endif
}

/** 
 * @brief  Writes time to the voltmeter display using calibration values.
 * @note   
 * @param  time: The time to write
 * @retval None
 */
void writeTime(timeofday_t time) {
    #if DEBUG == 1
    Serial.print("Write time: ");
    Serial.print(time.h); Serial.print(':');
    Serial.print(time.m); Serial.print(':');
    Serial.println(time.s); 
    #endif

    writeHoursRaw(
        getPwm(time.h, 24, _config.hours.min, _config.hours.max));
    writeMinutesRaw(
        getPwm(time.m, 60, _config.minutes.min, _config.minutes.max));
    writeSecondsRaw(
        getPwm(time.s, 60, _config.seconds.min, _config.seconds.max));

    _displayedTime = time;
}

// == STATE MACHINE ============================================================

#define STATE_NONE 0

/** 
 * @brief  Handles the "Show Time" state
 * @param  init: true if a state transition just happened; otherwise, false;
 * @retval The new state or STATE_NONE to stay in the same state.
 */
uint8_t stateShowTime(bool init) {
    timeofday_t time = readTime();
    
    // Only push time if it is not the correctly displayed one
    if (_displayedTime.s != time.s ||
        _displayedTime.m != time.m ||
        _displayedTime.h != time.h)
        writeTime(time);        
    
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
}

/** 
 * @brief  Main program loop
 * @retval None
 */
void loop() {  
}
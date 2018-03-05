#include <Arduino.h>
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

// == GLOBALS ==================================================================
RTC_DS3231 _rtc;
uint8_t _lastSecond = 0;
uint8_t _btnState = 0;
uint32_t _btnTimeout = 0;


// == CODE =====================================================================
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
 * @brief  Updates the time display
 * @retval None
 */
void updateDisplay() {
    DateTime now = _rtc.now();
    if (now.second() != _lastSecond) {
        _lastSecond = now.second();

        uint8_t pwmH = map(now.hour(), 0, 23, 0, 255);
        uint8_t pwmM = map(now.minute()*60l+now.second(), 0, 3599, 0, 255);
        uint8_t pwmS = map(now.second(), 0, 59, 0, 255);

        /*
        Serial.print(F("Writing time: "));
        Serial.print(now.hour()); Serial.print(':');
        Serial.print(now.minute()); Serial.print(':');
        Serial.print(now.second());
        Serial.print(F(" ("));
        Serial.print(pwmH); Serial.print(',');
        Serial.print(pwmM); Serial.print(',');
        Serial.print(pwmS);
        Serial.println(')');
        */

        // Update the display
        analogWrite(PIN_PWM_HRS, pwmH);
        analogWrite(PIN_PWM_MIN, pwmM);
        analogWrite(PIN_PWM_SEC, pwmS);
    }    
}



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

    
}

uint16_t value = 0;

/** 
 * @brief  Main program loop
 * @retval None
 */
void loop() {
    updateDisplay();

    char button = readButton();
    if (button) {
        Serial.print("Button press or repeat: ");
        Serial.println(button);
    }
}
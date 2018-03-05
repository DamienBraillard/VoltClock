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
char _btnState = 0;
uint32_t _btnTimeout = 0;


// == CODE =====================================================================
/** 
 * @brief  Detects button presses including repeats with long presses
 * @retval char '\0' if no button press was registered,
 *              'S', '+', '-' if the Set, Inc or Dec button was pressed
 *              or a repeat occurred due to a long press.
 */
char readButton() {
    const uint8_t BTN_PINS[3] = { PIN_BTN_SET, PIN_BTN_INC, PIN_BTN_DEC };
    const char BTN_VALUES[3] = { 'S', '+', '-' };
    const uint8_t FLAG_DEBOUNCE = 0x80;
    const uint8_t FLAG_REPEATED = 0X40;
    const uint8_t FLAG_DOWN     = 0x20;
    const uint8_t BUTTON_MASK   = 0x0F;

    // No button currently pressed, check if a new press is being detected
    if (!(_btnState & FLAG_DOWN)) {
        // Detect a press or return now
        for(uint8_t i = 0; !_btnState && i < sizeof(BTN_PINS); i++)
            if (digitalRead(BTN_PINS[i]) == LOW)
                _btnState = i | FLAG_DOWN;
        if (!(_btnState & FLAG_DOWN))
            return 0;

        Serial.println(F("BUTTON: Press detected"));

        // Start debounce
        _btnTimeout = millis() + DEBOUNCE_MS;
        _btnState |= FLAG_DEBOUNCE;
    }

    // If debouncing, wait for the debounce timeout
    if (_btnState & FLAG_DEBOUNCE) {
        // Debounce still pending ?
        if ((long)(millis() -_btnTimeout) < 0)
            return 0;
        
        // Debounce complete !
        // Set first repeat timeout & remove debounce flag
        Serial.println(F("BUTTON: Debounce complete"));
        _btnTimeout += (uint32_t)(REPEAT_FIRST_MS - DEBOUNCE_MS);
        _btnState &= ~FLAG_DEBOUNCE;
    }

    // If released, reset state and return released button if not yet repeated
    if (digitalRead(BTN_PINS[_btnState & BUTTON_MASK]) == HIGH) {
        Serial.println(F("BUTTON: Released"));
        char result = 0;
        if (!(_btnState & FLAG_REPEATED))
            result = BTN_VALUES[_btnState & BUTTON_MASK];
        _btnState = 0;
        return result;
    }

    // Handle repeats
    if ((long)(millis() -_btnTimeout) >= 0) {
        Serial.println(F("BUTTON: Repeat detected"));
        _btnState |= FLAG_REPEATED;
        _btnTimeout += (uint32_t)REPEAT_NEXT_MS;
        return BTN_VALUES[_btnState & BUTTON_MASK];
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


        Serial.print(F("Writing time: "));
        Serial.print(now.hour()); Serial.print(':');
        Serial.print(now.minute()); Serial.print(':');
        Serial.print(now.second());
        Serial.print(F(" ("));
        Serial.print(pwmH); Serial.print(',');
        Serial.print(pwmM); Serial.print(',');
        Serial.print(pwmS);
        Serial.println(')');

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
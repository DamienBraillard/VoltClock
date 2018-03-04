#include <Arduino.h>
#include "RTClib.h"

// == PINS =========================================================================================
#define PIN_PWM_HRS  9
#define PIN_PWM_MIN 10
#define PIN_PWM_SEC 11
#define PIN_BTN_INC  2
#define PIN_BTN_DEC  3

// == GLOBALS ======================================================================================
RTC_DS3231 _rtc;
uint8_t _lastSecond;


// == CODE =========================================================================================

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
}
// arduino lib
#include <Arduino.h>

// >>> ### --- SHT45 TEMPERATURE & HUMIDITY SENSOR --- ### >>>
// Cable black = SHIELD on Case
// Cable red = VDD 5V PIN_5V
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
// Cable black = GND PIN_GND
#include <Wire.h>
#include <ArtronShop_SHT45.h>
ArtronShop_SHT45 sensor(&Wire, 0x44); // default is right (&Wire, 0x44) SHT45-AD1B => I2C address 0x44
float temperature;
float humidity;
constexpr int MAX_TEMPERATURE = 18;
constexpr int MAX_HUMIDITY = 58;
constexpr int MED_HUMIDITY = 55;

// >>> ### --- DISPLAY --- ### >>>
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
#include <LiquidCrystal_I2C.h>       //Include lib
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for 16 chars and 2 line display

// >>> ### --- COOLING_COMPRESSOR RELAY --- ### >>>
constexpr int COOLING_COMPRESSOR_PIN = 9; // Signal to SPDT, 30 A
#include "Relay.h"
Relay cooling_compressor(COOLING_COMPRESSOR_PIN, true);


// >>> ### --- PWM CASE FAN 25 kHz --- ### >>>
struct Pin {
    uint8_t pwmPin;
};

constexpr Pin FAN_PWM_PIN_A{3};
constexpr Pin FAN_PWM_PIN_B{11};
constexpr int DEFAULT_SPEED = 5;


// overwrite default analogWrite function for pwm fan pins
void analogWrite(const Pin &pin, const uint8_t percent) {
    if (pin.pwmPin == FAN_PWM_PIN_A.pwmPin || pin.pwmPin == FAN_PWM_PIN_B.pwmPin)
        OCR1A = map(percent, 0, 100, 0, ICR1);
}

byte textLen;

void setup() {
    // reset the atmega timer on nano
    TCNT1 = 0; // Count Increment or decrement TCNT1 by 1. set to 0
    TCCR1A = 0;
    TCCR1B = 0; // TCCR1 register to 0
    ICR1 = 320; // counter limiter
    TCCR1B |= (1 << WGM13);
    TCCR1A |= (1 << WGM11); // 16.4 Timer/Counter Clock Sources Register B (TCCR1B)
    TCCR1A |= (1 << COM1A1); // 16.2.1 Registers
    TCCR1B |= (1 << CS10);

    // inti serial
    Serial.begin(9600);

    // Init the LCD-Display
    lcd.init();
    lcd.backlight();

    // init wire chanel check SHT45 connectet
    Wire.begin();
    textLen = sizeof("SHT45 not found !") - 1;
    while (!sensor.begin()) {
        Serial.println("SHT45 not found !");
        // add error to display
        lcd.print("SHT45 not found !");
        // Scroll hidden text through entire row to the right outside the screen
        for (byte positionCounter = 0; positionCounter < textLen + 16; positionCounter++) {
            lcd.scrollDisplayRight();
            delay(500);
        }
    }

    // Relay init the pin set it to "OUTPUT"
    cooling_compressor.begin();
}

void loop() {
    // if the sensor connected
    if (sensor.measure()) {
        // serial debug message
        Serial.print("Temperature: ");
        Serial.print(sensor.temperature(), 1);
        Serial.print(" *C\tHumidity: ");
        Serial.print(sensor.humidity(), 1);
        Serial.print(" %RH");
        Serial.println();

        // get sensor values
        temperature = sensor.temperature();
        humidity = sensor.humidity();

        // HANDLE TEMPERATURE
        //  is HIGH
        if (temperature > MAX_TEMPERATURE) {
            // start compressor
            cooling_compressor.turnOn();
            // display message
            lcd.setCursor(0, 0);
            lcd.print("Die Kuehlung ist eingeschaltet");
            lcd.setCursor(0, 1);
            lcd.print(String("Temperatur: ") + String(temperature));
            // Fan circulation on
            analogWrite(FAN_PWM_PIN_A, 40);
            analogWrite(FAN_PWM_PIN_B, 50);
        } else {
            // stop compressor
            cooling_compressor.turnOff();
            // display message
            lcd.setCursor(0, 0);
            lcd.print("Kuehlung aus");
            lcd.setCursor(0, 1);
            lcd.print(String("Temperatur: ") + String(temperature));
        }

        delay(10000); // 10 second to next


        // HANDLE HUMIDITY
        if (humidity >= MAX_HUMIDITY) {
            // MAX_HUMIDITY: Fast fan circulation it is real wet
            analogWrite(FAN_PWM_PIN_A, 70);
            analogWrite(FAN_PWM_PIN_B, 80);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Sehr feuchte Luft: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Luefter Cyrculation sehr schnell!");
        } else if (humidity > MED_HUMIDITY) {
            // MED_HUMIDITY: Moderate fan circulation
            analogWrite(FAN_PWM_PIN_A, 40);
            analogWrite(FAN_PWM_PIN_B, 50);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Die Luftfeuchtigkeit ist: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Lüfter laufen langsam.");
        } else {
            // DEFAULT: Slow fan cyrculation
            analogWrite(FAN_PWM_PIN_A, 5);
            analogWrite(FAN_PWM_PIN_B, 8);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Die Luftfeuchtigkeit ist: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Luefter laufen langsam.");
        }

        delay(10000); // 10 second to next
    } else {
        // sensor connection error run fan by default low
        analogWrite(FAN_PWM_PIN_A, 10);
        analogWrite(FAN_PWM_PIN_B, 13);
        // print out error message
        Serial.println("SHT45 read error");
        lcd.setCursor(0, 0);
        lcd.print("SHT45 sensor read error");
    }
}

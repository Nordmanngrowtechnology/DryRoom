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
constexpr float MAX_TEMPERATURE = 18.0;
constexpr float MAX_HUMIDITY = 58.0;
constexpr float MED_HUMIDITY = 55.0;

// >>> ### --- DISPLAY --- ### >>>
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
#include <LiquidCrystal_I2C.h>       //Include lib
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for 16 chars and 2 line display

// >>> ### --- COOLING_COMPRESSOR RELAY --- ### >>>
constexpr int COOLING_COMPRESSOR_PIN = 9; // Signal to SPDT, 30 A

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
unsigned long previousMillis = 0;
unsigned long interval = 10000; // 10 second
bool checkup = false;

bool startUpCheck() {
    // display message
    lcd.setCursor(0, 0);
    lcd.print("Startup Check");
    analogWrite(FAN_PWM_PIN_A, 100);
    analogWrite(FAN_PWM_PIN_B, 100);
    delay(1500);
    analogWrite(FAN_PWM_PIN_A, 50);
    analogWrite(FAN_PWM_PIN_B, 50);
    delay(1500);
    analogWrite(FAN_PWM_PIN_A, 25);
    analogWrite(FAN_PWM_PIN_B, 25);
    delay(1500);
    analogWrite(FAN_PWM_PIN_A, 0);
    analogWrite(FAN_PWM_PIN_B, 0);
    delay(1500);
    lcd.clear();
    return true;
}

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

    // Relay init the pin set it to "OUTPUT"
    pinMode(COOLING_COMPRESSOR_PIN, OUTPUT);
    digitalWrite(COOLING_COMPRESSOR_PIN, LOW);
    delay(15);

    // pwm
    pinMode(FAN_PWM_PIN_A.pwmPin, OUTPUT);
    delay(15);
    pinMode(FAN_PWM_PIN_B.pwmPin, OUTPUT);
    delay(15);

    // init wire chanel check SHT45 connected
    Wire.begin();
    while (!sensor.begin()) {
        Serial.println("SHT45 notfound!");
        // add error to display
        lcd.print("SHT45 notfound!");
        delay(2000);
    }
}

void loop() {
    // Boot up fan check
    if (checkup == false) {
        checkup = startUpCheck();
    }


    // if the sensor connected
    if (sensor.measure()) {
        // serial debug message
        Serial.print("Temperature: ");
        Serial.print(sensor.temperature(), 1);
        Serial.print(" *C\tHumidity: ");
        Serial.print(sensor.humidity(), 1);
        Serial.print(" %RH");
        Serial.println();

        // time marker
        const unsigned long currentMillis = millis();

        // HANDLE TEMPERATURE
        //  is HIGH
        if (temperature >= MAX_TEMPERATURE) { digitalWrite(COOLING_COMPRESSOR_PIN, HIGH); }

        if (currentMillis - previousMillis >= interval) {
            lcd.clear();
            temperature = sensor.temperature();
            if (temperature >= MAX_TEMPERATURE) {
                // start compressor
                digitalWrite(COOLING_COMPRESSOR_PIN, HIGH);
                // display message
                lcd.setCursor(0, 0);
                lcd.print("Kuehlung ON");
                lcd.setCursor(0, 1);
                lcd.print(String("Temperatur: ") + String(temperature));
                // Fan circulation on
                analogWrite(FAN_PWM_PIN_A, 30);
                analogWrite(FAN_PWM_PIN_B, 40);
            } else {
                // stop compressor
                digitalWrite(COOLING_COMPRESSOR_PIN, LOW);
                // display message
                lcd.setCursor(0, 0);
                lcd.print("Kuehlung OFF");
                lcd.setCursor(0, 1);
                lcd.print(String("Temperatur: ") + String(temperature));
            }
        }

        // set new time
        previousMillis = currentMillis;

        // HANDLE HUMIDITY
        humidity = sensor.humidity();
        if (humidity >= MAX_HUMIDITY) {
            // MAX_HUMIDITY: Fast fan circulation it is real wet
            analogWrite(FAN_PWM_PIN_A, 30);
            analogWrite(FAN_PWM_PIN_B, 40);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Humidity: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Fast Fan!");
        } else if (humidity > MED_HUMIDITY) {
            // MED_HUMIDITY: Moderate fan circulation
            analogWrite(FAN_PWM_PIN_A, 30);
            analogWrite(FAN_PWM_PIN_B, 40);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Humidity: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Lower Fan!.");
        } else {
            // DEFAULT: Slow fan circulation
            analogWrite(FAN_PWM_PIN_A, 15);
            analogWrite(FAN_PWM_PIN_B, 13);
            // display message
            lcd.setCursor(0, 0);
            lcd.print(String("Humidity: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Slow Fan!.");
        }
    } else {
        // sensor connection error run fan by default low
        analogWrite(FAN_PWM_PIN_A, 5);
        analogWrite(FAN_PWM_PIN_B, 8);
        // print out error message
        Serial.println("SHT45 read error");
        lcd.setCursor(0, 0);
        lcd.print("SHT45 sensor read error");
    }
}

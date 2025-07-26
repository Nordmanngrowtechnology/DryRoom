// arduino lib
#include <Arduino.h>

// >>> ### --- SHT45 TEMPERATURE & HUMIDITY SENSOR --- ### >>>
// Cable black = SHIELD on Case
// Cable red = VDD 5V PIN_5V
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
// Cable black = GND PIN_GND
#include <ArtronShop_SHT45.h>
#include <Wire.h>
ArtronShop_SHT45 sensor(&Wire,
                        0x44); // default is right (&Wire, 0x44) SHT45-AD1B => I2C address 0x44
float temperature;
float humidity;
constexpr float MIN_TEMPERATURE = 17.0; // cooling stops
constexpr float MAX_TEMPERATURE = 19.0; // cooling starts
constexpr float MAX_HUMIDITY = 58.0;    // Circulation max speed
constexpr float MIN_HUMIDITY = 54.0;    // Below this value Circulation FAN_MINIMUM_DUTY_CYCLE

// >>> ### --- DISPLAY --- ### >>>
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
#include <LiquidCrystal_I2C.h> //Include lib
LiquidCrystal_I2C lcd(0x27, 16,
                      2); // set the LCD address to 0x27 for 16 chars and 2 line display

// >>> ### --- COOLING_COMPRESSOR RELAY --- ### >>>
constexpr int COOLING_COMPRESSOR_PIN = 5; // Signal to SPDT, 30 A

/*
 * Configuration for the PWM fan signal and the circulation setup.
 * Bind the pins
 */
struct Pin
{
    uint8_t pwmPin;
};

constexpr Pin FAN_PWM_PIN_TOP_FAN{10};         // 10
constexpr Pin FAN_PWM_PIN_BOTTOM_FAN{9};       // 9
constexpr uint8_t FAN_MINIMUM_DUTY_CYCLE = 7;  // The lowest PWM Duty Cycle the fank can look
                                               // in datasheet for your fan
constexpr uint8_t FAN_MAXIMAL_DUTY_CYCLE = 50; // Maximal Fan Speed
constexpr uint8_t CIRCULATION_SPREAD = 5;      // Circulation spread between pwm groups top fan bottom fan
bool fanOn = false;

// INT CONF
unsigned long previousMillis = 0;
unsigned long temp_interval = 30000; // 30 second temp_interval for checkup relay off?
unsigned long dry_interval = 180000; // 3 Minutes 180000ms

void setup()
{
    cli(); // disable interrupts

    /*
     *  FIND MORE INFORMATION HOW IT WORKS
     *
     *  DEFINITION TIMER1 FOR ATMEL ATmega328P
     *  https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     *  https://www.mikrocontroller.net/articles/AVR-Tutorial:_Timer
     *
     *
     *  CHANGE TIMER1 SETTINGS FOR 25kHz SIGNAL CALCULATION
     */

    // RESET  Clear Timer/Counter control registers
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;

    // Set TOP value for 25kHz frequency (ICR1)
    ICR1 = 320; // Set TOP limiter to 320 >>> update OCR1A at TOP

    // REGISTER DEFINITION
    TCCR1A |= 1 << WGM11; // PWM, Phase Correct
    TCCR1B |= 1 << WGM13; // The phase correct Pulse Width Modulation or phase
                          // correct PWM mode (WGM13:0 = 1
    TCCR1B |= 1 << CS10;  // clkI/O/1 (No prescaling)

    // Need COM1A1 and COM1B1 for tow pin output . Clear OC1A/OC1B on compare
    // match
    TCCR1A |= 1 << COM1A1 | 1 << COM1B1;
    // Clear OC1A/OC1B on Compare match when down counting set output to low
    // level

    // Set Arduino Nano pins to output with Data Direction Registers B (Arduino
    // Pin 9 is PB1), (Arduino Pin 10 is PB2) is a part of Port B
    DDRB |= 1 << 1 | 1 << 2; // set as OUTPUT

    // Set RELAY pin PD5 is a part off Port D
    DDRD |= 1 << 5;     // set as OUTPUT
    PORTD &= ~(1 << 5); // set to LOW

    // end
    sei(); // allow interrupts

    // inti serial
    Serial.begin(9600);

    // Init the LCD-Display
    lcd.init();
    lcd.backlight();

    /*// Relay init the pin set it to "OUTPUT"
    pinMode(COOLING_COMPRESSOR_PIN, OUTPUT);
    digitalWrite(COOLING_COMPRESSOR_PIN, LOW);
    delay(15);*/

    // init wire chanel check SHT45 connected
    Wire.begin();
    while (!sensor.begin())
    {
        Serial.println("SHT45 notfound!");
        // add error to display
        lcd.print("SHT45 not found!");
        delay(2000);
    }
}

// overwrite default analogWrite function for pwm fan pins
void analogWrite(const Pin &pin, const uint8_t percent)
{
    if (pin.pwmPin == FAN_PWM_PIN_TOP_FAN.pwmPin)
    {
        OCR1A = map(percent, 0, 100, 0, ICR1);
    } // ICR1 320
    if (pin.pwmPin == FAN_PWM_PIN_BOTTOM_FAN.pwmPin)
    {
        OCR1B = map(percent, 0, 100, 0, ICR1);
    } // ICR1 320
}

void circulation(const uint8_t percent)
{

    if (percent - CIRCULATION_SPREAD <= FAN_MINIMUM_DUTY_CYCLE)
    {
        // minimum circulation
        analogWrite(FAN_PWM_PIN_TOP_FAN, FAN_MINIMUM_DUTY_CYCLE);
        analogWrite(FAN_PWM_PIN_BOTTOM_FAN, percent);
    }
    else
    {
        // calculated circulation
        analogWrite(FAN_PWM_PIN_TOP_FAN, percent - CIRCULATION_SPREAD);
        analogWrite(FAN_PWM_PIN_BOTTOM_FAN, percent);
    }

    // print
    Serial.print("Fan run ");
    Serial.print(percent);
    Serial.println("%");
    delay(1000);
}

// calculate the speed
uint8_t mySpeed(const float humi)
{

    if (humi <= MIN_HUMIDITY)
    {
        fanOn = true;
        return FAN_MINIMUM_DUTY_CYCLE;
    }

    constexpr float h_range = 100 - MIN_HUMIDITY;                            // 46
    constexpr int s_range = FAN_MAXIMAL_DUTY_CYCLE - FAN_MINIMUM_DUTY_CYCLE; // 44

    constexpr float part = s_range / h_range;

    const auto speed = static_cast<uint8_t>(humi - MIN_HUMIDITY * part + FAN_MINIMUM_DUTY_CYCLE);
    // 61 - 54 * 0.95 + 6;

    return speed; // 12
}

void loop()
{
    // time marker
    const unsigned long currentMillis = millis();

    if (sensor.measure())
    {
        const float temperature = sensor.temperature();
        const float humidity = sensor.humidity();
        const long speed = mySpeed(humidity);

        // serial output
        Serial.print("Temperature: ");
        Serial.print(temperature, 1);
        Serial.print(" *C\tHumidity: ");
        Serial.print(humidity, 1);
        Serial.print(" %RH");
        Serial.println();

        // TEMPERATURE IS HIGH COMPRESSOR RELAY ON
        if (temperature >= MAX_TEMPERATURE)
        {
            // start compressor
            digitalWrite(COOLING_COMPRESSOR_PIN, HIGH);
            Serial.println("Kühlung ON: ");
            lcd.setCursor(0, 0);
            lcd.print("Kuehlung ON");
            lcd.setCursor(0, 1);
            lcd.print(String("Temperatur: ") + String(temperature));
            delay(3000);
            lcd.clear();
            // Fan circulation on
            if (!fanOn)
            {
                fanOn = true;
                circulation(speed);
            }
        }

        // TEMPERATUR IS COOLING DOWN
        if (temperature <= MIN_TEMPERATURE)
        {
            // show important massage
            Serial.println("Kühlung ON: ");
            lcd.setCursor(0, 0);
            lcd.print("Kuehlung ON");
            lcd.setCursor(0, 1);
            lcd.print(String("Temperatur: ") + String(temperature));
            delay(3000);
            lcd.clear();
            // Fan circulation on
            if (!fanOn)
            {
                fanOn = true;
                circulation(speed);
            }
        }

        // TEMPERATURE IS LOW COMPRESSOR RELAY OFF
        if (temperature <= MIN_TEMPERATURE)
        {
            // stop compressor
            digitalWrite(COOLING_COMPRESSOR_PIN, LOW);
            Serial.println("Kühlung OFF: ");
            lcd.setCursor(0, 0);
            lcd.print("Kuehlung OFF");
            lcd.setCursor(0, 1);
            lcd.print(String("Temperatur: ") + String(temperature));
            delay(3000);
            lcd.clear();
        }

        circulation(speed); // this funktion write to serial
        lcd.setCursor(0, 0);
        lcd.print(String("Humidity: ") + String(humidity));
        lcd.setCursor(0, 1);
        lcd.print("Fan Speed:" + String(speed));
        delay(3000);
        lcd.clear();

        // set the time
        previousMillis = currentMillis;
    }
    else
    {
        // sensor connection error run fan by default low
        fanOn = true;
        circulation(FAN_MINIMUM_DUTY_CYCLE);
        // print out error message
        Serial.println("SHT45 read error");
        lcd.setCursor(0, 0);
        lcd.print("SHT45 sensor read error");
        delay(5000);
    }
}

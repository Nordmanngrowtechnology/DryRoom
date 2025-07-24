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
constexpr uint8_t FAN_MINIMUM_DUTY_CYCLE = 6;  // The lowest PWM Duty Cycle the fank can look
                                               // in datasheet for your fan
constexpr uint8_t FAN_MAXIMAL_DUTY_CYCLE = 50; // Maximal Fan Speed
constexpr uint8_t CIRCULATION_SPREAD = 5;     // Circulation spread between pwm groups top fan bottom fan
bool fanOn = false;

// INT CONF
unsigned long previousMillis = 0;
unsigned long temp_interval = 30000; // 30 second temp_interval for checkup relay off?
unsigned long dry_interval = 180000; // 3 Minutes 180000ms

// debug function uncomment for debug
bool checkup = false;
// #define DEBUG_PWM true
#ifdef DEBUG_PWM
long i = 0;
long timer_cycles = 0;
long Frequency = 0;

bool pwmSignalDebug(const long i)
{
    // arduino nano 16MHz
    Serial.println("PWM Signal Debug Output Nr: " + String(i));
    Serial.println("#############################");

    // show pin
    Serial.print("PWM Connected to pin: ");
    Serial.print(FAN_PWM_PIN_TOP_FAN.pwmPin);
    Serial.print(" and ");
    Serial.println(FAN_PWM_PIN_BOTTOM_FAN.pwmPin);
    Serial.println("----------------------");

    // test calculation of duty cycle
    OCR1A = map(100, 0, 100, 0, ICR1);

    // top
    Serial.print("ICR1: ");
    Serial.println(ICR1);
    Serial.println("----------------------");
    // duty cycle
    Serial.print("OCR1A: ");
    Serial.println(OCR1A);
    Serial.println("----------------------");

    // register timer1
    Serial.print("Timer1 TCNT1: ");
    Serial.print(TCNT1);
    Serial.println(" Count before overflow");
    Serial.println("----------------------");
    // register
    Serial.print("TCCR1A: ");
    Serial.println(TCCR1A);
    Serial.println("----------------------");
    // register
    Serial.print("TCCR1B: ");
    Serial.println(TCCR1B);
    Serial.println("----------------------");
    // register
    Serial.print("Register Setting COM1A0: ");
    Serial.println(COM1A0);
    Serial.println("----------------------");

    // show frequency
    Serial.print("PWM Signal Frequency: ");
    timer_cycles = ICR1 + 1;
    Frequency = 16000000 / (1 * timer_cycles) / 2;
    Serial.print(Frequency);
    Serial.println(" Hz");
    Serial.println("----------------------");

    delay(5000);
    return true;
}
#endif

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
#ifndef DEBUG_PWM
    Wire.begin();
    while (!sensor.begin())
    {
        Serial.println("SHT45 notfound!");
        // add error to display
        lcd.print("SHT45 notfound!");
        delay(2000);
    }
#endif
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
    if (percent <= FAN_MINIMUM_DUTY_CYCLE)
    { // stop fan
        analogWrite(FAN_PWM_PIN_TOP_FAN, 0);
        analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 0);
        Serial.println("Stop fan circulation.");
        return;
    }

    if (percent - CIRCULATION_SPREAD <= FAN_MINIMUM_DUTY_CYCLE)
    { // minimum circulation
        analogWrite(FAN_PWM_PIN_TOP_FAN, FAN_MINIMUM_DUTY_CYCLE);
        analogWrite(FAN_PWM_PIN_BOTTOM_FAN, FAN_MINIMUM_DUTY_CYCLE);
        Serial.println("Fan circulation spread value to big: Fan run default speed.");
        return;
    }

    // calculated circulation
    analogWrite(FAN_PWM_PIN_TOP_FAN, percent - CIRCULATION_SPREAD);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, percent);

    Serial.print("Fan run ");
    Serial.print(percent);
    Serial.println("%");
}

// calculate the percent speed for todo validate types and resolution
uint8_t mySpeed(const float humi)
{

    if (humi <= MIN_HUMIDITY)
    {
        return FAN_MINIMUM_DUTY_CYCLE;
    }

    constexpr float h_range = 100 - MIN_HUMIDITY;                   // 46
    constexpr int s_range = FAN_MAXIMAL_DUTY_CYCLE - FAN_MINIMUM_DUTY_CYCLE; // 44

    constexpr float part = s_range / h_range;

    const auto speed = static_cast<uint8_t>(humi - MIN_HUMIDITY * part + FAN_MINIMUM_DUTY_CYCLE);
    // 61 - 54 * 0.95 + 6;

    return speed; // 12
}

// system boot test
bool startUpCheck()
{
    // display message
    lcd.setCursor(0, 0);
    lcd.print("Startup Check");
    analogWrite(FAN_PWM_PIN_TOP_FAN, 100);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 10);
    Serial.println("Fan run 100% and 10%");
    delay(20000);
    analogWrite(FAN_PWM_PIN_TOP_FAN, 10);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 100);
    Serial.println("Fan run 10% and 100%");
    delay(20000);
    analogWrite(FAN_PWM_PIN_TOP_FAN, 50);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 50);
    Serial.println("Fan run 50%");
    delay(10000);
    analogWrite(FAN_PWM_PIN_TOP_FAN, 30);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 30);
    Serial.println("Fan run 30%");
    delay(10000);
    analogWrite(FAN_PWM_PIN_TOP_FAN, 5);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 5);
    Serial.println("Fan run 5%");
    delay(10000);
    analogWrite(FAN_PWM_PIN_TOP_FAN, 100);
    analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 100);
    Serial.println("Fast run 100%");
    delay(10000);
    lcd.clear();
    return true;
}

void loop()
{
#ifdef DEBUG_PWM
    i++;
    pwmSignalDebug(i);

    // Boot up fan check
    if (checkup == false)
    {
        checkup = startUpCheck();
    }
#endif

#ifndef DEBUG_PWM
    // if the sensor connected
    if (sensor.measure())
    {
        // serial debug message
        Serial.print("Temperature: ");
        Serial.print(sensor.temperature(), 1);
        Serial.print(" *C\tHumidity: ");
        Serial.print(sensor.humidity(), 1);
        Serial.print(" %RH");
        Serial.println();

        // time marker
        const unsigned long currentMillis = millis();
        const float temperature = sensor.temperature();
        const float humidity = sensor.humidity();

        // HANDLE TEMPERATURE
        //  is HIGH
        if (temperature >= MAX_TEMPERATURE)
        {
            digitalWrite(COOLING_COMPRESSOR_PIN, HIGH);
        }

        if (currentMillis - previousMillis >= temp_interval)
        {
            lcd.clear();

            if (temperature >= MAX_TEMPERATURE)
            {
                // start compressor
                digitalWrite(COOLING_COMPRESSOR_PIN, HIGH);
                // display message
                lcd.setCursor(0, 0);
                lcd.print("Kuehlung ON");
                lcd.setCursor(0, 1);
                lcd.print(String("Temperatur: ") + String(temperature));
                delay(1000);
                lcd.clear();
                // Fan circulation on
                if (!fanOn)
                {
                    fanOn = true;
                    circulation(FAN_MINIMUM_DUTY_CYCLE);
                }
            }
            else
            {
                // stop compressor
                digitalWrite(COOLING_COMPRESSOR_PIN, LOW);
                // display message
                lcd.setCursor(0, 0);
                lcd.print("Kuehlung OFF");
                lcd.setCursor(0, 1);
                lcd.print(String("Temperatur: ") + String(temperature));
                delay(1000);
                lcd.clear();
            }
        }

        // set new time
        previousMillis = currentMillis;

        // it is wet
        if (humidity >= MIN_HUMIDITY)
        {
            fanOn = true;
            const long speed = mySpeed(humidity);
            circulation(speed);
            lcd.setCursor(0, 0);
            lcd.print(String("Humidity: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Fan Speed:" + String(speed));
            delay(5000);
            lcd.clear();
        }

        // it is too dry interval circulation
        if (humidity < MIN_HUMIDITY and currentMillis - previousMillis >= dry_interval)
        {
            circulation(FAN_MINIMUM_DUTY_CYCLE);
            lcd.setCursor(0, 0);
            lcd.print(String("Humidity: ") + String(humidity));
            lcd.setCursor(0, 1);
            lcd.print("Fan Speed:" + String(FAN_MINIMUM_DUTY_CYCLE));
            delay(30000); // 30sec
            lcd.clear();
            fanOn = false;
            analogWrite(FAN_PWM_PIN_TOP_FAN, 0);
            analogWrite(FAN_PWM_PIN_BOTTOM_FAN, 0);
        }
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
#endif
}

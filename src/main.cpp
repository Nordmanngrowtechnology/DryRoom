// arduino lib
#include <Arduino.h>

// >>> ### --- SHT45 TEMP & HUMIDY --- ### >>>
// Cable black = SHIELD on Case
// Cable red = VDD 5V PIN_5V
// Cable green = SCL PIN_19  **A5
// Cable yellow = SDA PIN_18 **A4
// Cable black = GND PIN_GND
#include <Wire.h>
#include <ArtronShop_SHT45.h>
ArtronShop_SHT45 sensor(&Wire, 0x44);  // default is right (&Wire, 0x44) SHT45-AD1B => I2C address 0x44
float temperature;
float humidity;

// >>> ### --- DISPLAY --- ### >>>
#include <LiquidCrystal_I2C.h>       //Include lib
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for 16 chars and 2 line display


// >>> ### --- COOLING_COMPRESSOR RELAY --- ### >>>
constexpr int COOLING_COMPRESSOR_PIN = 9; // Signal to SPDT, 30 A
#include "Relay.h"
Relay cooling_compressor(COOLING_COMPRESSOR_PIN, true);  // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed


// >>> ### --- PWM CASE FAN 25 kHz --- ### >>>
 struct Pin{
  uint8_t pwmPin;
};
constexpr Pin FAN_PWM_PIN_A {3};
constexpr Pin FAN_PWM_PIN_B  {11};
constexpr int DEFAULT_SPEED = 5;
constexpr int MAX_TEMPERATURE = 18;
constexpr int MAX_HUMIDITY = 58;
constexpr int MED_HUMIDITY = 55;



// overwrite default analogWrite function for pwm fan pins
void analogWrite(const Pin &pin, const uint8_t percent) {
  if (pin.pwmPin == FAN_PWM_PIN_A.pwmPin || pin.pwmPin == FAN_PWM_PIN_B.pwmPin) OCR1A = map(percent, 0, 100, 0, ICR1);
}

byte textLen;
void setup() {

  // reset the atmega timer on nano
  TCNT1 = 0;                                        // Count Increment or decrement TCNT1 by 1. set to 0
  TCCR1A = 0; TCCR1B = 0;                           // TCCR1 register to 0
  ICR1  = 320;                                      // counter limiter
  TCCR1B |= (1 << WGM13); TCCR1A |= (1 << WGM11);   // 16.4 Timer/Counter Clock Sources Register B (TCCR1B)
  TCCR1A |= (1 << COM1A1);                          // 16.2.1 Registers
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

  // Relays intit the pins
  cooling_compressor.begin();
  //humy_circulation.begin();
}
void loop() {

  // Fan circulation low on
  analogWrite(FAN_PWM_PIN_A, 10);
  analogWrite(FAN_PWM_PIN_B, 13);

  // test SHT45
  if (sensor.measure()) {
    Serial.print("Temperature: ");
    Serial.print(sensor.temperature(), 1);
    Serial.print(" *C\tHumidity: ");
    Serial.print(sensor.humidity(), 1);
    Serial.print(" %RH");
    Serial.println();

    // HANDLE TEMPERATURE
    temperature = sensor.temperature();

    // temperature is HIGH
    if (temperature > MAX_TEMPERATURE) {
      // start compressor
      cooling_compressor.turnOn();
      lcd.setCursor(0, 0);
      lcd.print("Die Kuehlung ist eingeschaltet");
      lcd.setCursor(0, 1);
      lcd.print(String("Temparatur: ") + String(temperature));

      // Fan circulation on
      analogWrite(FAN_PWM_PIN_A, 50);
      analogWrite(FAN_PWM_PIN_B, 40);
    } else {
      // stop compressor
      cooling_compressor.turnOff();
      // show temp on display
    lcd.setCursor(0, 1);
    lcd.print(String("Temparatur: ") + String(temperature));
    }

    delay(10000); // 10 second to next


    // HANDLE HUMIDITY
    humidity = sensor.humidity();

    if (humidity >= MAX_HUMIDITY) {
      // MAX_HUMIDITY: Fast fan circulation it is real wet
      analogWrite(FAN_PWM_PIN_A, 80);
      analogWrite(FAN_PWM_PIN_B, 70);
      // massage output
      lcd.setCursor(0, 0);
      lcd.print(String("Sehr feuchte Luft: ") + String(humidity));
      lcd.setCursor(0, 1);
      lcd.print("Luefter Cyrculation sehr schnell!");
    }
    else if (humidity > MED_HUMIDITY) {
      // MED_HUMIDITY: Moderate fan circulation
      analogWrite(FAN_PWM_PIN_A, 50);
      analogWrite(FAN_PWM_PIN_B, 40);
      // massage output
      lcd.setCursor(0, 0);
      lcd.print(String("Die Luftfeuchtigkeit ist: ") + String(humidity));
      lcd.setCursor(0, 1);
      lcd.print("Lüfter laufen langsam.");
    }else{
    // DEFAULT: Slow fan cyrculation
    analogWrite(FAN_PWM_PIN_A, 10);
    analogWrite(FAN_PWM_PIN_B, 13);
    // massage output
      lcd.setCursor(0, 0);
      lcd.print(String("Die Luftfeuchtigkeit ist: ") + String(humidity));
      lcd.setCursor(0, 1);
      lcd.print("Luefter laufen langsam.");
    }

    delay(10000); // 10 second to next


  
  } else {  //SHT read error
    Serial.println("SHT45 read error");
    lcd.setCursor(0, 0);
    lcd.print("SHT45 sensor read error");
  }
}
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
ArtronShop_SHT45 sht45(&Wire, 0x44);  // SHT45-AD1B => I2C adress 0x44
float temp;
float humidity;

// >>> ### --- DISPLAY --- ### >>>
#include <LiquidCrystal_I2C.h>       //Include lib
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


// >>> ### --- REALY --- ### >>>
constexpr int RELAY_FREZZER_PIN = 9; // Signal to SPDT, 30 A
//constexpr int RELAY_CIRCULATION_PIN = 15;
#include "Relay.h"
Relay frezzer_motor(RELAY_FREZZER_PIN, true);  // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
//Relay humy_circulation(RELAY_CIRCULATION_PIN, true);  // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed

// >>> ### --- PWM CASE FAN 25 kHz --- ### >>>
constexpr int PWM_FAN_TOP = 3;
constexpr int PWM_FAN_BOTTOM = 5;
constexpr int fan;
constexpr int speed;

// overwrite default analogWrite function for pwm fan pins
void analogWrite(const Pin &nPin, uint8_t percent) {
  if (nPin == PWM_FAN_TOP || nPin == PWM_FAN_BOTTOM) OCR1A = map(percent, 0, 100, 0, ICR1);
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
  while (!sht45.begin()) {
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
  frezzer_motor.begin();
  //humy_circulation.begin();
}
void loop() {

  // Fan circulation low on
  fanSpeed(PWM_FAN_TOP, 10);
  fanSpeed(PWM_FAN_BOTTOM, 13);

  // test SHT45
  if (sht45.measure()) {
    Serial.print("Temperature: ");
    Serial.print(sht45.temperature(), 1);
    Serial.print(" *C\tHumidity: ");
    Serial.print(sht45.humidity(), 1);
    Serial.print(" %RH");
    Serial.println();

    // HANDLE TEMPARATUR
    temp = sht45.temperature();

    // temparature to hige
    if (temp > 18) {
      // run kompressor
      frezzer_motor.turnOn();
      lcd.setCursor(0, 0);
      lcd.print("Die Kuehlung ist eingeschaltet");
      lcd.setCursor(0, 1);
      lcd.print(String("Temparatur: ") + String(temp));

      // Fan circulation on
      fanSpeed(PWM_FAN_TOP, 50);
      fanSpeed(PWM_FAN_BOTTOM, 40);
    } else {
      // frezzer relay off
      frezzer_motor.turnOff();
      // show temp on display
    lcd.setCursor(0, 1);
    lcd.print(String("Temparatur: ") + String(temp));
    }

    delay(10000); // 10 second to next


    // HANDLE HUMIDITY
    humidity = sht45.humidity();

    if (humidity >= 58) {
      // WET: Fast fan circulation
      fanSpeed(PWM_FAN_TOP, 80);
      fanSpeed(PWM_FAN_BOTTOM, 70);
      // massage output
      lcd.setCursor(0, 0);
      lcd.print(String("Sehr feuchte Luft: ") + String(humidity));
      lcd.setCursor(0, 1);
      lcd.print("Luefter Cyrculation sehr schnell!");
    }
    else if (humidity > 55){
      // NORMAL: Modarate fan circulation
      fanSpeed(PWM_FAN_TOP, 50);
      fanSpeed(PWM_FAN_BOTTOM, 40);
      // massage output
      lcd.setCursor(0, 0);
      lcd.print(String("Die Luftfeuchtigkeit ist: ") + String(humidity));
      lcd.setCursor(0, 1);
      lcd.print("Lüfter laufen langsam.");
    }else{
    // DEFAULT: Slow fan cyrculation
    fanSpeed(PWM_FAN_TOP, 10);
    fanSpeed(PWM_FAN_BOTTOM, 13);
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
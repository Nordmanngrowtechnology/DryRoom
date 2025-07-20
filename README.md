# DryRoom Controller with Arduino Nano ATmega328 (New Bootloader)
https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328new.html
- Arduino Nano
- Atmega328P 16Mhz
- USB-C

## Forum Post
[https://forum.arduino.cc/t/kleine-klimasteuerung-fragen-bevor-es-durchbrennt/1393747](https://forum.arduino.cc/t/kleine-klimasteuerung-fragen-bevor-es-durchbrennt/1393747)

![Trocknung_ArduinoNano_Steckplatine.png](fritzing/Trocknung_ArduinoNano_Steckplatine.png)

# Specification PWM Case Fan
### PWM Signal specification 25 kHz best patrice case fan

```
void setup() {
    cli(); // disable interrupts

    /*
     *  DEFINITION TIMER1 FOR ATMEL ATmega328P
     *  https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     */

    // RESET
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0; // defined timer & register to 0

    // TOP
    ICR1 = 320; // Set TOP limiter to 320 >>> update OCR1A at TOP

    // REGISTER DEFINITION
    TCCR1B |= (1 << WGM13); // The phase correct Pulse Width Modulation or phase correct PWM mode (WGM13:0 = 1
    TCCR1A |= (1 << WGM11); // PWM, Phase Correct
    TCCR1A |= (1 << COM1A1); // Clear OC1A/OC1B on Compare match when down counting set output to low level
    TCCR1B |= (1 << CS10); // clkI/O/1 (No prescaling)

    sei(); // allow interrupts
```

## Attention
Minimum fan PWM signal in my case 5% duty cycle. Use Paired Timer TC1 on pin 3 & 11


## Hardware to test the dry room controller

Important is we use Timer1 >> and the located output to OC1A, OC1B by Arduino Nano with ATmega328P. 

| Pin | Method         | Hard.                             | More.    |
|-----|----------------|-----------------------------------|----------|
| 10  | analogWrite()  | Top FAN                           | PWM      |
| 9   | analogWrite()  | Bottom FAN                        | PWM      |
| 18  | readBytes()    | SHT 45                            | SDA 0x44 |
| 19  | readBytes()    | SHT 45                            | SCL 0x44 |
| 18  | readBytes()    | Display 16*2                      | SDA 0x27 |
| 19  | readBytes()    | Display 16*2                      | SCL 0x27 |
| 14  | readAnalog()   | FAN-Tacho Top                     |          |
| 15  | readAnalog()   | FAN-Tacho Bottom                  |          |
| 5   | digitalWrite() | Relay SPDT, 30 A, SLA-05VDC-SL-C  |          |

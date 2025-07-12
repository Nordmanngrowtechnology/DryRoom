# Test Arduino Nano (Old Bootloader)


![Trocknung_ArduinoNano_Steckplatine.png](fritzing/Trocknung_ArduinoNano_Steckplatine.png)

# PWM Signal specification 25 kHz best patrice case fan

How to page 134 16.9.4, 135 Phase Correct PWM
https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf


```
setup()
// rest the atmega timer
TCNT1 = 0;                                        // Count Increment or decrement TCNT1 by 1. set to 0
TCCR1A = 0; TCCR1B = 0;                           // TCCR1 register to 0
ICR1  = 320;                                      // counter limiter
TCCR1B |= (1 << WGM13); TCCR1A |= (1 << WGM11);   // 16.4 Timer/Counter Clock Sources Register B (TCCR1B)
TCCR1A |= (1 << COM1A1);                          // 16.2.1 Registers
TCCR1B |= (1 << CS10);
```

# Test as dry room controller

- 1x Relay SPDT, 30 A, SLA-05VDC-SL-C
- 1x LCD I2C 2*16
- 1x SHT45 Sensor
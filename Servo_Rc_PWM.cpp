/*
  This library was written only for "Arduino Mega 1280" and "Arduino Mega 2560". It does not support other cards.
  Bu kütüphane sadece "Arduino Mega 1280" ve "Arduino Mega 2560" için yazılmıştır. Diğer kartları desteklemez.
*/

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include <Arduino.h>
#include <Servo_Rc_PWM.h>

Servo::Servo() {
  this->servoPin = 0;
}

uint8_t Servo::attach(int pin) {
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, DEFAULT_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max) {
  return this->attach(pin, min, max, DEFAULT_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max, int defaultPos) {
  this->min = min;
  this->max = max;
  this->defaultPos = defaultPos * 2;

  if (pin == 2 || pin == 3 || pin == 5)
  {
    //resetting the control register A, B, C:
    TCCR3A = 0x0;
    TCCR3B = 0x0;
    TCCR3C = 0x0;

    //setting the prescaler to 8 (2MHz):
    TCCR3B |= (1 << CS31);

    //setting the waveform generation mode to 14:
    TCCR3A |= (1 << WGM31) | (0 << WGM30);
    TCCR3B |= (1 << WGM32) | (1 << WGM33);

    //setting the TOP value:
    ICR3 = MAX_TIMER_COUNT; //results in 50Hz at 2MHz Clock

    if (pin == 2) {
      //setting the output to non inverted:
      TCCR3A |= (1 << COM3B1);

      OCR3B = this->defaultPos; //setting the pulse width

      //OC3B, Port E, Bit 4; setting pin 2 as output:
      DDRE |= (1 << PE4); //bit 4 (pin 2) as output
    }
    if (pin == 3) {
      //setting the output to non inverted:
      TCCR3A |= (1 << COM3C1);

      OCR3C = this->defaultPos; //setting the pulse width

      //OC3C, Port E, Bit 5; setting pin 3 as output:
      DDRE |= (1 << PE5); //bit 5 (pin 3) as output
    }

    if (pin == 5) {

      //setting the output to non inverted:
      TCCR3A |= (1 << COM3A1);

      OCR3A = this->defaultPos; //setting the pulse width

      //OC3A, Port E, Bit 3; setting pin 5 as output:
      DDRE |= (1 << PE3); //bit 3 (pin 5) as output
    }

    this->servoPin = pin;
  }

  else if (pin == 6 || pin == 7 || pin == 8)
  {
    //resetting the control register A, B, C:
    TCCR4A = 0x0;
    TCCR4B = 0x0;
    TCCR4C = 0x0;

    //setting the prescaler to 8 (2MHz):
    TCCR4B |= (1 << CS41);

    //setting the waveform generation mode to 14:
    TCCR4A |= (1 << WGM41) | (0 << WGM40);
    TCCR4B |= (1 << WGM42) | (1 << WGM43);

    //setting the TOP value:
    ICR4 = MAX_TIMER_COUNT; //results in 50Hz at 2MHz Clock

    if (pin == 6) {
      //setting the output to non inverted:
      TCCR4A |= (1 << COM4A1);

      OCR4A = this->defaultPos; //setting the pulse width

      //OC4A, Port H, Bit 3; setting pin 6 as output:
      DDRH |= (1 << PH3); //bit 3 (pin 6) as output
    }

    if (pin == 7) {
      //setting the output to non inverted:
      TCCR4A |= (1 << COM4B1);

      OCR4B = this->defaultPos; //setting the pulse width

      //OC4B, Port H, Bit 4; setting pin 7 as output:
      DDRH |= (1 << PH4); //bit 4 (pin 7) as output
    }

    if (pin == 8) {
      //setting the output to non inverted:
      TCCR4A |= (1 << COM4C1);

      OCR4C = this->defaultPos; //setting the pulse width

      //OC4C, Port H, Bit 5; setting pin 8 as output:
      DDRH |= (1 << PH5); //bit 5 (pin 8) as output
    }
    this->servoPin = pin;
  }


  else if (pin == 11 || pin == 12 || pin == 13)
  {
    //resetting the control register A, B, C:
    TCCR1A = 0x0;
    TCCR1B = 0x0;
    TCCR1C = 0x0;

    //setting the prescaler to 8 (2MHz):
    TCCR1B |= (1 << CS11);

    //setting the waveform generation mode to 14:
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    //setting the TOP value:
    ICR1 = MAX_TIMER_COUNT; //results in 50Hz at 2MHz Clock

    if (pin == 11) {
      //setting the output to non inverted:
      TCCR1A |= (1 << COM1A1);

      OCR1A = this->defaultPos; //setting the pulse width

      //OC1A, Port B, Bit 5; setting pin 11 as output:
      DDRB |= (1 << PB5); // (pin 11) as output
    }

    if (pin == 12) {
      //setting the output to non inverted:
      TCCR1A |= (1 << COM1B1);

      OCR1B = this->defaultPos; //setting the pulse width

      //OC1B, Port L, Bit 6; setting pin 12 as output:
      DDRB |= (1 << PB6); //(pin 12) as output
    }
    if (pin == 13) {
      //setting the output to non inverted:
      TCCR1A |= (1 << COM1C1);

      OCR1C = this->defaultPos; //setting the pulse width

      //OC1C, Port B, Bit 7; setting pin 13 as output:
      DDRB |= (1 << PB7); // (pin 13) as output
    }
    this->servoPin = pin;
  }
  else if (pin == 44 || pin == 45 || pin == 46)
  {
    //resetting the control register A, B, C:
    TCCR5A = 0x0;
    TCCR5B = 0x0;
    TCCR5C = 0x0;

    //setting the prescaler to 8 (2MHz):
    TCCR5B |= (1 << CS51);

    //setting the waveform generation mode to 14:
    TCCR5A |= (1 << WGM51) | (0 << WGM50);
    TCCR5B |= (1 << WGM52) | (1 << WGM53);

    //setting the TOP value:
    ICR5 = MAX_TIMER_COUNT; //results in 50Hz at 2MHz Clock

    if (pin == 44) {
      //setting the output to non inverted:
      TCCR5A |= (1 << COM5C1);

      OCR5C = this->defaultPos; //setting the pulse width

      //OC5C, Port L, Bit 5; setting pin 44 as output:
      DDRL |= (1 << PL5); //bit 5 (pin 44) as output
    }

    if (pin == 45) {
      //setting the output to non inverted:
      TCCR5A |= (1 << COM5B1);

      OCR5B = this->defaultPos; //setting the pulse width

      //OC5B, Port L, Bit 4; setting pin 45 as output:
      DDRL |= (1 << PL4); //bit 4 (pin 45) as output
    }
    if (pin == 46) {
      //setting the output to non inverted:
      TCCR5A |= (1 << COM5A1);

      OCR5A = this->defaultPos; //setting the pulse width

      //OC5A, Port L, Bit 3; setting pin 46 as output:
      DDRL |= (1 << PL3); //bit 3 (pin 46) as output
    }
    this->servoPin = pin;
  }
}


void Servo::write(int value) {
  float angleValue;

  if (value <= 0)
  {
    angleValue = 0.0;
  }
  else if (value >= 180)
  {
    angleValue = 180.0;
  }
  else
  {
    angleValue = (float)value;
  }
  angleValue = (((this->max - this->min) * angleValue) / 180.0) + this->min;
  value = (int)angleValue;
  this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value) {
  if (value < this->min)
  {
    value = this->min;
  }
  else if (value > this->max)
  {
    value = this->max;
  }
  this->pulseWidth = value * 2;

  if (this->servoPin == 2)
  {
    OCR3B = 0x0;
    OCR3B = this->pulseWidth;
  }
  else if (this->servoPin == 3)
  {
    OCR3C = 0x0;
    OCR3C = this->pulseWidth;
  }
  else if (this->servoPin == 5)
  {
    OCR3A = 0x0;
    OCR3A = this->pulseWidth;
  }
  else if (this->servoPin == 6)
  {
    OCR4A = 0x0;
    OCR4A = this->pulseWidth;
  }
  else if (this->servoPin == 7)
  {
    OCR4B = 0x0;
    OCR4B = this->pulseWidth;
  }
  else if (this->servoPin == 8)
  {
    OCR4C = 0x0;
    OCR4C = this->pulseWidth;
  }
  else if (this->servoPin == 11)
  {
    OCR1A = 0x0;
    OCR1A = this->pulseWidth;
  }
  else if (this->servoPin == 12)
  {
    OCR1B = 0x0;
    OCR1B = this->pulseWidth;
  }
  else if (this->servoPin == 13)
  {
    OCR1C = 0x0;
    OCR1C = this->pulseWidth;
  }
  else if (this->servoPin == 44)
  {
    OCR5C = 0x0;
    OCR5C = this->pulseWidth;
  }
  else if (this->servoPin == 45)
  {
    OCR5B = 0x0;
    OCR5B = this->pulseWidth;
  }
  else if (this->servoPin == 46)
  {
    OCR5A = 0x0;
    OCR5A = this->pulseWidth;
  }
}

int Servo::read() {
  float angle;

  if ((this->readMicroseconds() - this->min) <= 0)
  {
    angle = 0.0;
  }
  else
  {
    angle = (180.0 / (this->max - this->min)) * (this->readMicroseconds() - this->min);
  }
  return (int)angle;
}

int Servo::readMicroseconds() {
  return this->pulseWidth;
}

#endif

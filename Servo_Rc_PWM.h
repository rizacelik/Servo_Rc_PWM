#ifndef Servo_Rc_PWM_h
#define Servo_Rc_PWM_h

#include <inttypes.h>

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
#error "ERROR: This library only supports boards with an ATmega1280 or ATmega2560 processor. (Arduino/Genuino Mega/Mega1280/Mega2560)"
#endif

#define Servo_VERSION       1.3.0  // software version of this library

#define MIN_PULSE_WIDTH       500 // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500 // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500 // default pulse width when servo is attached
#define MAX_TIMER_COUNT   40000   // the timer TOP value (for creating 50Hz)

class Servo {
  public:
    Servo();
    uint8_t attach(int pin);
    uint8_t attach(int pin, int min, int max);
    uint8_t attach(int pin, int min, int max, int defaultPos);

    void write(int value);               // write angle in degrees
    void writeMicroseconds(int value);   // write pulse width in microseconds
    int read();                          // returns the current write angle in degrees
    int readMicroseconds();              // returns the current write angle in microseconds

  private:
    uint8_t servoPin;                   // pin number of the attached Servo
    uint16_t min;                       // lower pulse width limit
    uint16_t max;                       // upper pulse width limit
    uint16_t defaultPos;                // pulse width when servo is attached
    uint16_t pulseWidth;                // set pulse width
};

#endif

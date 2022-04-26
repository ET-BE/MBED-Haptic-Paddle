#ifndef MOTOR_H_
#define MOTOR_H_

#include <mbed.h>
#include "FastPWM/FastPWM.h"

/**
 * Basic motor class to combine PWM and direction pin.
 *
 * Use FastPWM (not the default PwmOut) to prevent resetting the duty
 * cycle phase when writing a new PWM value.
 */
class Motor {

public:

    /**
     * Constructor.
     */
    Motor(PinName pin_pwm, PinName pin_dir);

    /**
     * Set motor speed.
     *
     * @param speed     Target speed [-1, 1], output will be clamped
     */
    void set(float speed);

private:
    FastPWM pwm_;
    DigitalOut dir_;
};

#endif

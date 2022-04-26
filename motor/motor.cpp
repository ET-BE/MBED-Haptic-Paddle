#include "motor.h"

Motor::Motor(PinName pin_pwm, PinName pin_dir) :
    pwm_(pin_pwm), dir_(pin_dir) {

    pwm_.write(0.0f);
    pwm_.period_us(60); // 60 us ~ 17 kHz
    dir_.write(0);
}

void Motor::set(float speed) {

    bool direction = (speed < 0.0f);

    dir_.write(direction);

    speed = abs(speed);
    if (speed > 1.0f) {
        speed = 1.0f;
    }

    pwm_.write(speed); // pwm takes input from from 0.0f to 100.0f
}

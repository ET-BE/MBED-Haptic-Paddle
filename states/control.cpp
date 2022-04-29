#include <mbed.h>
#include <cstdint>
#include <ctime>

#include "control.h"

BufferedSerial Control::console(STDIO_UART_TX, STDIO_UART_RX, 115200);

// Constructor
Control::Control(float Fs) {

    Fs_ = Fs;

    pos_ = vel_ = 0.0f;
    torque_ = 0.0f;

    offset_motor_rev_ = 0.0f;

    max_pwm_ = 0.2f;

    // Use pointers so we don't have to use the initialiser list
    // of this class, and instead use delayed creation

    led_blue_ = new DigitalOut(LED_BLUE);

    button_stop_ = new DigitalIn(SW2);

    motor_ = new Motor(D5, D4);
    encoder_ = new QEI(D12, D13, 4096, QEI::X4_ENCODING);
    loadcell_ = new HX711(D2, D3);

    pid_ = new PID(5.0f, 0.1f, 0.0f, 1.0f / Fs_);
    pid_->setFilter(50.0f);

    dynamics_ = new Dynamics(Fs_);
    // At any time dynamics_ could be deleted and replaced a subclass

    printf("Connecting to uScope...\n");
    ThisThread::sleep_for(1ms); // Let print come through

    scope_ = new HIDScope(4);

    printf("Connected\n");

    printf("Taring loadcell...\n");
    // Scale = sensor counts (int) per Nm
    // (int rangesensor * amplifier gain) / (Vcc * sensitivity)
    // The sensitivy was roughly calibrated
    loadcell_->setScale((8388608.0f * 128.0f) / (5.0f * 31.0f));
    loadcell_->powerUp();
    loadcell_->tare();

    // Print instructions
    printf("\nChange settings by typing: [letter]=[value]\n");
    printf("Use: (current value)\n");
    printf("m - Mass [g.m2] (%.2f)\n", dynamics_->mass);
    printf("d - Damping [Nm/rad/s] (%.2f)\n", dynamics_->damping);
    printf("k - Sprinf stiffness [Nm/rad] (%.2f)\n", dynamics_->stiffness);
    printf("p - Max. motor PWM (%.2f)\n", max_pwm_);
}

// Destructor
Control::~Control() {
    if (led_blue_) delete led_blue_;
    if (motor_) delete motor_;
    if (encoder_) delete encoder_;
    if (loadcell_) delete loadcell_;
    if (scope_) delete scope_;
    if (pid_) delete pid_;
}

bool Control::blink(int dt) const {
    return getStateTime() % (2*dt) > dt;
}

void Control::handle_serial_input() {

    static char c;
    static char input_buffer[32];
    static unsigned int input_i = 0;

    while (console.readable()) {
        console.read(&c, 1);
        
        // If buffer is maxed out or [Enter] was pressed
        if (input_i >= 32 || c == 13) {

            printf("\n");
            if (input_i > 1) {

                input_buffer[input_i] = '\0'; // Insert null terminator

                char var, sep;
                float val;
                sscanf(input_buffer, "%c%c%f", &var, &sep, &val);
                // E.g. "m=0.5"

                if (var == 'm') {
                    if (val > 0.001f) {
                        dynamics_->mass = val;
                        printf("New mass [g.m2]: %.2f\n", val);
                    }
                } else if (var == 'd') {
                    if (val >= 0.0f) {
                        dynamics_->damping = val;
                        printf("New damping: %.2f\n", val);
                    }
                } else if (var == 'k') {
                    if (val >= 0.0f) {
                        dynamics_->stiffness = val;
                        printf("New stiffness: %.2f\n", val);
                    }
                } else if (var == 'p') {
                    if (val >= 0.0f) {
                        max_pwm_ = val;
                        printf("New max. PWM: %.2f\n", val);
                    }
                } else {
                    printf("Unknown command [%c]\n", var);
                }
            }

            input_i = 0;
            break;
        } else {
            console.write(&c, 1); // Echo back
        }

        input_buffer[input_i++] = c;
    }
}

// State selection
void Control::run_state() {

    state_all();

    switch (getState()) {
        case CALIBRATE:
            state_calibrate();
            break;
        case RUN:
            state_run();
            break;
        case IDLE:
            state_idle();
            break;
    }
}

// Run before each state
void Control::state_all() {

    // Blink LED to show we're alive
    led_blue_->write(blink(500));

    static BiquadFilter filter_torque(Fs_, 50.0f, BiquadFilter::TYPE_LOW_PASS);
    static BiquadFilter filter_velocity(Fs_, 30.0f, BiquadFilter::TYPE_LOW_PASS);

    const bool use_torque_filter = true;

    // Convert motor angle to paddle angle (in rad)
    pos_ = (encoder_->getRevolutions() - offset_motor_rev_)
        * 2.0f * M_PI / 4.0f;
    static float old_pos = pos_;

    vel_ = filter_velocity.sample(Fs_ * (pos_ - old_pos));
    old_pos = pos_;

    if (loadcell_->isReady()) {
        torque_ = -1.0f * loadcell_->getUnits();
        // Flip sign so it matches the direction of the encoder
    }
    if (use_torque_filter) {
        torque_ = filter_torque.sample(torque_);
    }

    // Listen for serial input
    handle_serial_input();
}

// ------------- CALIBRATE -------------

void Control::state_calibrate() {

    static bool clockwise;
    static float ref_cw, ref_ccw; // Revolutions of the CW and CCW mechanical limits
    static uint32_t still_time; // Time when the motor stopped moving

    if (isInit()) {
        printf("New state: CALIBRATE\n");

        clockwise = true;

        still_time = getStateTime();
    }

    float pwm = 0.15f; // Enough to move past friction 
    if (!clockwise) {
        pwm *= -1.0f;
    }

    motor_->set(pwm);

    if (fabs(vel_) > 0.1f) {
        still_time = getStateTime();
        // Time won't be updated when the motor stopped moving
    }

    // If the motor hasn't moved in a bit
    if (getStateTime() - still_time > 1000) {

        if (clockwise) {
            ref_cw = encoder_->getRevolutions();
            printf("ref_cw: %.2f\n", ref_cw);
        } else {
            ref_ccw = encoder_->getRevolutions();
            printf("ref_ccw: %.2f\n", ref_ccw);

            setState(RUN);

            offset_motor_rev_ = (ref_cw + ref_ccw) / 2.0f;
        }

        clockwise = !clockwise;
        still_time = getStateTime();
    }
}

// ------------- RUN -------------

void Control::state_run() {

    static bool idle_button_pressed;

    if (isInit()) {
        printf("New state: RUN\n");

        // Reset dynamics to match with the physical setup
        dynamics_->resetPosition(pos_);

        motor_->set(0.0f);

        pid_->reset();

        idle_button_pressed = true;
    }
    
    dynamics_->update(torque_);

    float target = dynamics_->getPosition();

    // Limit position
    if (target < -2.0f) {
        dynamics_->resetPosition(-2.0f);
    } else if (target > 2.0f) {
        dynamics_->resetPosition(2.0f);
    }

    //float t = (float)getStateTime() / 1000.0f;
    //float target = 0.5f * sinf(0.3f * t * 2.0f * M_PI);

    float pwm = pid_->control(target - pos_);

    // Limit PWM
    if (pwm > max_pwm_) {
        pwm = max_pwm_;
    } else if (pwm < -max_pwm_) {
        pwm = -max_pwm_;
    }
    motor_->set(pwm);

    scope_->set(0, target);
    scope_->set(1, pos_);
    scope_->set(2, pwm);
    scope_->set(3, torque_);
    scope_->send();

    // Go to idle
    if (!button_stop_->read()) { // Button is pulled up
        if (idle_button_pressed == false) {
            setState(IDLE);
        }
        idle_button_pressed = true;
    } else {
        idle_button_pressed = false;
    }
}

// ------------- IDLE -------------

void Control::state_idle() {

    static unsigned int run_button_count;

    if (isInit()) {
        printf("New state: IDLE\n");

        run_button_count = 0;
    }

    motor_->set(0.0f);

    // Go back to run mode
    if (!button_stop_->read()) {
        run_button_count++;
    } else {
        if (run_button_count > 0) {
            run_button_count--;
        }
    }

    if (run_button_count > 500) {
        setState(RUN);
    }
}

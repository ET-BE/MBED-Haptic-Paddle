#ifndef CONTROL_H
#define CONTROL_H

#include "state_machine.h"
#include "qei/QEI.h"
#include "motor/motor.h"
#include "hx711/HX711.h"
#include "uscope/scope_hid.h"
#include "biquad-filter/biquad_filter.h"
#include "pid/pid.h"
#include "dynamics/dynamics.h"

/**
 * Main state machine class of this program
 */
class Control : public StateMachine {

public:
    enum State {
        CALIBRATE = 0,
        RUN,
        IDLE,
    };

    /**
     * Constructor
     *
     * @param Fs Loop rate
     */
    Control(float Fs);

    /**
     * Destructor
     */
    ~Control();

    /**
     * Choose the right state function based on current state
     */
    void run_state() override;

    /**
     * Global serial instance.
     *
     * mbed_app.json is also configured to use this.
     * 
     * @see mbed_override_console
     */
    static BufferedSerial console;

private:

    void state_all();
    void state_calibrate();
    void state_run();
    void state_idle();

    /**
     * Read the current buffer and process the textual input
     */
    void handle_serial_input();

    /**
     * Return alternating true and false, based on state time
     *
     * @param dt Period of blink frequency (milliseconds)
     */
    bool blink(int dt) const;

    float Fs_; ///< Loop rate [Hz]

    DigitalOut* led_blue_;
    DigitalIn* button_stop_;

    QEI* encoder_;
    Motor* motor_;
    HX711* loadcell_;

    PID* pid_; ///< Motor position controller

    HIDScope* scope_;

    Dynamics* dynamics_;

    float pos_, vel_; ///< Position and velocity of the paddle[rad], [rad/s]
    float torque_;

    float offset_motor_rev_; ///< Offset (in revolutions) to encoder output so the paddle lines out in the middle
};

#endif // CONTROL_H

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "mbed.h"

/**
 * Abstract class to create state machines
 */
class StateMachine {

public:

    /**
     * Constructor
     * 
     * @param state Initial state
     */
    StateMachine(uint8_t initial_state = 0);

    /**
     * Get current state
     */
    uint8_t getState() const;

    /**
     * Set new state
     * 
     * @param request_state New state
     */
    void setState(uint8_t request_state);

    /**
     * Return true if this is the first iteration of a new state
     */
    bool isInit() const;

    /**
     * Loop state machine once
     * 
     * Should be called continously on a fixed rate.
     */
    void run();

    /**
     * Run the body of a state
     * 
     * All meta stuff is done inside `run`. Override this method to
     * select your own states.
     */
    virtual void run_state() = 0;

    /**
     * Return number of milliseconds in runtime
     */
    static uint32_t getTime();

    /**
     * Return number of milliseconds spent in current state
     */
    uint32_t getStateTime() const;

private:

    uint8_t state; /// Current state
    uint8_t new_state; /// Requested new state
    bool init; /// True if first iteration of a state
    uint32_t state_start_time; /// Runtime when new state started

    /**
     * Change the actual state
     */
    void updateState();
};

#endif // STATE_MACHINE_H

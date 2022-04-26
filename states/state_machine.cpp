#include "state_machine.h"
#include <chrono>

StateMachine::StateMachine(uint8_t initial_state) {
    
    setState(initial_state);
    updateState(); // Trigger initialization
}

uint8_t StateMachine::getState() const {
    return state;
}

void StateMachine::setState(uint8_t request_state) {
    new_state = request_state;
}

void StateMachine::updateState() {

    state = new_state;
    init = true;
    state_start_time = getTime();
}

bool StateMachine::isInit() const {
    return init;
}

uint32_t StateMachine::getTime() {
    
    using namespace std::chrono;
    // Convert time_point to one in microsecond accuracy:
    auto now_ms = time_point_cast<milliseconds>(Kernel::Clock::now());
    return now_ms.time_since_epoch().count();
}

uint32_t StateMachine::getStateTime() const {

    return getTime() - state_start_time;
}

void StateMachine::run() {

    run_state();

    // Reset init flag
    if (init) {
        init = false;
    }

    // Make sure that a state is not changed during execution of a state
    if (state != new_state) {
        updateState();
    }

}

#include "dynamics.h"

Dynamics::Dynamics(float Fs) {
    dt_ = 1.0f / Fs;

    mass_ = 1.0f;
    damping_ = 1.0f;

    q_ = dq_ = ddq_ = 0.0f;
}

void Dynamics::update(float input_torque) {

    ddq_ = computeDynamics(input_torque);

    // Euler integration:
    dq_ += ddq_ * dt_;
    q_ += dq_ * dt_;
}

float Dynamics::computeDynamics(float input_torque) {

    // Mass-damper system
    return (input_torque - damping_ * dq_) / mass_;
}

float Dynamics::getPosition() const {
    return q_;
}

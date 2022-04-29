#include "dynamics.h"

Dynamics::Dynamics(float Fs) {
    dt_ = 1.0f / Fs;

    mass = 1.0f;
    damping = 1.0f;
    stiffness = 0.0f; // Disabled

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
    return (input_torque - stiffness * q_ - damping * dq_) / mass;
}

float Dynamics::getPosition() const {
    return q_;
}

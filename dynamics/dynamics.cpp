#include "dynamics.h"

Dynamics::Dynamics(float Fs) {
    dt_ = 1.0f / Fs;

    mass = 0.5f; // g.m2
    damping = 30.0f;
    stiffness = 50.0f;

    q_ = dq_ = ddq_ = 0.0f;
}

void Dynamics::update(float input_torque) {

    ddq_ = computeDynamics(input_torque);

    // Euler integration:
    dq_ += ddq_ * dt_;
    q_ += dq_ * dt_;
}

float Dynamics::computeDynamics(float input_torque) {

    // Coefficients are not in SI units, scale those first

    // Mass-damper system
    float sum_force = stiffness / 1000.0f * q_ + damping / 1000.0f * dq_;
    return (input_torque - sum_force) / (mass / 1000.0f);
}

float Dynamics::getPosition() const {
    return q_;
}

void Dynamics::resetPosition(float pos) {
    q_ = pos;
    dq_ = 0.0f;
}

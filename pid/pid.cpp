#include "pid.h"

PID::PID(float k_p, float k_d, float k_i, float Ts) {

    _k_p = k_p;
    _k_d = k_d;
    _k_i = k_i;

    _Ts = Ts;    

    _error_filter = nullptr;
}

// Destructor
PID::~PID() {
    if (_error_filter) {
        delete _error_filter;
    }
}

float PID::control(float error) {

    float error_dt = (error - _error_prev) / _Ts;
    
    if (_error_filter) {
        error_dt = _error_filter->sample(error_dt);
    }

    float value = _k_p * error +
        _k_d * error_dt + 
        _k_i * _error_int;

    _error_prev = error;
    _error_int += error * _Ts;

    return value;
}

void PID::setFilter(float Fc, BiquadFilter::TYPE type) {

    if (_error_filter) {
        delete _error_filter;
    }

    if (Fc > 0.0f) {
        _error_filter = new BiquadFilter(1.0f / _Ts, Fc, type);
    }
}
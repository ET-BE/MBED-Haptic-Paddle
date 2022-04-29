#include "../biquad-filter/biquad_filter.h"

/**
 * Class for a simple PID controller.
 *
 * Use `setFilter` to create a PID+ controller.
 */
class PID {

public:
    /**
     * @brief Construct a new PID object
     * 
     * Control is computed with:
     *      k_p * e(t) + k_d * e'(t) + k_i * E(t)
     * 
     * @param k_p Proportional gain
     * @param k_d Derivative gain
     * @param k_i Integral gain
     * @param Ts  Sample rate
     */
    PID(float k_p, float k_d = 0.0, float k_i = 0.0, float Ts = 0.01f);

    ~PID();

    /**
     * @brief Get new control output for error
     * 
     * @param error 
     * @return float 
     */
    float control(float error);

    /**
     * @brief Create filter for error derivative
     *
     * @param fc        Cutoff frequency for filter (Hz)
     * @param type      Filter type
     */
    void setFilter(float Fc, BiquadFilter::TYPE type = BiquadFilter::TYPE_LOW_PASS);

    /** Reset any persistent values */
    void reset();

private:

    float _k_p, _k_d, _k_i; ///< Gains
    float _Ts;

    float _error_prev; ///< Previous error value
    float _error_int; ///< Integrated error

    BiquadFilter* _error_filter; ///< Filter for the error derivative
};
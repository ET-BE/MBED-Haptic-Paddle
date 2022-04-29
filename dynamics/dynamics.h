/**
 * Template class for dynamics implementation.
 *
 * Extend this class and override the `computeDynamics` method to
 * implement it.
 */
class Dynamics {

public:

    /**
     * @brief Construct a new Dynamics object
     * 
     * @param Fs Sample rate [Hz]
     */
    Dynamics(float Fs);

    /**
     * @brief Compute next frame for virtual dynamics
     * 
     * This will also do the integrations.
     * 
     * @param input_torque Torque measured by loadcell [Nm]
     * @return float 
     */
    void update(float input_torque);

    /**
     * @return float Current position state
     */
    float getPosition() const;

    float mass; ///< Virtual mass
    float damping; ///< Virtual damping
    float stiffness; ///< Virtual spring stiffness

protected:

    /**
     * @brief Compute the virtual acceleration
     * 
     * @param input_torque Torque measured by loadcell [Nm]
     * @return float 
     */
    virtual float computeDynamics(float input_torque);

    float dt_; ///< Sample time

    float q_, dq_, ddq_; ///< Virtual states

};

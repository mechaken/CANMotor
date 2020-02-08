/*
  *This software includes the work that is distributed in the Apache License 2.0
  */

#ifndef MOTOR_H
#define MOTOR_H

/** A Motor is abstract base class for moving the motor
 */

class Motor
{
public:
    enum State
    {
        Free,
        CW,  // Clock Wise
        CCW, // Counter Clock Wise
        Brake,

        TotalState,
    };

    enum DutyCycleChangeLevel
    {
        OFF = 0,
        Low = 2,
        Middle = 4,
        High = 6,
        Max = 7,

        TotalDutyCycleChangeLevel,
    };

    enum Control
    {
        SlowDecay,
        MixedDecay,
        FastDecay,

        TotalControl,
    };

    Motor();

    /** Set the duty cycle
    *
    * @param duty_cycle duty cycle to set
    */
    void duty_cycle(float value);

    /** Return the duty cycle
    *
    * @returns
    *    the duty cycle
    */
    float duty_cycle() const;

    /** Set the state
    *
    * @param type state to set
    */
    void state(int type);

    /** Return the state
    *
    * @returns
    *    the state
    */
    int state() const;

    /** Set the rise level
    *
    * @param level rise level to set
    */
    void rise_level(int level);

    /** Return the rise level
    *
    * @returns
    *    the rise level
    */
    int rise_level() const;

    /** Set the fall level
    *
    * @param level fall level to set
    */
    void fall_level(int level);

    /** Return the fall level
    *
    * @returns
    *    the fall level
    */
    int fall_level() const;

    /** Set the pulse period
    *
    * @param pulse period to set
    */
    void pulse_period(float seconds);

    /** Set the pulse frequency
    *
    * @param pulse frequency to set
    */
    void frequency(float hz);

    /** Set the release time(ms)
    *
    * @param release time to set
    */
    void release_time_ms(float ms);

    /** Set the control(LAP, SMB...)
    *
    * @param decay to set
    */
    void control(int value);

    static const int default_state;
    static const int default_duty_cycle_chenge_level;
    static const float default_pulse_period;
    static const float default_frequency;
    static const float defalut_release_time_ms;
    static const int default_control;

protected:
    float _duty_cycle;
    enum State _state;
    enum DutyCycleChangeLevel _rise_level;
    enum DutyCycleChangeLevel _fall_level;
    float _pulse_period;
    float _release_time_ms;
    enum Control _control;
};

#endif

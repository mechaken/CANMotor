/*
  *This software includes the work that is distributed in the Apache License 2.0
  */

#include "Motor.h"

Motor::Motor()
{
    init();
}

void Motor::init()
{
    _duty_cycle = 0.0f;
    _state = (State)default_state;
    _rise_level = (DutyCycleChangeLevel)default_duty_cycle_chenge_level;
    _fall_level = (DutyCycleChangeLevel)default_duty_cycle_chenge_level;
    _pulse_period = default_pulse_period;
    _release_time_ms = defalut_release_time_ms;
    _control = (Control)default_control;
}

void Motor::duty_cycle(float value)
{
    if ((0.00f <= value) && (value <= 1.00f))
    {
        _duty_cycle = value;
    }
}

float Motor::duty_cycle() const
{
    return _duty_cycle;
}

void Motor::state(int type)
{
    if ((0 <= type) && (type < TotalState))
    {
        _state = (State)type;
    }
}

int Motor::state() const
{
    return _state;
}

void Motor::rise_level(int level)
{
    if ((0 <= level) && (level < TotalDutyCycleChangeLevel))
    {
        _rise_level = (DutyCycleChangeLevel)level;
    }
}

int Motor::rise_level() const
{
    return _rise_level;
}

void Motor::fall_level(int level)
{
    if ((0 <= level) && (level < TotalDutyCycleChangeLevel))
    {
        _fall_level = (DutyCycleChangeLevel)level;
    }
}

int Motor::fall_level() const
{
    return _fall_level;
}

void Motor::pulse_period(float seconds)
{
    if ((0 < seconds) && (seconds <= max_pulse_period))
    {
        _pulse_period = seconds;
    }
}

void Motor::frequency(float hz)
{
    pulse_period(1.0f / hz);
}

void Motor::release_time_ms(float ms)
{
    _release_time_ms = ms;
}

void Motor::control(int value)
{
    if ((0 <= value) && (value < TotalControl))
    {
        _control = (Control)value;
    }
}

const int Motor::default_state = Brake;
const int Motor::default_duty_cycle_chenge_level = OFF;
const float Motor::default_pulse_period = 0.00002f;
const float Motor::default_frequency = 1.0f / default_pulse_period;
const float Motor::defalut_release_time_ms = 100.0f;
const int Motor::default_control = SlowDecay;

const float Motor::max_pulse_period = 60000.0f;
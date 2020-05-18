/*
  *This software includes the work that is distributed in the Apache License 2.0
  */

#ifndef CAN_MOTOR_MANAGER_H
#define CAN_MOTOR_MANAGER_H

#include "mbed.h"
#include "CANMotor.h"
#include <vector>
#include "platform/SingletonPtr.h"
#include "platform/PlatformMutex.h"

class CANMotor;

class CANMotorManager
{
public:
    CANMotorManager(CAN &can);
    ~CANMotorManager();

    void add(CANMotor *ptr);
    void erase(CANMotor *ptr);
    int connect_all(int interval_ms = 100);
    int write_all(int interval_ms = 5);

private:
    CAN &_can;
    CANMessage _msg;

    static std::vector<CANMotor*> _motor_ptr;
    static SingletonPtr<PlatformMutex> _mutex;

    void decode();

    /** Acquire exclusive access to this motor_ptr
     */
    virtual void lock();

    /** Release exclusive access to this motor_ptr
     */
    virtual void unlock();

};

#endif
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
    /** Create a CAN Motor Manager interface
    *
    * @param can connect to can pins
    */
    CANMotorManager(CAN &can);
    
    ~CANMotorManager();

    /** Register additional instances of CAN Motor
     */
    void add(CANMotor *ptr);

    /** Delete the registered CAN Motor instance
     */
    void erase(CANMotor *ptr);

    /** Connect to all CAN Motors
     */
    int connect_all(int interval_ms = 100);

    /** Write to all CAN Motors
     */
    int write_all(int interval_ms = 5);

private:
    CAN &_can;
    CANMessage _msg;

    static std::vector<CANMotor*> _motor_ptr;
    static SingletonPtr<PlatformMutex> _mutex;

    /** Parse CAN message
     */
    void parse();

    /** Acquire exclusive access to this motor_ptr
     */
    virtual void lock();

    /** Release exclusive access to this motor_ptr
     */
    virtual void unlock();

};

#endif
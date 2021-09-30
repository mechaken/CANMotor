/*
*This software includes the work that is distributed in the Apache License 2.0
*/

#include "CANMotorManager.h"

#include "CANMotor.h"
#include "mbed.h"
#include <vector>
#include "platform/SingletonPtr.h"
#include "platform/PlatformMutex.h"

std::vector<CANMotor*> CANMotorManager::_motor_ptr;
SingletonPtr<PlatformMutex> CANMotorManager::_mutex;

CANMotorManager::CANMotorManager(CAN &can)
    : _can(can)
{
//    _can.attach(callback(this, &CANMotorManager::parse));
}

CANMotorManager::~CANMotorManager()
{
    
}

void CANMotorManager::add(CANMotor *ptr)
{
    lock();
    _motor_ptr.push_back(ptr);
    unlock();
}

void CANMotorManager::erase(CANMotor *ptr)
{
    lock();
    for(std::vector<CANMotor*>::iterator itr = _motor_ptr.begin(); itr != _motor_ptr.end(); ++itr)
    {
        if (*itr == ptr)
        {
            _motor_ptr.erase(itr);
            break;
        }
    }
    unlock();
}

int CANMotorManager::connect_all(int interval_ms)
{
    return 0;
//    int miss = 0;
//    int interval_us = interval_ms * 1000;
//
//    for(std::vector<CANMotor*>::iterator itr = _motor_ptr.begin(); itr != _motor_ptr.end(); ++itr)
//    {
//        int timeout = 0;
//        while(((*itr)->connect() == false))
//        {
//            wait_us(interval_us);
//            
//            if (timeout++ >= 3)
//            {
//                    miss++;
//                    break;
//            }
//        }
//    }
//
//    return miss;
}

int CANMotorManager::write_all(int interval_ms)
{
    int miss = 0;
    int interval_us = interval_ms * 1000;

    for(std::vector<CANMotor*>::iterator itr = _motor_ptr.begin(); itr != _motor_ptr.end(); ++itr)
    {
        if ((*itr)->write() == 0)
        {
            miss++;
            // debug("Couldn't write to can bus.\r");
            // debug("%d: pwm %0.2f, state %d\r", motor_number, duty_cycle, state);
        }
        wait_us(interval_us);
    }

    return miss;
}

void CANMotorManager::parse()
{
//    _can.read(_msg);
//
//    for(std::vector<CANMotor*>::iterator itr = _motor_ptr.begin(); itr != _motor_ptr.end(); ++itr)
//    {
//        if ((*itr)->id() == _msg.id)
//        {
//            (*itr)->parse(_msg.data);
//            break;
//        }
//    }
}

void CANMotorManager::lock()
{
    _mutex->lock();
}

void CANMotorManager::unlock()
{
    _mutex->unlock();
}

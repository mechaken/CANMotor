/*
  * This software includes the work that is distributed in the Apache License 2.0
  */

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include "Motor.h"
#include "mbed.h"

/** An CAN Motor is used to controll CAN motor driver
  *
  * You can use as many separate CANMotor objects as you require.
  *
  * Example:
  * @code
  * #include "mbed.h"
  * #include "CANMotor.h"
  *
  * using namespace nitk;
  *
  * static const int id = 0x30;
  *
  * CANMotor motor(p9, p10, id, 0);
  * // CAN can(p9, p10);
  * // CANMotor motor(can, id, 0);
  *
  * int main() {
  *   motor.duty_cycle(0.314);
  *   motor.state(CW);
  *   motor.rise_level(Middle);
  *   motor.fall_level(Low);
  *
  *   while(true) {
  *     motor.adapt_setting();
  *     wait_ms(10);
  *   }
  * }
  * @endcode
  */
class CANMotor : public Motor
{

public:
    /** Create a CAN Motor interface
    *
    * @param sda CAN data line pin
    * @param scl CAN clock line pin
    * @param id 8-bit CAN slave id [ addr | 0 ]
    * @param number motor number in this id
    */
    CANMotor(PinName rd, PinName td, int dip, int number);

    /** Create a CAN Motor interface
    *
    * @param sda CAN data line pin
    * @param scl CAN clock line pin
    * @param id 8-bit CAN slave id [ addr | 0 ]
    */
    CANMotor(PinName rd, PinName td, int id);

    /** Create a CAN Motor interface
    *
    * @param can_obj connect to can pins
    * @param id 8-bit CAN slave id [ addr | 0 ]
    * @param number motor number in this id
    */
    CANMotor(CAN &can_obj, int dip, int number);

    /** Create a CAN Motor interface
    *
    * @param can_obj connect to can pins
    * @param id 8-bit CAN slave id [ addr | 0 ]
    */
    CANMotor(CAN &can_obj, int id);

    ~CANMotor();

    /** Set CAN id
    *
    * @param id 8bit CAN slave id [ addr | 0 ]
    */
    void id(int value);

    /** Return the CAN id
    *
    * @returns
    *    the bus id
    */
    int id() const;

    int connect();

    int decode_can_message(unsigned char *data);

    /** Set the frequency of the CAN interface
    *
    *  @param hz The bus frequency in hertz
    */
    void can_frequency(int hz);

    /** Return the CAN frequency
    *
    * @returns
    *    the bus frequency in hertz
    */
    int can_frequency() const;

    /** Write a CANMessage to the bus.
     *
     *  @param msg The CANMessage to write.
     *
     *  @returns
     *    0 if write failed,
     *    1 if write was successful
     */
    virtual int write(void);

    /** Write extention_data to CAN bus
    *
    * @param data data to write out on CAN bus
    */
    // virtual int write_extention(void); // connect()に集約

    static const int offset_id_number;

protected:
    CAN *_can_p;
    CAN &_can;
    CANMessage _normal_msg;
    CANMessage _initial_msg;

    int _number;
    int _hz;
    int _has_received_ack;

    /** Update the duty cycle to MotorDriver
    *
    * @param duty_cycle duty cycle to update
    */
    void update_duty_cycle_data();

    /** Update the state to MotorDriver
    *
    * @param type state to update
    */
    void update_state_data();

    /** Update the extention data to MotorDriver
    *
    * @param type state to update
    */
    void update_extention_data();

    int int_encode(int value, int size, unsigned char *data, int bit_number);

    int float_to_bfloat16_encode(float value, unsigned char *data, int bit_number);

    // int mast_use_write_extention(); // なにこれ？
};

#endif

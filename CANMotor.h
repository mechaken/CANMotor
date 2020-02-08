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
  * static const int total_motor = 4;
  * 
  * CAN can(p9, p10);
  * CANMessage msg;
  * 
  * CANMotor motor[total_motor] = {
  *     CANMotor(can, 0, 0);
  *     CANMotor(can, 0, 1);
  *     CANMotor(can, 1, 0);
  *     CANMotor(can, 1, 1);
  * }
  * 
  * void get_can_data()
  * {
  *     can.read(msg);
  *     for (int i = 0; i < total_motor; i++)
  *     {
  *         if (motor[i].id() + 1 == msg.id)
  *         {
  *             motor[i].decode_can_message(msg.data);
  *             return;
  *         }
  *     }
  * }
  * 
  * int main() {
  *     // モーター0のデューティー比の変化具合を設定
  *     motor[0].rise_level(Motor::Low);
  *     motor[0].fall_level(Motor::High);
  * 
  *     can.attach(&get_can_data);
  * 
  *     for (int i = 0; i < total_motor; i++)
  *     {
  *         int j = 0;
  *         while ((motor[i].connect() == false) && (j++ < 5))
  *         {
  *             wait_us(20000);
  *         }
  *     }
  * 
  *   while(true) {
  *     // 下のように、それぞれ設定したいモーターに
  *     // デューティー比と回転方向を入力していく（モーターには書き込まれない）
  *     motor[3].duty_cycle(0.42);
  *     motor[3].state(Motor::CW);
  * 
  *     for (int i = 0; i < total_motor; i++)
  *     {
  *         // motor[x].write() が実行されて初めてモータードライバに設定値が書き込まれる
  *         int result = motor[i].write(); // motor[x].write() が実行されて初めてモータードライバに設定値が書き込まれる
  *         
  *         debug_if(!result, "Couldn't write to can bus.\r");
  *         wait_us(1000);
  *     }
  *   }
  * }
  * @endcode
  */
class CANMotor : public Motor
{

public:
    /** Create a CAN Motor interface
    *
    * @param sda CAN Receiver line pin
    * @param scl CAN Transmitter line pin
    * @param dip Slave DPI value
    * @param number Slave motor number
    */
    CANMotor(PinName rd, PinName td, int dip, int number);

    /** Create a CAN Motor interface
    *
    * @param sda CAN Receiver line pin
    * @param scl CAN Transmitter line pin
    * @param id Slave id
    */
    CANMotor(PinName rd, PinName td, int id);

    /** Create a CAN Motor interface
    *
    * @param can_obj connect to can pins
    * @param dip Slave DPI value
    * @param number Slave motor number
    */
    CANMotor(CAN &can_obj, int dip, int number);

    /** Create a CAN Motor interface
    *
    * @param can_obj connect to can pins
    * @param id Slave id
    */
    CANMotor(CAN &can_obj, int id);

    ~CANMotor();

    /** Set CAN id
    *
    * @param id Slave id
    */
    void id(int value);

    /** Return the CAN id
    *
    * @returns
    *    the slave id
    */
    int id() const;

    /** Connect MotorDriver
     * 
     * @returns
     *   0 if connect failed,
     *   1 if connect successful
     */
    int connect();

    /** Decode can message
     * 
     * @param data The data to decode
     * @returns
     *   0 if decode failed,
     *   1 if get initialization data request
     *   2 if get ack
     */
    int decode_can_message(unsigned char *data);

    /** Set the frequency of the CAN bus
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
     *    1 if write successful
     */
    virtual int write(void);

    /** Write extention_data to CAN bus
    *
    * @param data data to write out on CAN bus
    */

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
    */
    void update_extention_data();

    /** Embed integers at specified locations in data
    * 
    * @param value Value to embed
    * @param size Size of value to embed
    * @param data Write destination data
    * @param bit_nubmer to start writing
    * @returns
    *   0 if failed
    *   1 if successful  
    */
    int int_encode(int value, int size, unsigned char *data, int bit_number);

    /** Embed floating point data at specified location in data (convert float to bfloat16)
    * 
    * @param value Value to embed
    * @param size Size of value to embed
    * @param data Write destination data
    * @param bit_nubmer to start writing
    * @returns
    *   0 if failed
    *   1 if successful  
    */
    int float_to_bfloat16_encode(float value, unsigned char *data, int bit_number);
};

#endif

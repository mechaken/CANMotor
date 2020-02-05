/*
*This software includes the work that is distributed in the Apache License 2.0
*/
 
#include "CANMotor.h"
 
#include "Motor.h"
#include "mbed.h"
#include <cstdint>
#include "bfloat16.h"

CANMotor::CANMotor(PinName rd, PinName td, int dip, int number)
    : _can_p(new CAN(rd, td)), _can(*_can_p)
{
    int id = dip * 16 + number * 2 + offset_id_number;
    _normal_msg.id = id;
    _normal_msg.len = 3;
    _normal_msg.data[0] = 0x00;
    _number = number;
    
    _initial_msg.id = id;
}

CANMotor::CANMotor(PinName rd, PinName td, int id)
    : _can_p(new CAN(rd, td)), _can(*_can_p)
{
    _normal_msg.id = id;
    _normal_msg.len = 3;
    _normal_msg.data[0] = 0x00;
    
    _initial_msg.id = id;
}
 
CANMotor::CANMotor(CAN &can_obj, int dip, int number)
    : _can_p(NULL), _can(can_obj)
{
    int id = dip * 16 + number * 2 + offset_id_number;
    _normal_msg.id = id;
    _normal_msg.len = 3;
    _normal_msg.data[0] = 0x00;
    _number = number;
    
    _initial_msg.id = id;
}

CANMotor::CANMotor(CAN &can_obj, int id)
    : _can_p(NULL), _can(can_obj)
{
    _normal_msg.id = id;
    _normal_msg.len = 3;
    _normal_msg.data[0] = 0x00;
    
    _initial_msg.id = id;
}
 
CANMotor::~CANMotor()
{
    if (NULL != _can_p)
        delete _can_p;
}
 
void CANMotor::id(int id) { _normal_msg.id = id; }
 
int CANMotor::id() const { return _normal_msg.id; }

int CANMotor::connect()
{
    _has_received_ack = false;

    update_extention_data();
    _can.write(_initial_msg);

    int i = 0;
    while ((_has_received_ack == false) && (i++ < 10))
    {
        wait_us(10000);
    }

    debug_if(_has_received_ack == false, "Don't receive ack.\n");

    return _has_received_ack;
}

int CANMotor::decode_can_message(unsigned char *data)
{
    if (data[0] == 0)
    {
        connect();
    }
    else if (data[0] == 1) {
        _has_received_ack = true;
        debug("received_ack\n");
    }

    return 0;
}
 
void CANMotor::can_frequency(int hz)
{
    _hz = hz;
    _can.frequency(hz);
}

int CANMotor::frequency() const { return _hz; }

void CANMotor::update_duty_cycle_data()
{
    uint16_t duty_cycle_16bit = 65536 * _duty_cycle; // 2^16 = 65536

    // Clear DutyCycle bits
    _normal_msg.data[0] &= 0x80;
    _normal_msg.data[1] &= 0x00;
    _normal_msg.data[2] &= 0x7F;

    _normal_msg.data[0] |= duty_cycle_16bit >> 9;
    _normal_msg.data[1] |= (duty_cycle_16bit & 0x01FE) >> 1;
    _normal_msg.data[2] |= (duty_cycle_16bit & 0x0001) << 7;
}

void CANMotor::update_state_data()
{
    // Clear state bits
    _normal_msg.data[2] &= 0x9F;

    _normal_msg.data[2] |= (_state << 5);
}

void CANMotor::update_extention_data()
{
    int bit_number = 0; // MSBからで、いままで使ったビット数を代入（次のビットを参照するため）

    int_encode(2, 2, _initial_msg.data, bit_number); // 初期値設定を示す接頭辞

    bit_number += 2;

    if ((_rise_level != default_duty_cycle_chenge_level)
        || (_fall_level != default_duty_cycle_chenge_level))
    {
        // DutyCycle Change Level heder
        int_encode(0b10, 2, _initial_msg.data, bit_number);
        bit_number += 2;

        int_encode(_rise_level, 3, _initial_msg.data, bit_number);
        bit_number += 3;

        int_encode(_fall_level, 3, _initial_msg.data, bit_number);
        bit_number += 3;
    }

    if (_control != default_control)
    {
        // control heder
        int_encode(0b10, 2, _initial_msg.data, bit_number);
        bit_number += 2;

        int_encode(_control, 2, _initial_msg.data, bit_number);
        bit_number += 2;
    }

    if (_pulse_period != default_pulse_period)
    {
        // pulse_period heder
        int_encode(0b010, 3, _initial_msg.data, bit_number);
        bit_number += 3;

        float_to_bfloat16_encode(_pulse_period, _initial_msg.data, bit_number);
        bit_number += 16;
    }

    if (_release_time_ms != defalut_release_time_ms)
    {
        // release_time_ms heder
        int_encode(0b110, 3, _initial_msg.data, bit_number);
        bit_number += 3;

        float_to_bfloat16_encode(_release_time_ms, _initial_msg.data, bit_number);
        bit_number += 16;
    }
}
 
int CANMotor::write()
{
    // 結構無駄な計算だけど、Motor.hをCANMotor.hありきのライブラリにしたくないのでこっちのほうがいい
    update_duty_cycle_data();
    update_state_data();

    int result = _can.write(_normal_msg);
    return result;
}

int CANMotor::int_encode(int value, int size, unsigned char *data, int bit_number)
{
    if (bit_number + size > 64)
        return 1;

    if ((value >> size) > 0)
        return 2;

    for (int i = 0; i < size; i++) // 1bitずつ代入
    {
        int subscript = (bit_number + i) / 8; // 配列の添え字を計算
        int bit_num_of_subscript = (bit_number + i) % 8; // どのbitなのか計算
        data[subscript] |= ((value >> (size - (i + 1))) & 0x01) << (7 - bit_num_of_subscript);
    }

    return 0;
}

int CANMotor::float_to_bfloat16_encode(float value, unsigned char *data, int bit_number)
{
    if (bit_number + 16 > 64)
        return 1;
    
    for (int i = 0; i < 16; i++)
    {
        int subscript = (bit_number + i) / 8;
        int bit_num_of_subscript = (bit_number + i) % 8;

        uint16_t value_bfloat16 = bfloat16::float32_to_bfloat16(value);

        data[subscript] |= ((value_bfloat16 >> (15 - i)) & 0x01) << (7 - bit_num_of_subscript);
    }

    return 0;
}

const int CANMotor::offset_id_number = 0x300;
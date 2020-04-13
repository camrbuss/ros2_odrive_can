#ifndef __ODRIVE_CAN_HPP_
#define __ODRIVE_CAN_HPP_

#include <stdint.h>
#include <algorithm>
#include <cstring>
#include <iterator>
#include <linux/can.h>

namespace odrive_can
{

//     Frame
// nodeID | CMD
// 6 bits | 5 bits
static const uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

enum AXIS
{
    AXIS_0 = 0,
    AXIS_1 = 1,
    AXIS_0_ID = 3 << NUM_CMD_ID_BITS,
    AXIS_1_ID = 1 << NUM_CMD_ID_BITS,
};

struct Msg
{
    enum Value
    {
        MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR, // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_CURRENT,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_A_PER_CSS,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,
        MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
    };
};

struct can_Signal_t
{
    const uint8_t startBit;
    const uint8_t length;
    const bool isIntel;
    const float factor;
    const float offset;
};

template <typename T>
T can_getSignal(can_frame msg, const uint8_t startBit, const uint8_t length, const bool isIntel)
{
    uint64_t tempVal = 0;
    uint64_t mask = (1ULL << length) - 1;

    if (isIntel)
    {
        std::memcpy(&tempVal, msg.data, sizeof(tempVal));
        tempVal = (tempVal >> startBit) & mask;
    }
    else
    {
        std::reverse(std::begin(msg.data), std::end(msg.data));
        std::memcpy(&tempVal, msg.data, sizeof(tempVal));
        tempVal = (tempVal >> (64 - startBit - length)) & mask;
    }

    T retVal;
    std::memcpy(&retVal, &tempVal, sizeof(T));
    return retVal;
}

template <typename T>
float can_getSignal(can_frame msg, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset)
{
    T retVal = can_getSignal<T>(msg, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T>
void can_setSignal(can_frame &msg, const T &val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset)
{
    T scaledVal = (val - offset) / factor;
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &scaledVal, sizeof(scaledVal));

    uint64_t mask = (1ULL << length) - 1;

    if (isIntel)
    {
        uint64_t data = 0;
        std::memcpy(&data, msg.data, sizeof(data));

        data &= ~(mask << startBit);
        data |= valAsBits << startBit;

        std::memcpy(msg.data, &data, sizeof(data));
    }
    else
    {
        uint64_t data = 0;
        std::reverse(std::begin(msg.data), std::end(msg.data));
        std::memcpy(&data, msg.data, sizeof(data));

        data &= ~(mask << (64 - startBit - length));
        data |= valAsBits << (64 - startBit - length);

        std::memcpy(msg.data, &data, sizeof(data));
        std::reverse(std::begin(msg.data), std::end(msg.data));
    }
}

template <typename T>
float can_getSignal(can_frame msg, const can_Signal_t &signal)
{
    return can_getSignal<T>(msg, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

template <typename T>
void can_setSignal(can_frame &msg, const T &val, const can_Signal_t &signal)
{
    can_setSignal(msg, val, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

} // namespace odrive_can

#endif
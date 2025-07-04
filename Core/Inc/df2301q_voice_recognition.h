/*
 * df2301q_voice_recognition.h
 *
 *  Created on: Jul 3, 2025
 *      Author: user
 */

#ifndef INC_DF2301Q_VOICE_RECOGNITION_H_
#define INC_DF2301Q_VOICE_RECOGNITION_H_

#include "i2c.h"

#define DF2301Q_I2C_HANDLE_PTR           &hi2c1
#define DF2301Q_I2C_ADDR                 (0x64<<1)   //!< I2C address
// https://github.com/DFRobot/DFRobot_DF2301Q
#define DF2301Q_I2C_REG_CMDID            0x02   //!< Address of the register for requesting command word ID
#define DF2301Q_I2C_REG_PLAY_CMDID       0x03   //!< Address of the register for playing audio by command word ID
#define DF2301Q_I2C_REG_SET_MUTE         0x04   //!< Register for setting mute mode
#define DF2301Q_I2C_REG_SET_VOLUME       0x05   //!< Register for setting volume
#define DF2301Q_I2C_REG_WAKE_TIME        0x06   //!< Address of the register for wake-up time

#define VOICE_CMD_TURN_OFF_THE_FAN 76
#define VOICE_CMD_TURN_OFF_THE_FAN_STRING "Turn off the fan."
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_ONE 77
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_ONE_STRING "Turn fan speed to gear one."
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_TWO 78
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_TWO_STRING "Turn fan speed to gear two."
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_THREE 79
#define VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_THREE_STRING "Turn fan speed to gear three."

uint8_t DF2301G_GetCommandID(void);

#endif /* INC_DF2301Q_VOICE_RECOGNITION_H_ */

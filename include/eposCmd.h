/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
 /**
 * @file eposCmd.h
 * @author Cyril Jourdan
 * @date Jan 23, 2017
 * @version 2.0.0
 * @brief header file for controlling EPOS2 from Maxon motor, over CAN bus
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 27, 2013
 */
 
#ifndef EPOSCMD_H
#define EPOSCMD_H

/*! Includes */
//#include "mbed.h"
#include <stdint.h>
//#include <unistd.h> //for usleep
//#include <time.h>

#include "registers.h"
#include "robot.h"

/*! Defines */
#define NUMBER_BYTES_PER_MSG        8
#define NB_SAMPLES_MEDIAN           5
#define PAUSE                       10
#define EPOS_OK                     0
#define EPOS_ERROR                  -1
#define TIMEOUT                     10000

/*! Enums */

/*!< enum that describe the possible commands received from ROS */
enum ROSCommand //TODO special EPOS4 cases
{
    SET_TARGET_POSITION = 0,
    SET_TARGET_VELOCITY = 1,    
    SET_PROFILE_ACCELERATION = 2,
    SET_PROFILE_DECELERATION = 3,
    SET_PROFILE_VELOCITY = 4,
    SET_OUTPUT_CURRENT_LIMIT = 5,
    SET_CONTROLWORD = 6,
    SET_CURRENT_MODE_SETTING_VALUE = 7,
    SET_MAXIMAL_SPEED_IN_CURRENT_MODE = 8,
    SET_MODES_OF_OPERATION = 9,     
    SEND_DUMB_MESSAGE = 15
}; 

/*!< enum CustomValue */
enum CustomValue
{
    FORCE_VALUE = 0,
    BOARD_STATUS = 1
};

/*! Global variables */


//DigitalOut ledchain[];   //!< used for debugging
char data[8];

ActivatedModeOfOperation activMode[NUMBER_OF_MOTORS]; //duplicate of modesOfOperation[], to merge
ROSCommand rosCmd[NUMBER_OF_MOTORS];

//latest stored commands
//RPDO1
int32_t targetPosition[NUMBER_OF_MOTORS];
int32_t targetVelocity[NUMBER_OF_MOTORS];
//RPDO2
uint32_t profileAcceleration[NUMBER_OF_MOTORS];
uint32_t profileDeceleration[NUMBER_OF_MOTORS];
//RPDO3
uint32_t profileVelocity[NUMBER_OF_MOTORS];
uint16_t outputCurrentLimit[NUMBER_OF_MOTORS];
int16_t currentModeSettingValue[NUMBER_OF_MOTORS];
//RPDO4
uint32_t maximalSpeedInCurrentMode[NUMBER_OF_MOTORS];
uint16_t controlword[NUMBER_OF_MOTORS];
int8_t modesOfOperation[NUMBER_OF_MOTORS];

//sensor variables
//TPDO1
int32_t position[NUMBER_OF_MOTORS];
int32_t velocity[NUMBER_OF_MOTORS];
//TPDO2
int16_t current[NUMBER_OF_MOTORS];
int16_t followingError[NUMBER_OF_MOTORS];
uint16_t statusword[NUMBER_OF_MOTORS];
//TPDO3
//int8_t modesOfOperation[NUMBER_OF_MOTORS];
//TPDO4
uint32_t incEnc1CntAtIdxPls[NUMBER_OF_MOTORS];

int8_t boardStatus[NUMBER_OF_MOTORS];

/*! functions */
void setMultiplexerChannel(const uint8_t);
static void setNMT(const uint8_t, uint8_t);
int8_t setObjectSDO(const uint8_t, const int32_t, int32_t);
static int8_t setPDO(const uint8_t, uint16_t, uint8_t, uint32_t, uint8_t);

//EPOS2 boards and Motor settings
//static void setMotorType(const uint8_t);
//static void setPolePair(const uint8_t);
//static void setMaximalMotorSpeed(const uint8_t);
//static void setMaximalProfileVelocity(const uint8_t);
//static void setMaxAcceleration(const uint8_t);
//static void setThermalTimeConstantWinding(const uint8_t);
//static void setMaximalFollowingError(const uint8_t);

int8_t setModeOfOperationSDO(const uint8_t, int8_t);

void shutdownControlword(const uint8_t);
void shutdownControlwordIT(const uint8_t);
void switchOnEnableOperationControlword(const uint8_t);
void switchOnEnableOperationControlwordIT(const uint8_t);
void faultResetControlword(const uint8_t);
//static void reEnableControlword(const uint8_t);

int8_t initEposBoard(const uint8_t);

//!< RPDO
void setTargetPosition(const uint8_t , int32_t );
void setTargetVelocity(const uint8_t, int32_t);
void setProfileAccelerationDeceleration(const uint8_t, uint32_t, uint32_t);
void setProfileAcceleration(const uint8_t, uint32_t);
void setProfileDeceleration(const uint8_t, uint32_t);
void setProVelContCurrLmtCrtlWord(const int8_t, uint32_t, int16_t, uint16_t);
void setProfileVelocity(const uint8_t, uint32_t);
void setOutputCurrentLimit(const uint8_t, uint16_t);
void setControlword(const uint8_t, uint16_t);
void setCurrentSpeedMode(const uint8_t, int16_t, uint32_t, int8_t);
void setCurrentModeSettingValue(const uint8_t, int16_t);
void setMaximalSpeedInCurrentMode(const uint8_t, uint32_t);
void setModesOfOperation(const uint8_t, int8_t);

//!< TPDO
void getPositionVelocity(const uint8_t);
void getCurrentFollErrStatusword(const uint8_t);
void getModesOfOperation(const uint8_t);
void getIncEnc1CntAtIdxPls(const uint8_t);

#endif //EPOSCMD_H

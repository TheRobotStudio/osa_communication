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
 * @file enums.h
 * @author Cyril Jourdan
 * @date Mar 03, 2017
 * @version 2.0.0
 * @brief header file for enums definitions
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 11, 2015
 */

#ifndef ENUMS_H
#define ENUMS_H

/*! Defines */
//#define NUMBER_OF_MOTORS	2
#define NUMBER_BYTES_PER_MSG        8
#define NB_SAMPLES_MEDIAN           5
#define PAUSE                       10
#define EPOS_OK                     0
#define EPOS_ERROR                  -1
#define TIMEOUT                     10000

/*! Enums */
enum MotorType
{
    NONE = 0,
    DCX10 = 1,
    DCX14 = 2,
    DCX16 = 3,
    DCX22 = 4,
    DCX32 = 5,
    RE13 = 6,
    RE30 = 7,
    ECI40 = 8,
    ECI52 = 9,
    EC90 = 10
}; 

enum ControllerType
{
    NOT_USED = 0,
    EPOS2 = 1,
    EPOS4 = 2
}; 

/*!< enum that describe the activated mode of operation of a motor controller */
enum ActivatedModeOfOperation
{
    INTERPOLATED_POSITION_MODE = 0,
    PROFILE_VELOCITY_MODE = 1,
    PROFILE_POSITION_MODE = 2,
    POSITION_MODE = 3,
    VELOCITY_MODE = 4,
    CURRENT_MODE = 5,  
    CYCLIC_SYNCHRONOUS_TORQUE_MODE = 6
    //NO_MODE = 15
}; 

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

#endif //ENUMS_H

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
 * @file robot.h
 * @author Cyril Jourdan
 * @date Mar 03, 2017
 * @version 2.0.0
 * @brief header file for Robot
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 11, 2015
 */

#ifndef ROBOT_H
#define ROBOT_H

/*! Includes */
//#include "ConfigFile.h"
#include <unistd.h> //for usleep
#include <time.h>

/*! Defines */
#define NUMBER_OF_MOTORS	2

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
 
/*! Structures */
typedef struct Motor 
{
   int nodeID;
   ControllerType controllerType;
   MotorType motorType;   
   bool inverted; //0 for non inverted, 1 for inverted, this will invert the sign of commands.
   ActivatedModeOfOperation mode;
   int value; //current
} Motor;

        
/*! Variables */
//extern Serial pc;
Motor motorArray[NUMBER_OF_MOTORS];
uint8_t numberEposBoards;

#endif //ROBOT_H

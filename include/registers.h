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
 * @file registers.h
 * @author Cyril Jourdan
 * @date Mar 02, 2017
 * @version 2.0.0
 * @brief header file that defines the CAN registers, based on MAXON EPOS2 Firmware Specification
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 27, 2013
 */
 
#ifndef REGISTERS_H
#define REGISTERS_H

/*! Defines */

/*! Communication */
/*! Node IDs (FS 7.1) */
#define NODE_ID_BROADCAST   0x00

/*! Network Management (FS 7.3) */
#define CAN_NMT_ID                  0x00
#define CS_ENTER_PRE_OPERATIONAL    0x80
#define CS_RESET_COMMUNICATION      0x82
#define CS_RESET_NODE               0x81
#define CS_START_REMOTE_NODE        0x01
#define CS_STOP_REMOTE_NODE         0x02

/*! Application Notes 10.4.1 Expedited SDO Protocol */
/*! Writing Object Table 10-141 */
#define WRITING_OBJECT_1_BYTE       0x2F 
#define WRITING_OBJECT_2_BYTE       0x2B 
#define WRITING_OBJECT_4_BYTE       0x23 
#define WRITING_OBJECT_NOT_DEFINED  0x22

/*! COB-ID of Emergency Object (FS 8.2.12) */
#define COB_ID_EMCY_INDEX                               0x1014
#define COB_ID_EMCY_SUBINDEX                            0x00
#define COB_ID_EMCY_DEFAULT                             0x00000080

/*! SDO Server Parameter (FS 8.2.17) */
#define COB_ID_SDO_INDEX                                0x1200
#define COB_ID_SDO_CLIENT_TO_SERVER_SUBINDEX            0x01
#define COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT             0x00000600
#define COB_ID_SDO_SERVER_TO_CLIENT_SUBINDEX            0x02
#define COB_ID_SDO_SERVER_TO_CLIENT_DEFAULT             0x00000580

/*! PDOs */
/*! Receive PDO 1 Parameter (FS 8.2.18) */
#define RECEIVE_PDO_1_PARAMETER_INDEX                   0x1400
#define COB_ID_RECEIVE_PDO_1_SUBINDEX                   0x01
#define COB_ID_RECEIVE_PDO_1_ENABLE                     0x00000200
#define COB_ID_RECEIVE_PDO_1_DISABLE                    0xC0000200 //!< C is for valid = 1b and RTR = 1b
#define TRANSMISSION_TYPE_RECEIVE_PDO_1_SUBINDEX        0x02    //!< not used, default value is 255 for asynchronous

/*! Receive PDO 2 Parameter (FS 8.2.19) */
#define RECEIVE_PDO_2_PARAMETER_INDEX                   0x1401
#define COB_ID_RECEIVE_PDO_2_SUBINDEX                   0x01
#define COB_ID_RECEIVE_PDO_2_ENABLE                     0x00000300
#define COB_ID_RECEIVE_PDO_2_DISABLE                    0xC0000300
#define TRANSMISSION_TYPE_RECEIVE_PDO_2_SUBINDEX        0x02

/*! Receive PDO 3 Parameter (FS 8.2.20) */
#define RECEIVE_PDO_3_PARAMETER_INDEX                   0x1402
#define COB_ID_RECEIVE_PDO_3_SUBINDEX                   0x01
#define COB_ID_RECEIVE_PDO_3_ENABLE                     0x00000400
#define COB_ID_RECEIVE_PDO_3_DISABLE                    0xC0000400
#define TRANSMISSION_TYPE_RECEIVE_PDO_3_SUBINDEX        0x02

/*! Receive PDO 4 Parameter (FS 8.2.21) */
#define RECEIVE_PDO_4_PARAMETER_INDEX                   0x1403
#define COB_ID_RECEIVE_PDO_4_SUBINDEX                   0x01
#define COB_ID_RECEIVE_PDO_4_ENABLE                     0x00000500
#define COB_ID_RECEIVE_PDO_4_DISABLE                    0xC0000500
#define TRANSMISSION_TYPE_RECEIVE_PDO_4_SUBINDEX        0x02

/*! Receive PDO 1 Mapping (FS 8.2.22) */
#define MAPPED_OBJECT_RECEIVE_PDO_1_INDEX               0x1600

/*! Receive PDO 2 Mapping (FS 8.2.23) */
#define MAPPED_OBJECT_RECEIVE_PDO_2_INDEX               0x1601

/*! Receive PDO 3 Mapping (FS 8.2.24) */
#define MAPPED_OBJECT_RECEIVE_PDO_3_INDEX               0x1602

/*! Receive PDO 4 Mapping (FS 8.2.25) */
#define MAPPED_OBJECT_RECEIVE_PDO_4_INDEX               0x1603

#define NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX             0x00
#define MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX            0x01
#define MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX            0x02
#define MAPPED_OBJECT_3_RECEIVE_PDO_SUBINDEX            0x03
#define MAPPED_OBJECT_4_RECEIVE_PDO_SUBINDEX            0x04
#define MAPPED_OBJECT_5_RECEIVE_PDO_SUBINDEX            0x05
#define MAPPED_OBJECT_6_RECEIVE_PDO_SUBINDEX            0x06
#define MAPPED_OBJECT_7_RECEIVE_PDO_SUBINDEX            0x07
#define MAPPED_OBJECT_8_RECEIVE_PDO_SUBINDEX            0x08

/*! Receive PDO Mapping Objects (FS 8.2.22) */
/*! List of mappable objects Table 8-77 */
#define OBJECT_HIGH_RESOLUTION_TIME_STAMP           0x10130020
#define OBJECT_CURRENT_MODE_SETTING_VALUE           0x20300010
#define OBJECT_POSITION_MODE_SETTING_VALUE          0x20620020
#define OBJECT_VELOCITY_MODE_SETTING_VALUE          0x206B0020
#define OBJECT_POSITION_MARKER_COUNTER              0x20740410
#define OBJECT_DIGITAL_OUTPUT_FUNCTIONALITIES       0x20780110
#define OBJECT_POSITION_COMPARE_CONFIGURATION       0x207A0110
#define OBJECT_POSITION_COMPARE_REFERENCE_POSITION  0x207A0220
#define OBJECT_ANALOG_OUTPUT_1                      0x207E0010
#define OBJECT_CURRENT_THRESHOLD_FOR_HOMING_MODE    0x20800010
#define OBJECT_HOME_POSITION                        0x20810020
#define OBJECT_INTERPOLATION_DATA_RECORD            0x20C10040
#define OBJECT_CONTROLWORD                          0x60400010
#define OBJECT_MODES_OF_OPERATION                   0x60600008
#define OBJECT_MAXIMAL_FOLLOWING_ERROR              0x60650020
#define OBJECT_TARGET_POSITION                      0x607A0020
#define OBJECT_HOME_OFFSET                          0x607C0020
#define OBJECT_PROFILE_VELOCITY                     0x60810020
#define OBJECT_PROFILE_ACCELERATION                 0x60830020
#define OBJECT_PROFILE_DECELERATION                 0x60840020
#define OBJECT_QUICKSTOP_DECELERATION               0x60850020
#define OBJECT_MOTION_PROFILE_TYPE                  0x60860010
#define OBJECT_HOMING_METHOD                        0x60980008
#define OBJECT_SPEEDS_FOR_SWITCH_SEARCH             0x60990120
#define OBJECT_SPEEDS_FOR_ZERO_SEARCH               0x60990220
#define OBJECT_HOMING_ACCELERATION                  0x609A0020
#define OBJECT_BUFFER_CLEAR                         0x60C40608
#define OBJECT_CURRENT_REGULATOR_P_GAIN             0x60F60110
#define OBJECT_CURRENT_REGULATOR_I_GAIN             0x60F60210
#define OBJECT_VELOCITY_REGULATOR_P_GAIN            0x60F90110
#define OBJECT_VELOCITY_REGULATOR_I_GAIN            0x60F90210
#define OBJECT_VELOCITY_FEEDFORWARD_FACTOR          0x60F90410
#define OBJECT_ACCELERATION_FEEDFORWARD_FACTOR      0x60F90510
#define OBJECT_POSITION_REGULATOR_P_GAIN            0x60FB0110
#define OBJECT_POSITION_REGULATOR_I_GAIN            0x60FB0210
#define OBJECT_POSITION_REGULATOR_D_GAIN            0x60FB0310
#define OBJECT_VELOCITY_FEED_FORWARD_FACTOR         0x60FB0410
#define OBJECT_ACCELERATION_FEED_FORWARD_FACTOR     0x60FB0510
#define OBJECT_TARGET_VELOCITY                      0x60FF0020
#define OBJECT_CONTINUOUS_CURRENT_LIMIT             0x64100110
#define OBJECT_OUTPUT_CURRENT_LIMIT                 0x64100210
#define OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE        0x64100420

/*! SDO  objects only */
#define OBJECT_DEVICE_TYPE                          0x10000020
#define OBJECT_ERROR_REGISTER                       0x10010008
#define OBJECT_NUMBER_OF_ERRORS                     0x10030008
#define OBJECT_COB_ID_SYNC                          0x10050020
#define OBJECT_POLE_PAIR_NUMBER                     0x64100308
#define OBJECT_MAXIMAL_PROFILE_VELOCITY             0x607F0020
#define OBJECT_MAXIMAL_ACCELERATION                 0x60C50020
#define OBJECT_THERMAL_TIME_CONSTANT_WINDING        0x64100510
#define OBJECT_MISCELLANEOUS_CONFIGURATION          0x20080010
#define OBJECT_STORE_PARAMETERS                     0x10100120
#define OBJECT_MOTOR_TYPE                           0x64020010

/*! Objects values */
/*! Motor Type Table 8-125 */
#define BRUSHED_DC_MOTOR            0x0001
#define EC_MOTOR_HALL_ENCODER1      0x000A
#define EC_MOTOR_HALL               0x000B //0x001A
#define EC_MOTOR_HALL_ENCODER2      0xFFFF

#define SAVE_ALL                    0x65766173 

/*! Transmit PDO 1 Parameter (FS 8.2.26) */
#define TRANSMIT_PDO_1_PARAMETER_INDEX                  0x1800
#define COB_ID_TRANSMIT_PDO_1_SUBINDEX                  0x01
#define COB_ID_TRANSMIT_PDO_1_ENABLE                    0x00000180
#define COB_ID_TRANSMIT_PDO_1_DISABLE                   0xC0000180
#define TRANSMISSION_TYPE_TRANSMIT_PDO_1_SUBINDEX       0x02
#define INHIBIT_TIME_TRANSMIT_PDO_1_SUBINDEX            0x03

/*! Transmit PDO 2 Parameter (FS 8.2.27) */
#define TRANSMIT_PDO_2_PARAMETER_INDEX                  0x1801
#define COB_ID_TRANSMIT_PDO_2_SUBINDEX                  0x01
#define COB_ID_TRANSMIT_PDO_2_ENABLE                    0x00000280
#define COB_ID_TRANSMIT_PDO_2_DISABLE                   0xC0000280
#define TRANSMISSION_TYPE_TRANSMIT_PDO_2_SUBINDEX       0x02
#define INHIBIT_TIME_TRANSMIT_PDO_2_SUBINDEX            0x03

/*! Transmit PDO 3 Parameter (FS 8.2.28) */
#define TRANSMIT_PDO_3_PARAMETER_INDEX                  0x1802
#define COB_ID_TRANSMIT_PDO_3_SUBINDEX                  0x01
#define COB_ID_TRANSMIT_PDO_3_ENABLE                    0x00000380 
#define COB_ID_TRANSMIT_PDO_3_DISABLE                   0xC0000380
#define TRANSMISSION_TYPE_TRANSMIT_PDO_3_SUBINDEX       0x02
#define INHIBIT_TIME_TRANSMIT_PDO_3_SUBINDEX            0x03

/*! Transmit PDO 4 Parameter (FS 8.2.29) */
#define TRANSMIT_PDO_4_PARAMETER_INDEX                  0x1803
#define COB_ID_TRANSMIT_PDO_4_SUBINDEX                  0x01
#define COB_ID_TRANSMIT_PDO_4_ENABLE                    0x00000480 
#define COB_ID_TRANSMIT_PDO_4_DISABLE                   0xC0000480
#define TRANSMISSION_TYPE_TRANSMIT_PDO_4_SUBINDEX       0x02
#define INHIBIT_TIME_TRANSMIT_PDO_4_SUBINDEX            0x03

/*! Transmit parameter values */
#define PARAM_TRANSMISSION_TYPE_SYNC                    0x01
#define PARAM_TRANSMISSION_TYPE_RTR                     0xFD
#define PARAM_TRANSMISSION_TYPE_ASYNC                   0xFF

/*! Transmit PDO 1 Mapping (FS 8.2.30) */
#define MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX              0x1A00

/*! Transmit PDO 2 Mapping (FS 8.2.31) */
#define MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX              0x1A01

/*! Transmit PDO 3 Mapping (FS 8.2.32) */
#define MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX              0x1A02

/*! Transmit PDO 4 Mapping (FS 8.2.33) */
#define MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX              0x1A03

#define NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX            0x00
#define MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX           0x01
#define MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX           0x02
#define MAPPED_OBJECT_3_TRANSMIT_PDO_SUBINDEX           0x03
#define MAPPED_OBJECT_4_TRANSMIT_PDO_SUBINDEX           0x04
#define MAPPED_OBJECT_5_TRANSMIT_PDO_SUBINDEX           0x05
#define MAPPED_OBJECT_6_TRANSMIT_PDO_SUBINDEX           0x06
#define MAPPED_OBJECT_7_TRANSMIT_PDO_SUBINDEX           0x07
#define MAPPED_OBJECT_8_TRANSMIT_PDO_SUBINDEX           0x08

/*! Transmit PDO Mapping Objects (FS 8.2.30) */
#define OBJECT_HIGH_RESOLUTION_TIME_STAMP                   0x10130020
#define OBJECT_INCREMENTAL_ENCODER_1_COUNTER                0x20200010
#define OBJECT_INCREMENTAL_ENCODER_1_COUNTER_AT_INDEX_PULSE 0x20210010
#define OBJECT_HALL_SENSOR_PATTERN                          0x20220010
#define OBJECT_CURRENT_ACTUAL_VALUE_AVERAGED                0x20270010
#define OBJECT_VELOCITY_ACTUAL_VALUE_AVERAGED               0x20280020
#define OBJECT_CURRENT_MODE_SETTING_VALUE                   0x20300010
#define OBJECT_CURRENT_DEMAND_VALUE                         0x20310020
#define OBJECT_POSITION_MODE_SETTING_VALUE                  0x20620020
#define OBJECT_VELOCITY_MODE_SETTING_VALUE                  0x206B0020
#define OBJECT_DIGITAL_INPUT_FUNCTIONALITIES                0x20710110
#define OBJECT_POSITION_MARKER_CAPTURED_POSITION            0x20740120
#define OBJECT_POSITION_MARKER_COUNTER                      0x20740410
#define OBJECT_DIGITAL_OUTPUT_FUNCTIONALITIES               0x20780110
#define OBJECT_POSITION_COMPARE_CONFIGURATION               0x207A0110
#define OBJECT_POSITION_COMPARE_REFERENCE_POSITION          0x207A0220
#define OBJECT_ANALOG_INPUT_1                               0x207C0110
#define OBJECT_ANALOG_INPUT_2                               0x207C0210
#define OBJECT_ANALOG_OUTPUT_1                              0x207E0010
#define OBJECT_CURRENT_THRESHOLD_FOR_HOMING_MODE            0x20800010
#define OBJECT_HOME_POSITION                                0x20810020
#define OBJECT_INTERPOLATION_DATA_RECORD                    0x20C10040
#define OBJECT_INTERPOLATION_BUFFER_STATUS                  0x20C40110
#define OBJECT_FOLLOWING_ERROR_ACTUAL_VALUE                 0x20F40010
#define OBJECT_CONTROLWORD                                  0x60400010
#define OBJECT_STATUSWORD                                   0x60410010
#define OBJECT_MODES_OF_OPERATION                           0x60600008
#define OBJECT_MODES_OF_OPERATION_DISPLAY                   0x60610008
#define OBJECT_POSITION_DEMAND_VALUE                        0x60620020
#define OBJECT_POSITION_ACTUAL_VALUE                        0x60640020
#define OBJECT_MAXIMAL_FOLLOWING_ERROR                      0x60650020
#define OBJECT_VELOCITY_SENSOR_ACTUAL_VALUE                 0x60690020
#define OBJECT_VELOCITY_DEMAND_VALUE                        0x606B0020
#define OBJECT_VELOCITY_ACTUAL_VALUE                        0x606C0020
#define OBJECT_CURRENT_ACTUAL_VALUE                         0x60780010
#define OBJECT_TARGET_POSITION                              0x607A0020
#define OBJECT_HOME_OFFSET                                  0x607C0020
#define OBJECT_PROFILE_VELOCITY                             0x60810020
#define OBJECT_PROFILE_ACCELERATION                         0x60830020
#define OBJECT_PROFILE_DECELERATION                         0x60840020
#define OBJECT_QUICKSTOP_DECELERATION                       0x60850020
#define OBJECT_MOTION_PROFILE_TYPE                          0x60860010
#define OBJECT_HOMING_METHOD                                0x60980008
#define OBJECT_SPEEDS_FOR_SWITCH_SEARCH                     0x60990120
#define OBJECT_SPEEDS_FOR_ZERO_SEARCH                       0x60990220
#define OBJECT_HOMING_ACCELERATION                          0x609A0020
#define OBJECT_ACTUAL_BUFFER_SIZE                           0x60C40220
#define OBJECT_BUFFER_CLEAR                                 0x60C40608
//#define OBJECT_CURRENT_REGULATOR_P_GAIN                     0x60F60110
//#define OBJECT_CURRENT_REGULATOR_I_GAIN                     0x60F60210
//#define OBJECT_VELOCITY_REGULATOR_P_GAIN                    0x60F90110
//#define OBJECT_VELOCITY_REGULATOR_I_GAIN                    0x60F90210
#define OBJECT_VELOCITY_FEEDFORWARD_FACTOR                  0x60F90410
#define OBJECT_ACCELERATION_FEEDFORWARD_FACTOR              0x60F90510
//#define OBJECT_POSITION_REGULATOR_P_GAIN                    0x60FB0110
//#define OBJECT_POSITION_REGULATOR_I_GAIN                    0x60FB0210
//#define OBJECT_POSITION_REGULATOR_D_GAIN                    0x60FB0310
#define OBJECT_VELOCITY_FEED_FORWARD_FACTOR                 0x60FB0410
#define OBJECT_ACCELERATION_FEED_FORWARD_FACTOR             0x60FB0510
#define OBJECT_TARGET_VELOCITY                              0x60FF0020
#define OBJECT_CONTINUOUS_CURRENT_LIMIT                     0x64100110
#define OBJECT_OUTPUT_CURRENT_LIMIT                         0x64100210
//#define OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE                0x64100410

/*! OTHER OBJECTS INDEXES */
#define OBJECT_MODE_OF_OPERATION_INDEX      0x6060
#define OBJECT_MODE_OF_OPERATION_SUBINDEX   0x00
/*! CAN modes */
#define VALUE_INTERPOLATED_POSITION_MODE    0x07
#define VALUE_PROFILE_VELOCITY_MODE         0x03
#define VALUE_PROFILE_POSITION_MODE         0x01
#define VALUE_POSITION_MODE                 0xFF
#define VALUE_VELOCITY_MODE                 0xFE
#define VALUE_CURRENT_MODE                  0xFD
/*! Custom modes */
#define VALUE_CHANGE_OBJECT_VALUE           0x0B
//#define VALUE_NO_MODE                       0x7F//0x7F = 127 //0x0A    

/*! EPOS4 specific registers */
#define OBJECT_EPOS4_POLE_PAIR_NUMBER                   0x30010308
#define OBJECT_EPOS4_OUTPUT_CURRENT_LIMIT               0x30010220
#define OBJECT_EPOS4_FOLLOWING_ERROR_WINDOW             0x60650020 //same as Maximal Following Error on EPOS2
#define OBJECT_EPOS4_AXIS_CONFIGURATION_MISCELLANEOUS   0x30000420
#define OBJECT_EPOS4_MAX_PROFILE_VELOCITY               0x607F0020
#define OBJECT_EPOS4_MAX_ACCELERATION                   0x60C50020
#define OBJECT_EPOS4_TARGET_TORQUE                      0x60710010
#define OBJECT_EPOS4_MAX_MOTOR_SPEED                    0x60800020
#define OBJECT_EPOS4_VELOCITY_ACTUAL_VALUE_AVERAGED     0x30D30120
#define OBJECT_EPOS4_CURRENT_ACTUAL_VALUE_AVERAGED      0x30D10120

/*! EPOS4 specific values */
#define VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE            0x0A

#endif //REGISTERS_H

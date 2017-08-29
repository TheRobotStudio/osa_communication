/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
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
 * @file EPOSController.hpp
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 0.0.1
 * @brief Header file for class EPOSController
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#ifndef EPOS_CONTROLLER_H
#define EPOS_CONTROLLER_H

#include <ros/ros.h>
#include "enums.h"
#include "registers.h"

class EPOSController
{
	//methods
	public:
		//constructor
		EPOSController();
		//destructor
		~EPOSController();

		/*! methods */
		//setters
		int	setNodeID(unsigned int node_id);
		int	setName(std::string name);
		int setControllerType(ControllerType controller_type);
		int setMotorType(MotorType motor_type);
		int setInverted(bool inverted);
		int setMode(ActivatedModeOfOperation mode);
		int setValue(int value);

		int setPosition(int32_t position);
		int setCurrent(int16_t current);
		int setVelocity(int32_t velocity);
		int	setStatusword(uint16_t statusword);
		int setFollowingError(int16_t following_error);
		int setIncEnc1CntAtIdxPls(uint32_t inc_enc1_cnt_at_idx_pls);
		int setBoardStatus(int8_t board_status);

		//getters
		unsigned int getNodeID() const { return node_id_; };
		std::string	getName() const { return name_; };
		ControllerType getControllerType() const { return controller_type_; };
		MotorType getMotorType() const { return motor_type_; };
		bool getInverted() const { return node_id_; };
		ActivatedModeOfOperation getMode() const { return mode_; };
		int getValue() const { return value_; };

		int32_t getPosition() const { return position_; };
		int16_t getCurrent() const { return current_; };
		int32_t getVelocity() const { return velocity_; };
		uint16_t getStatusword() const { return statusword_; };
		int16_t getFollowingError() const { return following_error_; };
		uint32_t getIncEnc1CntAtIdxPls() const { return inc_enc1_cnt_at_idx_pls_; };

		int8_t getBoardStatus() const { return board_status_; };

	private:
		void canToEposWrite(int id, char* data, char len);
		void transmitPDOWrite(int tx_pdo_cob_id);
		void setNMT(uint8_t cs);
		int8_t setObjectSDO(const int32_t object, int32_t value);
		int8_t getObjectSDO(const int32_t object, int32_t value);
		int8_t setPDO(uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte);
		int8_t setModeOfOperationSDO(int8_t mode);

	public:
		void shutdownControlword();
		void shutdownControlwordIT();
		void switchOnEnableOperationControlword();
		void switchOnEnableOperationControlwordIT();
		void faultResetControlword();


		int8_t initEposBoard();

		//!< RPDO
		void setTargetPosition(int32_t position);
		void setTargetVelocity(int32_t velocity);
		void setProfileAccelerationDeceleration(uint32_t acceleration, uint32_t deceleration);
		void setProfileAcceleration(uint32_t acceleration);
		void setProfileDeceleration(uint32_t deceleration);
		void setCrtlWordProVelOutCurrLmt(uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt);
		void setProfileVelocity(uint32_t velocity);
		void setOutputCurrentLimit(uint16_t current);
		void setControlword(uint16_t ctrlWord);
		void setCurrentSpeedMode(int16_t current, uint32_t maxSpeedCurr, int8_t mode);
		void setCurrentModeSettingValue(int16_t current);
		void setMaximalSpeedInCurrentMode(uint32_t maxSpeedCurr);
		void setModesOfOperation(int8_t mode);
		//!< TPDO
		void getPositionVelocity();
		void getCurrentFollErrStatusword();
		void getModesOfOperation();
		void getIncEnc1CntAtIdxPls();

		int calibrate();
		void getData();

		void display();

	//attributes
	protected:

		unsigned int node_id_;
		std::string name_;
		ControllerType controller_type_;
		MotorType motor_type_;
		bool inverted_; //0 for non inverted, 1 for inverted, this will invert the sign of commands.
		ActivatedModeOfOperation mode_; //TODO duplicate of modes_of_operation_
		int value_;

		ros::Publisher *tx_frame_pub_;
		char data_[8]; //can message data

		ActivatedModeOfOperation activ_mode_; //duplicate of modesOfOperation[], to merge
		ROSCommand ros_cmd_;

		//latest stored commands
		//RPDO1
		int32_t target_position_;
		int32_t target_velocity_;
		//RPDO2
		uint32_t profile_acceleration_;
		uint32_t profile_deceleration_;
		//RPDO3
		uint32_t profile_velocity_;
		uint16_t output_current_limit_;
		int16_t current_mode_setting_value_;
		//RPDO4
		uint32_t maximal_speed_in_current_mode_;
		uint16_t controlword_;
		int8_t modes_of_operation_;

		//sensor variables
		//TPDO1
		int32_t position_;
		int32_t velocity_;
		//TPDO2
		int16_t current_;
		int16_t following_error_;
		uint16_t statusword_;
		//TPDO3
		//int8_t modesOfOperation;
		//TPDO4
		uint32_t inc_enc1_cnt_at_idx_pls_; //incremental encoder 1 count at index pulse

		int8_t board_status_;
};

#endif /* EPOS_CONTROLLER_H */

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
 * @version 0.1.0
 * @brief Header file for class EPOSController
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#ifndef OSA_COMMUNICATION_EPOS_CONTROLLER_H
#define OSA_COMMUNICATION_EPOS_CONTROLLER_H

#include <ros/ros.h>
#include "enums.h"
#include "registers.h"

namespace osa_communication
{

/**
 * @brief Class representing a maxon EPOS controller.
 */
class EPOSController
{
public:
	/**
	 * @brief Constructor.
	 */
	EPOSController(std::string name, std::string degree_of_freedom_type,
			int node_id, std::string controller_type,
			std::string motor_type, bool inverted,
			std::string mode, int value, ros::Publisher *tx_can_frame_pub);

	/** @brief Destructor. */
	~EPOSController();

	//setters //TODO check wether assessors are useful
	int	setName(std::string name);
	int	setNodeID(uint8_t node_id);
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
	std::string	getName() const { return name_; };
	uint8_t getNodeID() const { return node_id_; };

	ControllerType getControllerType() const { return controller_type_; };
	MotorType getMotorType() const { return motor_type_; };
	bool getInverted() const { return inverted_; };
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

	/*! \fn void setNMT(, uint8_t cs)
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return void
	 */
	void setNMT(uint8_t cs);

	/*! \fn int8_t setObjectSDO(const int32_t object, int32_t value)
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
	 */
	int8_t setObjectSDO(const int32_t object, int32_t value);

	/*! \fn int8_t getObjectSDO(const int32_t object, int32_t value)
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
	 */
	int8_t getObjectSDO(const int32_t object, int32_t value);

	/*! \fn int8_t setPDO(uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte)
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
	 */
	int8_t setPDO(uint16_t pdo_idx, uint8_t sub_idx, uint32_t value, uint8_t nb_byte);

	/*! \fn int8_t setModeOfOperationSDO(uint8_t mode)
	 *  \brief set the mode of operation using SDO.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param mode
	 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
	 */
	int8_t setModeOfOperationSDO(int8_t mode);

public:
	/*! \fn void shutdownControlword()
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \return void
	 */
	void shutdownControlword();

	/*! \fn void shutdownControlwordIT()
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \return void
	 */
	void shutdownControlwordIT();

	/*! \fn void switchOnEnableOperationControlword()
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \return void
	 */
	void switchOnEnableOperationControlword();

	/*! \fn void switchOnEnableOperationControlwordIT()
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \return void
	 */
	void switchOnEnableOperationControlwordIT();

	/*! \fn void faultResetControlword()
	 *  \brief
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \return void
	 */
	void faultResetControlword();

	/*! \fn int8_t initEposBoard()
	 *  \brief This funstion initialize the EPOS board of the specified node ID. It will stop if the motor type is set to NONE.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure or if the node is not responding.
	 */
	int8_t initEposBoard();

	/**< RPDO */
	/*! \fn void setTargetPosition(int32_t position)
	 *  \brief send a CAN frame using the Receive PDO 1, first object Target Position, the controller has to be in Profile Position Mode.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param position Target Position (in steps unit).
	 *  \return void
	 */
	void setTargetPosition(int32_t position);

	/*! \fn void setTargetVelocity(int32_t velocity)
	 *  \brief send a CAN frame using the Receive PDO 1, second object Target Velocity, the controller has to be in Profile Velocity Mode.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param velocity Target Velocity (in rpm unit).
	 *  \return void
	 */
	void setTargetVelocity(int32_t velocity);

	/*! \fn void setProfileAccelerationDeceleration(uint32_t acceleration, uint32_t deceleration)
	 *  \brief send a CAN frame using the Receive PDO 2, two objects: Profile Acceleration and Profile Deceleration.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
	 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
	 *  \return void
	 */
	void setProfileAccelerationDeceleration(uint32_t acceleration, uint32_t deceleration);

	/*! \fn void setProfileAcceleration(uint32_t acceleration)
	 *  \brief send a CAN frame using the Receive PDO 2, two objects: user defined Profile Acceleration and stored Profile Deceleration.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
	 *  \return void
	 */
	void setProfileAcceleration(uint32_t acceleration);


	/*! \fn void setProfileDeceleration(uint32_t deceleration)
	 *  \brief send a CAN frame using the Receive PDO 2, two objects: stored Profile Acceleration and user defined Profile Deceleration.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
	 *  \return void
	 */
	void setProfileDeceleration(uint32_t deceleration);

	/*! \fn void setCrtlWordProVelOutCurrLmt(uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt)
	 *  \brief send a CAN frame using the Receive PDO 3, three objects: Controlword, Profile Velocity, Output Current Limit (EPOS2 only).
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param ctrlWord Controlword.
	 *  \param velocity Profile Velocity (in rpm unit).
	 *  \param outCurrLmt Output Current Limit (in mA unit), only used with EPOS2.
	 *  \return void
	 */
	void setCrtlWordProVelOutCurrLmt(uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt);

	/*! \fn void setProfileVelocity(uint32_t velocity)
	 *  \brief send a CAN frame using the Receive PDO 3, three objects: user defined Profile Velocity, stored Output Current Limit and stored Controlword.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param velocity Profile Velocity (in rpm unit).
	 *  \return void
	 */
	void setProfileVelocity(uint32_t velocity);

	/*! \fn void setOutputCurrentLimit(uint16_t current)
	 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, user defined Output Current Limit and stored Controlword.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param current Output Current Limit (in mA unit).
	 *  \return void
	 */
	void setOutputCurrentLimit(uint16_t current);

	/*! \fn void setControlword(uint16_t ctrlWord)
	 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, stored Output Current Limit and user defined Controlword.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param ctrlWord Controlword.
	 *  \return void
	 */
	void setControlword(uint16_t ctrlWord);

	/*! \fn void setCurrentSpeedMode(int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
	 *  \brief send a CAN frame using the Receive PDO 4, three objects: Current Mode Setting Value, Maximal Speed In Current Mode and Modes Of Operation.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \param current Current Mode Setting Value (in mA unit).
	 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
	 *  \param mode Modes Of Operation.
	 *  \return void
	 */
	void setCurrentSpeedMode(int16_t current, uint32_t maxSpeedCurr, int8_t mode);

	/*! \fn void setCurrentModeSettingValue(int16_t current)
	 *  \brief send a CAN frame using the Receive PDO 4, three objects: user defined Current Mode Setting Value, stored Maximal Speed In Current Mode and stored Modes Of Operation.
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \param current Current Mode Setting Value (in mA unit).
	 *  \return void
	 */
	void setCurrentModeSettingValue(int16_t current);

	/*! \fn void setMaximalSpeedInCurrentMode(uint32_t maxSpeedCurr)
	 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, user defined Maximal Speed In Current Mode and stored Modes Of Operation.
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
	 *  \return void
	 */
	void setMaximalSpeedInCurrentMode(uint32_t maxSpeedCurr);

	/*! \fn void setModesOfOperation(int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
	 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, stored Maximal Speed In Current Mode and user defined Modes Of Operation.
	 *  \param node_id_ identifier of the node on the CAN bus
	 *  \param mode Modes Of Operation.
	 *  \return void
	 */
	void setModesOfOperation(int8_t mode);
	/**< TPDO */

	/*! \fn void getPositionVelocity()
	 *  \brief send a CAN frame using the Transmit PDO 1, two objects Position Actual Value and Velocity Actual Value Averaged, the answer frame is caught by interrupt.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return void
	 */
	void getPositionVelocity();

	/*! \fn void getCurrentFollErrStatusword()
	 *  \brief send a CAN frame using the Transmit PDO 2, two objects Current Actual Value Averaged, Following Error Actual Value and Statusword, the answer frame is caught by interrupt.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return void
	 */
	void getCurrentFollErrStatusword();

	/*! \fn void getModesOfOperation()
	 *  \brief send a CAN frame using the Transmit PDO 3, one object Modes Of Operation Display, the answer frame is caught by interrupt.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return void
	 */
	void getModesOfOperation();

	/*! \fn void getIncEnc1CntAtIdxPls()
	 *  \brief send a CAN frame using the Transmit PDO 4, one object Incremental Encoder 1 counter at Index Pulse, the answer frame is caught by interrupt.
	 *  \param node_id_ identifier of the node on the CAN bus.
	 *  \return void
	 */
	void getIncEnc1CntAtIdxPls();

	/*! \fn void calibrate()
	 *  \brief calibration routine.
	 *  \return int EPOS_OK or EPOS_ERROR
	 */
	int calibrate(); //TODO make it as a service

	/*! \fn void getData()
	 *  \brief
	 *  \return void
	 */
	void getData();  //TODO make it as a service

protected:
	std::string name_;
	DegreeOfFreedomType degree_of_freedom_type_;
	uint8_t node_id_;
	ControllerType controller_type_;
	MotorType motor_type_;
	bool inverted_; /**< 0 for non inverted, 1 for inverted, this will invert the sign of commands. */
	ActivatedModeOfOperation mode_; //TODO duplicate of modes_of_operation_
	int value_;

	ros::Publisher *tx_can_frame_pub_;
	char data_[8]; //can message data
	ActivatedModeOfOperation activ_mode_; //duplicate of modesOfOperation[], to merge
	ROSCommand ros_cmd_;

	//latest stored commands
	/**<RPDO1 */
	int32_t target_position_;
	int32_t target_velocity_;
	/**<RPDO2 */
	uint32_t profile_acceleration_;
	uint32_t profile_deceleration_;
	/**<RPDO3 */
	uint32_t profile_velocity_;
	uint16_t output_current_limit_;
	int16_t current_mode_setting_value_;
	/**<RPDO4 */
	uint32_t maximal_speed_in_current_mode_;
	uint16_t controlword_;
	int8_t modes_of_operation_;
	//sensor variables
	/**<TPDO1 */
	int32_t position_;
	int32_t velocity_;
	/**< TPDO2 */
	int16_t current_;
	int16_t following_error_;
	uint16_t statusword_;
	/**< TPDO3 */
	//int8_t modesOfOperation;
	/**< TPDO4 */
	uint32_t inc_enc1_cnt_at_idx_pls_; /**< Incremental encoder 1 count at index pulse. */
	int8_t board_status_;
};

} // osa_communication

#endif // OSA_COMMUNICATION_EPOS_CONTROLLER_H

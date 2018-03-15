/*
 * Copyright (c) 2018, The Robot Studio
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
 * @file socketcan_reader_nodelet.cpp
 * @author Cyril Jourdan
 * @date Feb 21, 2018
 * @version 0.1.0
 * @brief Implementation file for class SocketCANReaderNodelet
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 21, 2018
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <unistd.h>

#include "socketcan_reader_nodelet.h"
//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(osa_communication_nodelet::SocketCANReaderNodelet, nodelet::Nodelet)

using namespace osa_communication_nodelet;

/**
 * @brief Constructor.
 */
SocketCANReaderNodelet::SocketCANReaderNodelet() :
robot_name_(""),
robot_can_device_(""),
number_epos_boards_(0),
soc_(0),
read_can_port_(false),
pub_rx_can_frame_()
{
}

/**
 * @brief Destructor.
 */
SocketCANReaderNodelet::~SocketCANReaderNodelet()
{
	read_can_port_ = false;
	close_port();
}

void SocketCANReaderNodelet::onInit()
{
	ros::NodeHandle nh = this->getPrivateNodeHandle();

	// resolve node(let) name
	std::string name = nh.getUnresolvedNamespace();
	int pos = name.find_last_of('/');
	name = name.substr(pos + 1);

	NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");

	// Grab the parameters
	try
	{
		//load robot parameters
		if(!nh.param("/robot/name", robot_name_, std::string("my_robot")))
		{
			NODELET_WARN_STREAM("No /robot/name found in YAML config file");
		}

		if(!nh.param("/robot/dof", number_epos_boards_, int(0)))
		{
			NODELET_WARN_STREAM("No /robot/dof found in YAML config file");
		}

		if(!nh.param("/robot/can_device", robot_can_device_, std::string("can0")))
		{
			NODELET_WARN_STREAM("No /robot/can_device found in YAML config file");
		}

		NODELET_INFO_STREAM("Robot name=" << robot_name_<< ", dof=" << number_epos_boards_ << ", can=" << robot_can_device_);
	}
	catch(ros::InvalidNameException const &e)
	{
		NODELET_ERROR_STREAM(e.what());
		NODELET_ERROR_STREAM("Parameters didn't load correctly!");
		NODELET_ERROR_STREAM("Please modify your YAML config file and try again.");
		break;
	}

	//set CAN frame publisher, zero-copy data to the can_layer
	pub_rx_can_frame_ = new ros::Publisher(nh.advertise<can_msgs::Frame>("/rx_can_frame", 1));

	if(open_port(robot_can_device_.c_str()) == 0)
	{
		NODELET_INFO_STREAM("CAN port " << robot_can_device_ << " opened successfuly!");
	}
	else
	{
		NODELET_ERROR_STREAM("Couldn't open CAN port " << robot_can_device_ << "!");
		break;
	}

	read_port();
}

int SocketCANReaderNodelet::open_port(const char *port)
{
	struct ifreq ifr;
	struct sockaddr_can addr;

	/* open socket */
	soc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if(soc_<0)
	{
		return (-1);
	}

	addr.can_family = AF_CAN;
	strcpy(ifr.ifr_name, port);

	if(ioctl(soc_, SIOCGIFINDEX, &ifr) < 0)
	{
		return (-1);
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	fcntl(soc_, F_SETFL, O_NONBLOCK);

	if (bind(soc_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		return (-1);
	}

	return 0;
}

int SocketCANReaderNodelet::send_port(struct can_frame *frame)
{
    int retval;
    retval = write(soc_, frame, sizeof(struct can_frame));

    if (retval != sizeof(struct can_frame))
    {   
        return (-1);
    }
    else
    {
        return (0);
    }
}

void SocketCANReaderNodelet::read_port()
{
	struct can_frame frame_rd;
	int recvbytes = 0;

	can_msgs::Frame can_msg; //ROS CAN frame message
	can_msg.is_extended = false;
	can_msg.is_rtr = false;

	read_can_port_ = true;
	while(read_can_port_)
	{
		struct timeval timeout = {1, 0};
		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(soc_, &readSet);

		if (select((soc_ + 1), &readSet, NULL, NULL, &timeout) >= 0)
		{
			if (!read_can_port_)
			{
				break;
			}
			if (FD_ISSET(soc_, &readSet))
			{
				recvbytes = read(soc_, &frame_rd, sizeof(struct can_frame));
				if(recvbytes)
				{
					//filter out echo frames, RTR request...just take COB id of interest
					int16_t cob_id = 0x0F80 & frame_rd.can_id;

					if ((cob_id == COB_ID_TRANSMIT_PDO_1_ENABLE) ||
						(cob_id == COB_ID_TRANSMIT_PDO_2_ENABLE) ||
						(cob_id == COB_ID_TRANSMIT_PDO_3_ENABLE) ||
						(cob_id == COB_ID_TRANSMIT_PDO_4_ENABLE) ||
						(cob_id == COB_ID_EMCY_DEFAULT) ||
						(cob_id == COB_ID_SDO_SERVER_TO_CLIENT_DEFAULT))
					{
						//build ROS CAN frame
						can_msg.id = frame_rd.can_id;
						can_msg.dlc = frame_rd.can_dlc;
						for(int i=0; i<8; i++) can_msg.data[i] = frame_rd.data[i];

						//publish CAN frame
						pub_rx_can_frame_.publish(can_msg);
					}

					//printf("id = %X, dlc = %d, data = %X %X %X %X\n", frame_rd.can_id, frame_rd.can_dlc,
					//frame_rd.data[0], frame_rd.data[1], frame_rd.data[2], frame_rd.data[3]);
				}
			}
		}
	}
}

int SocketCANReaderNodelet::close_port()
{
    close(soc_);
    return 0;
}

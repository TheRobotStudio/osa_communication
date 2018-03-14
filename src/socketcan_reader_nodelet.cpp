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
SocketCANReaderNodelet::SocketCANReaderNodelet()
{
  open_port("can0");
    read_port();
    return 0;
}

/**
 * @brief Destructor.
 */
SocketCANReaderNodelet::~SocketCANReaderNodelet()
{

}

void SocketCANReaderNodelet::onInit()
{
	ros::NodeHandle nh = this->getPrivateNodeHandle();

	// resolve node(let) name
	std::string name = nh.getUnresolvedNamespace();
	int pos = name.find_last_of('/');
	name = name.substr(pos + 1);

	NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");

	//NODELET_DEBUG("Initializing nodelet...");
}

int open_port(const char *port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0)
    {   
        return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        
        return (-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        
        return (-1);
    }

    return 0;
}

int send_port(struct can_frame *frame)
{
    int retval;
    retval = write(soc, frame, sizeof(struct can_frame));

    if (retval != sizeof(struct can_frame))
    {   
        return (-1);
    }
    else
    {
        return (0);
    }
}

void read_port()
{
    struct can_frame frame_rd;
    int recvbytes = 0;

    read_can_port = 1;
    while(read_can_port)
    {
        struct timeval timeout = {1, 0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);

        if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
        {
            if (!read_can_port)
            {
                break;
            }
            if (FD_ISSET(soc, &readSet))
            {
                recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
                if(recvbytes)
                {
                    //printf("dlc = %d, data = %X\n", frame_rd.can_dlc,frame_rd.data);  
                    printf("id = %X, dlc = %d, data = %X %X %X %X\n", frame_rd.can_id, frame_rd.can_dlc,
                        frame_rd.data[0], frame_rd.data[1], frame_rd.data[2], frame_rd.data[3]);

                        int position = 0;
                        position = frame_rd.data[0] + (frame_rd.data[1]<<8) + (frame_rd.data[2]<<16) + (frame_rd.data[3]<<24);

                        if(frame_rd.can_id == 0x181)
                        {
                                printf("motor position = %d\n", position);
                        }
                }
            }
        }
    }
}

int close_port()
{
    close(soc);
    return 0;
}


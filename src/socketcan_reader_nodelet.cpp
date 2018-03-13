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

#include "socketcan_reader_nodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(osa_communication::SocketCANReaderNodelet, nodelet::Nodelet)

using namespace osa_communication;

/**
 * @brief Constructor.
 */
SocketCANReaderNodelet::SocketCANReaderNodelet()
{

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

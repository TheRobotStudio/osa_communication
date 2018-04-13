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
 *
 *  Created on: Feb 28, 2016
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include "trs_msgs/MotorCmdMultiArray.h"
#include "trs_msgs/MotorDataMultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include <sstream>
#include <string>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				100
#define TIMEOUT 				15//35//25 //20 // 1/(2*HEART_BEAT)
//#define BYTES_PER_MSG			6 //1 nodeID 1 mode 4 data
#define NUMBER_OF_AIN			8 //analog in
#define NUMBER_OF_DIN			8 //digital in
#define BYTES_PER_AIN_DATA		2
#define DATA_SIZE 				NUMBER_OF_AIN*BYTES_PER_AIN_DATA + NUMBER_OF_DIN + 1//+1 for the checksum
#define TOTAL_DATA_SIZE			DATA_SIZE + 4
#define RS232_BAUDRATE			115200 //XBee

/*** Variables ***/
char data[DATA_SIZE];

/*** Functions ***/
bool verifyChecksum(char* dataArray, int nbBytes)
{
	char checksum = 0x00;

	for(int i=0; i<nbBytes; i++)
	{
		checksum += dataArray[i+2]; //+2 for 2 0xAA to skip in the beginning
	}

	checksum++;

	if(checksum == 0x00)
	{
		return true;
	}
	else
	{
		ROS_ERROR("checksum 0x%02X", checksum);
		return false;
	}
}

/*** Main ***/
int main(int argc, char** argv)
{
	// Initialize ROS
    ros::init(argc, argv, "osa_waldo_controller_node");
    ros::NodeHandle nh("~");

    ros::Rate r(LOOP_RATE);

    // Parameters
    std::string usb_device_name;
    // Grab the parameters
    nh.param("usb_device", usb_device_name, std::string("/dev/ttyACM0")); //XBee 2 Pro

    cereal::CerealPort device;
    char reply[TOTAL_DATA_SIZE] = {0x00};
    bool dataValid = false;

	ros::Publisher rigInputs_pub = nh.advertise<std_msgs::UInt16MultiArray>("/waldo_controller_array", 1);
	
	//init data array
	std_msgs::UInt16MultiArray rigData;
	//Clear array
	rigData.data.clear();

    //Change the next line according to your port name and baud rate
	try{ device.open(usb_device_name.c_str(), RS232_BAUDRATE); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port %s at 115200 baud !!!", usb_device_name.c_str());
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : %s at 115200 baud", usb_device_name.c_str());

    //Main loop
    while(ros::ok())
    {
    	rigData.data.clear();

        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try{ device.readBytes(reply, TOTAL_DATA_SIZE, TIMEOUT); } //+4 for 2 0xAA and 2 0xBB
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }

        //check if the data are valid
        if(verifyChecksum(reply, DATA_SIZE)) //only check on data, not delimeters AA BB
        {
        	//ROS_INFO("verifyChecksum OK");

        	//ROS_INFO("reply[0] == %02X", (uint8_t)reply[0]);
        	//chek for delimeters
        	if(((uint8_t)reply[0] == 0xAA) && ((uint8_t)reply[0] == 0xAA) && ((uint8_t)reply[TOTAL_DATA_SIZE-1] == 0xBB) && ((uint8_t)reply[TOTAL_DATA_SIZE-2] == 0xBB))
        	{
        		//ROS_INFO("delimeters OK");
        		dataValid = true; //supposed it is true

        		//check for 8 digital in
        		for(int i=0; i<NUMBER_OF_DIN; i++)
				{
					if(((uint8_t)reply[i+2] != 0) && ((uint8_t)reply[i+2] != 1)) dataValid = false;
				}
        	}
        }

        if(dataValid)
        {
        	//ROS_INFO("Data valid");

        	for(int i=0; i<NUMBER_OF_DIN; i++)
			{
				rigData.data.push_back((uint8_t)reply[i+2]);
			}

        	//build the custom message
        	for(int i=0; i<NUMBER_OF_AIN; i++)
			{
        		rigData.data.push_back((uint8_t)reply[NUMBER_OF_DIN + i*BYTES_PER_AIN_DATA+2] + ((uint8_t)reply[NUMBER_OF_DIN + i*BYTES_PER_AIN_DATA+1+2] << 8));
			}

			rigInputs_pub.publish(rigData);
        	dataValid = false; //reset flag
        }
        else
        {
        	//ROS_ERROR("Data NOT valid");
        }

        device.flush();

        r.sleep();
    }
}

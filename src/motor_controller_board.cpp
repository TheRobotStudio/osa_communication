/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.

 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 * @file main.cpp
 * @author Cyril Jourdan <contact@therobotstudio.com>
 * @date Aug 29, 2017
 * @version 0.1.1
 * @brief Main function for the Open Source Android CAN layer node.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#include "i2c_master.h"

#define NB_DOF 12
#define FIRST_ADDR 8

using namespace osa_communication;

/**
 *  @brief Main function for the Open Source Android CAN layer node.
 *  @param argc
 *  @param argv
 *  @return int
 */
int main(int argc, char **argv)
{
	I2CMaster *i2c_master = new I2CMaster("/dev/i2c-1");

	//Add I2C Slaves
	for(int i=FIRST_ADDR; i<=(FIRST_ADDR+NB_DOF-1); i++)
	{
		bool inverted = false;

		if((i == 12) || (i == 14) || (i == 16)) inverted = true;
		
		i2c_master->addI2CSlave(i, inverted);				
	}
	
	if(i2c_master->init()) return -1;

	return 0;
}

/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <jsoncpp/json/json.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>


namespace drapebot
{
  mqtt_client::mqtt_client(const char *id, const char *host, int port, int keepalive) : mosquittopp(id)
	{
			connect(host, port, keepalive);
	}

	mqtt_client::~mqtt_client()
	{
	}

	void mqtt_client::on_connect(int rc)
	{
			if (!rc)
			{
				ROS_INFO_STREAM( "Connected - code " << rc );
			}
	}

	void mqtt_client::on_subscribe(int mid, int qos_count, const int *granted_qos)
	{
	//         std::cout << "Subscription succeeded." << std::endl;
	}

	void mqtt_client::on_message(const struct mosquitto_message *message)
	{
		
		message_struct* buf = new message_struct;
		memcpy(buf, message->payload, message->payloadlen);
		
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J1: "<< buf->J1);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J2: "<< buf->J2);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J3: "<< buf->J3);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J4: "<< buf->J4);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J5: "<< buf->J5);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback J6: "<< buf->J6);
		ROS_INFO_STREAM_THROTTLE(2.0,"feedback E0: "<< buf->E0);
		
		J1 = buf->J1;
		J2 = buf->J2;
		J3 = buf->J3;
		J4 = buf->J4;
		J5 = buf->J5;
		J6 = buf->J6;
		E0 = buf->E0;
			
	}

}


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

#include <ros/ros.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>

namespace  cnr
{
  namespace drapebot
  {

    void DrapebotMsgDecoder::on_message( const struct mosquitto_message *msg )
    {
      setNewMessageAvailable(true);
		  if ( msg->payloadlen/sizeof(double) == MSG_LENGTH )
      {
        memcpy(&mqtt_msg_, msg->payload, msg->payloadlen);
        data_valid_ = true;
      }
		  else
      {
        ROS_WARN("The message received from MQTT has a wrong length");
        data_valid_ = false;
      }
    }
    
    void DrapebotMsgEncoder::on_publish()
    {
      // Nothing to do here
    }

    MQTTDrapebotClient::MQTTDrapebotClient(const char *id, const char *host, int port, int keepalive)
    {
      cnr::drapebot::DrapebotMsgDecoder* drapebot_msg_decoder = new cnr::drapebot::DrapebotMsgDecoder(mqtt_msg_);
      cnr::drapebot::DrapebotMsgEncoder* drapebot_msg_encoder = new cnr::drapebot::DrapebotMsgEncoder(mqtt_msg_);

      cnr::mqtt::MsgDecoder* msg_decoder = dynamic_cast<cnr::mqtt::MsgDecoder*>(drapebot_msg_decoder);
      cnr::mqtt::MsgEncoder* msg_encoder = dynamic_cast<cnr::mqtt::MsgEncoder*>(drapebot_msg_encoder);

      mqtt_client = new cnr::mqtt::MQTTClient(id, host, port, msg_decoder, msg_encoder);

    }

    MQTTDrapebotClient::~MQTTDrapebotClient()
    {  
      delete drapebot_msg_decoder;
      delete drapebot_msg_encoder;
      delete mqtt_client;
    }

   int MQTTDrapebotClient::stop()
    {
      return mqtt_client->stop();
    }

    int MQTTDrapebotClient::loop()
    {
      return mqtt_client->loop();
    }

    int MQTTDrapebotClient::subscribe(int *mid, const char *sub, int qos)
    {
      return mqtt_client->subscribe(mid, sub, qos);
    }
    
    int MQTTDrapebotClient::unsubscribe(int *mid, const char *sub)
    {
      return mqtt_client->unsubscribe(mid, sub);
    }

    int MQTTDrapebotClient::publish(const uint8_t* payload, const uint32_t& payload_len, const std::string& topic_name)
    {
      return mqtt_client->publish(payload, payload_len, topic_name);
    }

    bool MQTTDrapebotClient::getLastReceivedMessage(cnr::drapebot::drapebot_msg* last_msg)
    {
      if (msg_decoder->isNewMessageAvailable())
      {
        msg_decoder->setNewMessageAvailable(false);
        last_msg = &mqtt_msg_;
        return true;
      }
      else
      {
        ROS_WARN("New message not available.");
        return false;
      }
      return true;
    }

    bool MQTTDrapebotClient::isNewMessageAvailable()
    {
      return msg_decoder->isNewMessageAvailable();
    }

    bool MQTTDrapebotClient::isDataValid()
    {
      return msg_decoder->isDataValid();
    }    

  }
}


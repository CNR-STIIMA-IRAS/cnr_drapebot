/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2022 National Council of Research of Italy (CNR)
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

    void DrapebotMsgDecoder::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
    {
      std::cout << "DrapebotMsgEncoder::on_message" << std::endl;
      setNewMessageAvailable(true);
		  if ( msg->payloadlen/sizeof(double) == MSG_LENGTH )
      {
        memcpy(&mqtt_msg_, msg->payload, msg->payloadlen);
        setDataValid(true);
      }
		  else
      {
        ROS_WARN("The message received from MQTT has a wrong length");
        setDataValid(false);
      }
    }
    
    void DrapebotMsgEncoder::on_publish(struct mosquitto *mosq, void *obj, int mid)
    {
      std::cout << "DrapebotMsgEncoder::on_publish" << std::endl;
      // Nothing to do here
    }

    MQTTDrapebotClient::MQTTDrapebotClient(const char *id, const char *host, int port, int keepalive)
    {
      //drapebot_msg_decoder_ = new cnr::drapebot::DrapebotMsgDecoder(mqtt_msg_);
      //drapebot_msg_encoder_ = new cnr::drapebot::DrapebotMsgEncoder(mqtt_msg_);

      //msg_decoder_ = dynamic_cast<cnr::mqtt::MsgDecoder*>(drapebot_msg_decoder_);
      //msg_encoder_ = dynamic_cast<cnr::mqtt::MsgEncoder*>(drapebot_msg_encoder_);


      cnr::mqtt::MsgDecoder msg_decoder_tmp_;
      cnr::mqtt::MsgEncoder msg_encoder_tmp_;

      msg_decoder_ = &msg_decoder_tmp_;
      msg_encoder_ = &msg_encoder_tmp_;

      if (msg_decoder_ && msg_encoder_ != NULL )
      {
        ROS_ERROR("NULL PTR");
        return;
      }
      else
      {
        ROS_INFO("Step 2");
        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, msg_decoder_, msg_encoder_);
      }
    }

    MQTTDrapebotClient::~MQTTDrapebotClient()
    {  
      delete msg_decoder_;
      delete msg_encoder_;
      delete drapebot_msg_decoder_;
      delete drapebot_msg_encoder_;
      delete mqtt_client_;
    }

   int MQTTDrapebotClient::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      
      else
        return -1;
    }

    int MQTTDrapebotClient::loop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop();
      else
        return -1;
    }

    int MQTTDrapebotClient::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      else
        return -1;
    }
    
    int MQTTDrapebotClient::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);
      else
        return -1;
    }

    int MQTTDrapebotClient::publish(const void* payload, int& payload_len, const char* topic_name)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      else
        return -1;
    }

    bool MQTTDrapebotClient::getLastReceivedMessage(cnr::drapebot::drapebot_msg* last_msg)
    {
      if (msg_decoder_->isNewMessageAvailable())
      {
        msg_decoder_->setNewMessageAvailable(false);
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
      if (msg_decoder_ != NULL)
        return msg_decoder_->isNewMessageAvailable();
      else
        return false;
    }

    bool MQTTDrapebotClient::isDataValid()
    {
      if (msg_decoder_ != NULL)
        return msg_decoder_->isDataValid();
      else
        return false;
    }    

  }
}


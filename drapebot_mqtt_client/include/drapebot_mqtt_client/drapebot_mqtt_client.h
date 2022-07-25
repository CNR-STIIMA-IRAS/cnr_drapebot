
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


#ifndef __DRAPEBOT_MQTT_CLIENT__
#define __DRAPEBOT_MQTT_CLIENT__

#include <mutex>
#include <cnr_mqtt_client/cnr_mqtt_client.h>

#define MAX_PAYLOAD_SIZE 1024
#define MSG_LENGTH 7
#define DEFAULT_KEEP_ALIVE 60

namespace cnr
{
  namespace drapebot
  {

    struct drapebot_msg 
    {
      double joints_values_[7];    
    }; 

    class DrapebotMsgDecoder: public cnr::mqtt::MsgDecoder
    {
    public:
      DrapebotMsgDecoder(cnr::drapebot::drapebot_msg& mqtt_msg): mqtt_msg_(mqtt_msg) {};
      // The method should be reimplemented on the base of the application
      void on_message(const struct mosquitto_message *msg);
    private:
      cnr::drapebot::drapebot_msg mqtt_msg_;
    };

    class DrapebotMsgEncoder: public cnr::mqtt::MsgEncoder
    {
    public:
      DrapebotMsgEncoder(cnr::drapebot::drapebot_msg& mqtt_msg): mqtt_msg_(mqtt_msg) {};
      // The method should be reimplemented on the base of the application
      void on_publish();
    private:
      cnr::drapebot::drapebot_msg mqtt_msg_;
    };


    struct drapebot_message 
    {
      double joints_values_[MSG_LENGTH] = {0};    
    };   

    class MQTTDrapebotClient
    {
    public:
      MQTTDrapebotClient (const char *id, const char *host, int port, int keepalive = 60);
      ~MQTTDrapebotClient();

      int stop();
      int loop();
      // int reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff);  
      int subscribe(int *mid, const char *sub, int qos);
      int unsubscribe(int *mid, const char *sub);
      int publish(const uint8_t* payload, const uint32_t& payload_len, const std::string& topic_name);
     

      bool getLastReceivedMessage(cnr::drapebot::drapebot_msg* last_msg);
      bool isNewMessageAvailable();
      bool isDataValid();    

    private:
      cnr::drapebot::DrapebotMsgDecoder* drapebot_msg_decoder;
      cnr::drapebot::DrapebotMsgEncoder* drapebot_msg_encoder;

      cnr::mqtt::MsgDecoder* msg_decoder;
      cnr::mqtt::MsgEncoder* msg_encoder; 

      drapebot_msg mqtt_msg_;      
      std::mutex mtx_mqtt_;  
      cnr::mqtt::MQTTClient* mqtt_client;

    };
  }

}
#endif

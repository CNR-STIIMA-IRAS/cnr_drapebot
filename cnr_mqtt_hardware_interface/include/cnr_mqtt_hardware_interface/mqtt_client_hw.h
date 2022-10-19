
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


#ifndef __DRAPEBOT_MQTT_CLIENT__
#define __DRAPEBOT_MQTT_CLIENT__

#include <mutex>
#include <cnr_mqtt_client/cnr_mqtt_client.h>

#define MSG_LENGTH 7 // The length is given by 6 axes robot + linear axis

namespace cnr
{
  namespace drapebot
  {
    struct drapebot_msg_hw 
    {
//       double joints_values_[MSG_LENGTH] = {0};
      double J1;
      double J2;
      double J3;
      double J4;
      double J5;
      double J6;
      double E0;
      unsigned long int count;
    }; 


    class DrapebotMsgDecoderHw: public cnr::mqtt::MsgDecoder
    {
    public:
      DrapebotMsgDecoderHw(cnr::drapebot::drapebot_msg_hw* mqtt_msg): mqtt_msg_(mqtt_msg) {};
      
      // The method should be reimplemented on the base of the application
      void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) override;
    private:
      cnr::drapebot::drapebot_msg_hw* mqtt_msg_;
      void vec_to_msg(std::vector<double> v, cnr::drapebot::drapebot_msg_hw* msg);
    };

    class DrapebotMsgEncoderHw: public cnr::mqtt::MsgEncoder
    {
    public:
      DrapebotMsgEncoderHw(cnr::drapebot::drapebot_msg_hw* mqtt_msg): mqtt_msg_(mqtt_msg) {};
      
      // The method should be reimplemented on the base of the application
      void on_publish(struct mosquitto *mosq, void *obj, int mid) override;
    private:
      cnr::drapebot::drapebot_msg_hw* mqtt_msg_;
    };

    class MQTTDrapebotClientHw
    {
    public:
      MQTTDrapebotClientHw (const char *id, const char *host, int port, int keepalive = 60);
      ~MQTTDrapebotClientHw();

      int stop();
      int loop();
      // int reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff);  
      int subscribe(int *mid, const char *sub, int qos);
      int unsubscribe(int *mid, const char *sub);
      int publish(const void* payload, int& payload_len, const char* topic_name);
      void publish_with_tracking(const std::string& cmd_topic, drapebot_msg_hw& m);
     
      bool getLastReceivedMessage(cnr::drapebot::drapebot_msg_hw& last_msg);
      bool isNewMessageAvailable();
      bool isDataValid();    

      int get_msg_count_cmd(){return msg_count_cmd;};
      void set_msg_count_cmd(const int& count){msg_count_cmd = count;};
      
      bool getFirstMessageStatus();
      
      
      cnr::drapebot::drapebot_msg_hw* mqtt_msg_enc_;
      cnr::drapebot::drapebot_msg_hw* mqtt_msg_dec_;

    private:
      std::mutex mtx_mqtt_;  
      
      unsigned long int msg_count_cmd;

      bool first_message_received_;
      
      cnr::drapebot::DrapebotMsgDecoderHw* drapebot_msg_hw_decoder_;
      cnr::drapebot::DrapebotMsgEncoderHw* drapebot_msg_hw_encoder_;


      cnr::mqtt::MQTTClient* mqtt_client_;
    };
  }

}



#endif

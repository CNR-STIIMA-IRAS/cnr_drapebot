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

#ifndef cnr_drapebot__DRAPEBOT_MQTT_CLIENT_H
#define cnr_drapebot__DRAPEBOT_MQTT_CLIENT_H

#include <ros/ros.h>
#include <mutex>
#include <ctime>
#include <chrono>
#include <fstream>

#include <cnr_mqtt_client/cnr_mqtt_client.h>
#include <jsoncpp/json/json.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#define MAX_PAYLOAD 50
#define DEFAULT_KEEP_ALIVE 60


namespace cnr
{
  namespace drapebot_converter
  {
    class MsgDecoder: public cnr::mqtt::MsgDecoder
    {
    public:
      MsgDecoder(control_msgs::FollowJointTrajectoryGoal* trajectory_msg): trajectory_msg_(trajectory_msg){};
      void on_message(const struct mosquitto_message *msg) override;

      control_msgs::FollowJointTrajectoryGoal *trajectory_msg_;
      bool cooperative_;
      void setJointParams(const std::vector<std::string>& joint_names);

    private:
      control_msgs::FollowJointTrajectoryGoal transform_trajectory(Json::Value traj);
      bool JsonToMsg(Json::Value traj, control_msgs::FollowJointTrajectoryGoal *msg);
      std::vector<std::string> joint_names_;
      int n_joints_;

    };

    class MsgEncoder: public cnr::mqtt::MsgEncoder
    {
    public:
      MsgEncoder() {};
      void on_publish(int mid) override;

    private:
    };

    class MQTTClient
    {
    public:
      MQTTClient (const char *id, const char *host, const int port, const bool use_json = true, int keepalive = 60);
      ~MQTTClient();

      int stop();
      int loop(int timeout=2000);
      int reconnect();  
      int subscribe(int *mid, const char *sub, int qos);
      int unsubscribe(int *mid, const char *sub);
      int publish(const void* payload, int& payload_len, const char* topic_name);
     
      //bool isFirstMsgRec(){ return msg_decoder_->isFirstMsgRec(); };

      bool getLastReceivedMessage(control_msgs::FollowJointTrajectoryGoal& last_msg, bool& cooperative);
      bool isNewMessageAvailable();
      bool isDataValid();    
      
      bool isTrajCooperative();    
      
      bool cooperative_;
      
      void set_joint_names(std::vector<std::string> jn);
      void set_config(const std::string& config);
      std::string get_config();

      control_msgs::FollowJointTrajectoryGoal* mqtt_traj_msg_dec_;

      int n_joints_;  
      std::vector<std::string> joint_names_;

    private:
      std::mutex mtx_mqtt_;  
      
      unsigned long int msg_count_cmd;

      bool first_message_received_;
      std::string configuration_;
      
      cnr::drapebot_converter::MsgDecoder* msg_decoder_;
      cnr::drapebot_converter::MsgEncoder* msg_encoder_;


      cnr::mqtt::MQTTClient* mqtt_client_;

    };
  }

}
#endif

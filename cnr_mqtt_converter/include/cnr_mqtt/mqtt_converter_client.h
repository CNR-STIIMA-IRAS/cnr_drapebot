#pragma once

#include <ros/ros.h>
#include <mutex>
#include <cnr_mqtt_client/cnr_mqtt_client.h>
#include <jsoncpp/json/json.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#define MAX_PAYLOAD 50
#define DEFAULT_KEEP_ALIVE 60


namespace cnr
{
  namespace drapebot_converter
  {
    class DrapebotMsgDecoderHw: public cnr::mqtt::MsgDecoder
    {
    public:
      DrapebotMsgDecoderHw(control_msgs::FollowJointTrajectoryGoal* trajectory_msg): trajectory_msg_(trajectory_msg){};
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

    class DrapebotMsgEncoderHw: public cnr::mqtt::MsgEncoder
    {
    public:
      DrapebotMsgEncoderHw() {};
      void on_publish(int mid) override;
    private:
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
      
      cnr::drapebot_converter::DrapebotMsgDecoderHw* drapebot_msg_hw_decoder_;
      cnr::drapebot_converter::DrapebotMsgEncoderHw* drapebot_msg_hw_encoder_;


      cnr::mqtt::MQTTClient* mqtt_client_;
    };
  }

}


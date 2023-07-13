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

static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";

#include <cnr_mqtt_converter/mqtt_converter_client.h>


namespace  cnr
{
  namespace drapebot_converter
  {



    bool MsgDecoder::JsonToMsg(Json::Value traj, control_msgs::FollowJointTrajectoryGoal *msg)
    {
      
      //TODO:: definire con Gottardi parametro per settare configuratione deformation/non deformation
      //set_config(traj["configuration"])
      
      
      msg->trajectory.points.resize(traj.size()-1); //TODO: the -1 is added because the "collaborative" field is considered as additional traj point
      msg->trajectory.joint_names.resize(joint_names_.size());
      msg->trajectory.header.frame_id = joint_names_[0];
      
      for(int i=0; i<joint_names_.size();i++)
      {
        msg->trajectory.joint_names.at(i) = joint_names_.at(i);
        ROS_DEBUG_STREAM(msg->trajectory.joint_names.back());
      }
            
      for (int i=0; i<msg->trajectory.points.size();i++)
      {
        std::string P = "P"+  std::to_string((i+1));
        ros::Duration time_from_start( std::stod( traj[P]["time"].asString() ));
        
        msg->trajectory.points[i].time_from_start = time_from_start;
        
        msg->trajectory.points[i].positions.resize(n_joints_);
        msg->trajectory.points[i].velocities.resize(n_joints_);
        msg->trajectory.points[i].accelerations.resize(n_joints_);
        
        for(int jj=0;jj<n_joints_;jj++)
        {
          std::string jn = "J";
          jn+= std::to_string(jj);
          double jp = traj[P][jn]["value"].asDouble() ;
          double jv = traj[P][jn]["velocity"].asDouble() ;
          
          msg->trajectory.points[i].positions[jj] = jp;
          msg->trajectory.points[i].velocities[jj]= jv;
          msg->trajectory.points[i].accelerations[jj]=0;
        }
      }
      
      bool coop = traj["collaborative"].asBool();
      
      ROS_INFO_STREAM(CYAN<<"coop : "<<coop);
      
      try
      {
      ROS_INFO_STREAM(GREEN<<"cooperative : "<<cooperative_);
        cooperative_ = coop;
      ROS_INFO_STREAM(GREEN<<"cooperative : "<<cooperative_);
        
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in cooperative coop: " <<  e.what() );
      }


      ROS_DEBUG_STREAM(YELLOW<<"goal trajectory: \n"<< *msg);
      
      return true;
    }
    
    
    void MsgDecoder::on_message(const struct mosquitto_message *msg)
    {
      char buf[msg->payloadlen];
      memcpy(buf, msg->payload, msg->payloadlen);
      
      Json::Reader reader;
      Json::Value root;
      
      reader.parse(buf,root);
      
      ROS_DEBUG_STREAM(GREEN<<"Json Trajectory from ITR MotionPlanner: \n" << buf);
      ROS_DEBUG_STREAM(YELLOW<<"root from ITR MotionPlanner: \n" << root);
      
      if ( !JsonToMsg(root, trajectory_msg_))
      {
        ROS_ERROR("error in the conversion");
        setNewMessageAvailable(false);
        setDataValid(false);
      }
      else
      {
        ROS_INFO("conversion ok");      
        setNewMessageAvailable(true); 
        setDataValid(true);
      }
      
    }
    
    void MsgDecoder::setJointParams(const std::vector<std::string>& joint_names)
    {
      n_joints_ = joint_names.size();
      joint_names_.resize(n_joints_);
      joint_names_ = joint_names;
      ROS_INFO_STREAM("joint nuimber: "<<n_joints_ );
      for (auto n:joint_names_)
        ROS_INFO_STREAM("joint : "<<n);
    }
    
    
    void MsgEncoder::on_publish(int mid)
    {
      // Nothing to do here
    }
    

    MQTTClient::MQTTClient(const char *id, const char *host, const int port, const bool use_json, int keepalive)
    {
      try
      {
        mqtt_traj_msg_dec_ = new control_msgs::FollowJointTrajectoryGoal;
        

        msg_encoder_ = new cnr::drapebot_converter::MsgEncoder();
        msg_decoder_ = new cnr::drapebot_converter::MsgDecoder(mqtt_traj_msg_dec_);

        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, msg_encoder_, msg_decoder_);
        
        msg_count_cmd = 0;
        first_message_received_ = false;
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in MqttClient constructor: " <<  e.what() );
      }
    }

    MQTTClient::~MQTTClient()
    {  
      delete mqtt_traj_msg_dec_;
      delete msg_decoder_;
      delete msg_encoder_;
      delete mqtt_client_;
    }

    int MQTTClient::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      

      return -1;
    }

    int MQTTClient::loop(int timeout)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop(timeout);
      
      return -1;
    }

    int MQTTClient::reconnect()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->reconnect();
      
      return -1;
    }

    int MQTTClient::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      
      return -1;
    }
    
    int MQTTClient::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);

      return -1;
    }

    int MQTTClient::publish(const void* payload, int& payload_len, const char* topic_name)
    {        
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      
      return -1;
    }

    bool MQTTClient::getLastReceivedMessage(control_msgs::FollowJointTrajectoryGoal& last_msg, bool& cooperative)
    {
      if (msg_decoder_->isNewMessageAvailable() )
      {
        ROS_DEBUG_STREAM(*mqtt_traj_msg_dec_);
        last_msg = *mqtt_traj_msg_dec_;
        
        cooperative_ = msg_decoder_->cooperative_;
        cooperative = cooperative_;
        msg_decoder_->setNewMessageAvailable(false);
        return true;
      }
      else
      {
        ROS_WARN("New msg not available.");
        return false;
      }
      return true;
    }

    bool MQTTClient::isNewMessageAvailable()
    {
      if (msg_decoder_ != NULL)
        return msg_decoder_->isNewMessageAvailable();
      else
      {
        ROS_ERROR("msg_decoder_ == NULL . return false");
        return false;
      }
    }

    bool MQTTClient::isDataValid()
    {
      if (msg_decoder_ != NULL)
        return msg_decoder_->isDataValid();
      else
        return false;
    }

    bool MQTTClient::isTrajCooperative()
    {
      return cooperative_;
    }
    
    void MQTTClient::set_joint_names(std::vector<std::string> jn)
    {
      for(auto n : jn)
      {
        joint_names_.push_back(n);
        ROS_DEBUG_STREAM(n);
      }
      n_joints_ = joint_names_.size();
      
      ROS_DEBUG_STREAM("joint nuimber: "<<n_joints_ );
      
      msg_decoder_->setJointParams(joint_names_);
      
    }

    void MQTTClient::set_config(const std::string& config)
    {
      configuration_ = config;
    }

    std::string MQTTClient::get_config()
    {
      return configuration_;
    }
    
  }
}










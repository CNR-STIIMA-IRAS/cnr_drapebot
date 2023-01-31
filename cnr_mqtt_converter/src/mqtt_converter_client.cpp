#include "cnr_mqtt/mqtt_converter_client.h"

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

namespace  cnr
{
  namespace drapebot_converter
  {
    
    void DrapebotMsgDecoderHw::on_message(const struct mosquitto_message *message)
    {
      char buf[message->payloadlen];
      memcpy(buf, message->payload, message->payloadlen);
      
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
    
    bool DrapebotMsgDecoderHw::JsonToMsg(Json::Value traj, control_msgs::FollowJointTrajectoryGoal *msg)
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
      try{
      *cooperative_ = coop;
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in cooperative coop: " <<  e.what() );
      }


      ROS_INFO_STREAM("deformation_address : "<<cooperative_);
      ROS_INFO_STREAM("deformation_active_ : "<<*cooperative_);
      
      ROS_DEBUG_STREAM(YELLOW<<"goal trajectory: \n"<< *msg);
      
      return true;
    }
    
    void DrapebotMsgDecoderHw::setJointParams(const std::vector<std::string>& joint_names)
    {
      n_joints_ = joint_names.size();
      joint_names_.resize(n_joints_);
      joint_names_ = joint_names;
      ROS_INFO_STREAM("joint nuimber: "<<n_joints_ );
      for (auto n:joint_names_)
        ROS_INFO_STREAM("joint : "<<n);
    }
    
    
    void DrapebotMsgEncoderHw::on_publish(int mid)
    {
      // Nothing to do here
    }
    


    
    
    
    
    
    
    
    MQTTDrapebotClientHw::MQTTDrapebotClientHw(const char *id, const char *host, int port, int keepalive)
    {
      try
      {
        mqtt_traj_msg_dec_ = new control_msgs::FollowJointTrajectoryGoal;
        
        ROS_INFO_STREAM(cooperative_);

        drapebot_msg_hw_encoder_ = new cnr::drapebot_converter::DrapebotMsgEncoderHw();
        drapebot_msg_hw_decoder_ = new cnr::drapebot_converter::DrapebotMsgDecoderHw(mqtt_traj_msg_dec_,cooperative_);

        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, drapebot_msg_hw_encoder_, drapebot_msg_hw_decoder_);
        
        msg_count_cmd = 0;
        first_message_received_ = false;
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in MQTTDrapebotClientHw constructor: " <<  e.what() );
      }
    }

    MQTTDrapebotClientHw::~MQTTDrapebotClientHw()
    {  
      delete mqtt_traj_msg_dec_;
      delete drapebot_msg_hw_decoder_;
      delete drapebot_msg_hw_encoder_;
      delete mqtt_client_;
    }

    int MQTTDrapebotClientHw::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      
      else
        return -1;
    }

    int MQTTDrapebotClientHw::loop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop();
      else
        return -1;
    }

    int MQTTDrapebotClientHw::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      else
        return -1;
    }
    
    int MQTTDrapebotClientHw::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);
      else
        return -1;
    }

    int MQTTDrapebotClientHw::publish(const void* payload, int& payload_len, const char* topic_name)
    {        
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      else
        return -1;
    }
    bool MQTTDrapebotClientHw::getLastReceivedMessage(control_msgs::FollowJointTrajectoryGoal& last_msg)
    {
      if (drapebot_msg_hw_decoder_->isNewMessageAvailable() )
      {
        ROS_DEBUG_STREAM(*mqtt_traj_msg_dec_);
        last_msg = *mqtt_traj_msg_dec_;
        
        drapebot_msg_hw_decoder_->setNewMessageAvailable(false);
        return true;
      }
      else
      {
        ROS_WARN("New message not available.");
        return false;
      }
      return true;
    }

    bool MQTTDrapebotClientHw::isNewMessageAvailable()
    {
      if (drapebot_msg_hw_decoder_ != NULL)
        return drapebot_msg_hw_decoder_->isNewMessageAvailable();
      else
      {
        ROS_ERROR("drapebot_msg_hw_decoder_ == NULL . return false");
        return false;
      }
    }

    bool MQTTDrapebotClientHw::isDataValid()
    {
      if (drapebot_msg_hw_decoder_ != NULL)
        return drapebot_msg_hw_decoder_->isDataValid();
      else
        return false;
    }

    bool MQTTDrapebotClientHw::isTrajCooperative()
    {
      try
      {
        ROS_INFO_STREAM(CYAN<<*cooperative_);
        return *cooperative_;
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in isTrajCooperative: " <<  e.what() );
      }
      return false;
    }
    
    void MQTTDrapebotClientHw::set_joint_names(std::vector<std::string> jn)
    {
      for(auto n : jn)
      {
        joint_names_.push_back(n);
        ROS_DEBUG_STREAM(n);
      }
      n_joints_ = joint_names_.size();
      
      ROS_DEBUG_STREAM("joint nuimber: "<<n_joints_ );
      
      drapebot_msg_hw_decoder_->setJointParams(joint_names_);
      
    }

    void MQTTDrapebotClientHw::set_config(const std::string& config)
    {
      configuration_ = config;
    }

    std::string MQTTDrapebotClientHw::get_config()
    {
      return configuration_;
    }
    
  }
}










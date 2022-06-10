#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <mqtt_to_joint_position_controller/mqtt_to_joint_position_controller.h>


namespace drapebot_controller
{
  
  MQTTToPositionController::MQTTToPositionController()
  {

  }

  MQTTToPositionController::~MQTTToPositionController()
  {
    delete mqtt_client_;  
  }
//   bool MQTTToPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
//   {
//     return this->init(hw,controller_nh);    
//   }

  bool MQTTToPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n)
  {
    ctrl_.init(hw,n); 

    // ---- MQTT params ----

    std::string cid;
    if (!n.getParam("client_id",cid))
    {
      cid = "Client_ID";
      ROS_WARN_STREAM("client id not found under " + n.getNamespace() + "/client_id . Using defalut client ID: " + cid);
    }

    int n_id = cid.length();
    char client_id[n_id + 1];
    strcpy(client_id, cid.c_str());
    
    std::string host_str;
    if (!n.getParam("broker_address",host_str))
    {
      host_str = "localhost";
      ROS_WARN_STREAM("broker_address not found under " + n.getNamespace() + "/broker_address . Using defalut broker address: "+ host_str);
    }

    int n_host = host_str.length();
    char host[n_host + 1];
    strcpy(host, host_str.c_str());
    
    int port;
    if (!n.getParam("port",port))
    {
      port = 1883;
      ROS_WARN_STREAM("port not found under " + n.getNamespace() + "/port. Using defalut broker address: "+ std::to_string( port));      
    }
    
    if (!n.getParam("mqtt_command_topic",mqtt_command_topic_))
    {
      mqtt_command_topic_ = "mqtt_command_topic";
      ROS_WARN_STREAM("mqtt_command_topic not found under " + n.getNamespace() + "/mqtt_command_topic . Using defalut broker address: "+ mqtt_command_topic_);      
    }
    
    mosquitto_lib_init();
    
    ROS_WARN_STREAM("Connencting mqtt: "<< client_id << ": " <<host);
    mqtt_client_ = new drapebot::MQTTClient(client_id, host, port);
    ROS_ERROR_STREAM("Connencted to: "<< client_id << ": " <<host);
    
    mosqpp::lib_init();
    
    j_pos_command_.resize(sizeof(mqtt_client_->msg_.joints_values_)/sizeof(double));
    j_pos_command_  = *ctrl_.commands_buffer_.readFromNonRT();
    
    ROS_FATAL_STREAM("mqtt_command_topic : "<< mqtt_command_topic_);
    
    drapebot::message_struct tmp_j_pos_feedback;
    
    tmp_j_pos_feedback.linear_axis_value_ = 0;
    
    void* payload_ = malloc( sizeof(tmp_j_pos_feedback) );
    
    memcpy(payload_, &tmp_j_pos_feedback, sizeof(tmp_j_pos_feedback));  
    
    first_cycle_ = true;
    
    ROS_WARN_STREAM("Controller MQTTToPositionController Initialized ! ");
    
    return true;
  }
  

  void MQTTToPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    mqtt_client_->loop();
    
    if (first_cycle_)
    {
      first_cycle_ = false;
      j_pos_command_  = *ctrl_.commands_buffer_.readFromNonRT();
    }
      
    // Read the new MQTT message and send the command to the robot
      
    if (mqtt_client_->is_new_message_available() && mqtt_client_->is_data_valid())
    {
      for (size_t i=0; i<sizeof(mqtt_client_->msg_.joints_values_)/sizeof(double); i++)
      {
        j_pos_command_[i] =  mqtt_client_->msg_.joints_values_[i];
      }
    }
    else
      ROS_WARN_THROTTLE(10.0,"no new msg available");
    
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_0 : "<< j_pos_command_[0]);
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_1 : "<< j_pos_command_[1]);
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_2 : "<< j_pos_command_[2]);
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_3 : "<< j_pos_command_[3]);
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_4 : "<< j_pos_command_[4]);
    ROS_FATAL_STREAM_THROTTLE(5.0,"joint command_5 : "<< j_pos_command_[5]);
    
    ctrl_.commands_buffer_.writeFromNonRT(j_pos_command_);
    ctrl_.update(time,period);
    
  }
    
  
  
  void MQTTToPositionController::starting(const ros::Time& time)
  { 
    ctrl_.starting(time);
    char topic_command[mqtt_command_topic_.size() + 1];
    strcpy(topic_command, mqtt_command_topic_.c_str());
    mqtt_client_->subscribe(NULL, topic_command);
    ROS_FATAL_STREAM("subscribing: "<< topic_command);
  }

  void MQTTToPositionController::stopping(const ros::Time& time) 
  {
    ctrl_.stopping(time);
    char topic_command[mqtt_command_topic_.size() + 1];
    strcpy(topic_command, mqtt_command_topic_.c_str());
    mqtt_client_->unsubscribe(NULL, topic_command);
    ROS_FATAL_STREAM("UNSUBSCRIBINg: "<< topic_command);
  }
  
  void MQTTToPositionController::waiting(const ros::Time& time)
  {
    ctrl_.waiting(time);
  }
  void MQTTToPositionController::aborting(const ros::Time& time) 
  {
    ctrl_.aborting(time);
  }
}

PLUGINLIB_EXPORT_CLASS(drapebot_controller::MQTTToPositionController, controller_interface::ControllerBase)


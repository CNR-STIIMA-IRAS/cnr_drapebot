#include <string>
#include <jsoncpp/json/json.h>

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

  bool MQTTToPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n)
  {
      // get joint name from the parameter server
      std::string my_joint;

      std::string jnt_namespace = "/egm/joint_group_mqtt_to_position_controller/joints";
      std::vector<std::string> joints_names; 
      if (!n.getParam(jnt_namespace, joints_names))
      {
        ROS_ERROR("Could not find the namespace.");
        return false;
      }

      if (joints_names.size() <= 6)
      {
        ROS_ERROR("The number of axis expected is 6");
        return false;
      }

      for (const std::string& jnt_name : joints_names)
        // get the joint object to use in the realtime loop
        joints_handle_.push_back( hw->getHandle(jnt_name) );  // throws on failure

      j_pos_feedback_.resize( sizeof(mqtt_client_->msg_)/sizeof(double) ); 

      // ---- MQTT params ----

      std::string cid;
      if (!n.getParam("client_id",cid))
      {
        cid = "Client_ID";
        ROS_WARN("Using defalut client ID: Client_ID");
      }

      int n_id = cid.length();
      char client_id[n_id + 1];
      strcpy(client_id, cid.c_str());
      
      std::string host_str;
      if (!n.getParam("host",host_str))
      {
        host_str = "localhost";
        ROS_WARN("Using defalut host: localhost");
      }

      int n_host = host_str.length();
      char host[n_host + 1];
      strcpy(host, host_str.c_str());
      
      int port;
      if (!n.getParam("port",port))
      {
        port = 1883;
        ROS_WARN("Using defalut port: 1883");
      }
      
      if (!n.getParam("command_mqtt_topic",mqtt_command_topic_))
      {
        mqtt_command_topic_ = "mqtt_command_topic";
        ROS_WARN("Using topic out : mqtt_command_topic");
      }
      
      mosquitto_lib_init();
      
      ROS_WARN_STREAM("Connencting mqtt: "<< client_id << ": " <<host);
      mqtt_client_ = new drapebot::MQTTClient(client_id, host, port);
      ROS_INFO_STREAM("Connencted to: "<< client_id << ": " <<host);
      
      mosqpp::lib_init();
      
      if (!n.getParam("feedback_mqtt_topic",mqtt_feedback_topic_))
      {
        mqtt_feedback_topic_ = "mqtt_feedback_topic";
        ROS_WARN("Using topic out: mqtt_feedback_topic");
      }
      
      if (!n.getParam("out_feedback_mqtt_topic",mqtt_out_feedback_topic_))
      {
        mqtt_out_feedback_topic_ = "out_mqtt_feedback_topic";
        ROS_WARN("Using topic out: out_mqtt_feedback_topic");
      }
      
      size_t feedback_topic_size = mqtt_feedback_topic_.size();
      char topic[feedback_topic_size + 1];
      strcpy(topic, mqtt_feedback_topic_.c_str());
      mqtt_client_->subscribe(NULL, topic);

      for(size_t idx=0; idx<sizeof(mqtt_client_->msg_)/sizeof(double); idx++)
        mqtt_client_->msg_.joints_values_[idx] = j_pos_feedback_[idx];
      
      return true;
  }

  void MQTTToPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    mqtt_client_->loop();

    // Read the new MQTT message and send the command to the robot
    std::vector<double> jnt_setpoint(sizeof(mqtt_client_->msg_)/sizeof(double));

    for (size_t idx=0; idx<sizeof(mqtt_client_->msg_)/sizeof(double); idx++)
      jnt_setpoint.at(idx) = mqtt_client_->msg_.joints_values_[idx];  
 
    for( size_t idx=0; idx<joints_handle_.size(); idx++)
      joints_handle_.at(idx).setCommand(jnt_setpoint.at(idx+1));
 
    // Write a feedback
    
    // Json::Value root;
    // Json::FastWriter writer;
  
    // root["J0"]["current_value"] = m_client->J1;
    // root["J1"]["current_value"] = m_client->J2;
    // root["J2"]["current_value"] = m_client->J3;
    // root["J3"]["current_value"] = m_client->J4;
    // root["J4"]["current_value"] = m_client->J5;
    // root["J5"]["current_value"] = m_client->J6;
    // root["J6"]["current_value"] = m_client->E0;
    
    // root["J0"]["command_value"] = m_cmd_pos.at(1);
    // root["J1"]["command_value"] = m_cmd_pos.at(2);
    // root["J2"]["command_value"] = m_cmd_pos.at(3);
    // root["J3"]["command_value"] = m_cmd_pos.at(4);
    // root["J4"]["command_value"] = m_cmd_pos.at(5);
    // root["J5"]["command_value"] = m_cmd_pos.at(6);
    // root["J6"]["command_value"] = m_cmd_pos.at(0);
    
    // root["J0"]["reference_value"] = m_nom_traj.at(1);
    // root["J1"]["reference_value"] = m_nom_traj.at(2);
    // root["J2"]["reference_value"] = m_nom_traj.at(3);
    // root["J3"]["reference_value"] = m_nom_traj.at(4);
    // root["J4"]["reference_value"] = m_nom_traj.at(5);
    // root["J5"]["reference_value"] = m_nom_traj.at(6);
    // root["J6"]["reference_value"] = m_nom_traj.at(0);
  
    // Json::StreamWriterBuilder builder;
    // const std::string json_file = Json::writeString(builder, root);
      
    // int n = m_mqtt_out_feedback_topic.length();
    // char topic[n+ 1];
    // strcpy(topic, m_mqtt_out_feedback_topic.c_str());
    
    // int len = json_file.length();
    // char pl[len+1];
    // strcpy(pl, json_file.c_str());
    // m_client->publish(NULL, topic, sizeof(pl), pl);
  
  }

  void MQTTToPositionController::starting(const ros::Time& time)
  { 

  }

  void MQTTToPositionController::stopping(const ros::Time& time) 
  {

  }
}

PLUGINLIB_EXPORT_CLASS(drapebot_controller::MQTTToPositionController, controller_interface::ControllerBase)


#include "egm_to_mqtt_state_ctrl/egm_to_mqtt_state_ctrl.h"
#include <algorithm>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>

#include <jsoncpp/json/json.h>

#include "joint_state_controller/joint_state_controller.h"

namespace drapebot_controller
{

  bool EgmToMQTTCtrl::init(hardware_interface::JointStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    // List of joints to be published
    std::vector<std::string> joint_names;

    // Get list of joints: This allows specifying a desired order, or
    // alternatively, only publish states for a subset of joints. If the
    // parameter is not set, all joint states will be published in the order
    // specified by the hardware interface.
    if (controller_nh.getParam("joints", joint_names)) {
      ROS_INFO_STREAM("Joints parameter specified, publishing specified joints in desired order.");
    } else {
      // get all joint names from the hardware interface
      joint_names = hw->getNames();
    }

    num_hw_joints_ = joint_names.size();
    for (unsigned i=0; i<num_hw_joints_; i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

    // get joints and allocate message
    for (unsigned i=0; i<num_hw_joints_; i++){
      joint_state_.push_back(hw->getHandle(joint_names[i]));
      realtime_pub_->msg_.name.push_back(joint_names[i]);
      realtime_pub_->msg_.position.push_back(0.0);
      realtime_pub_->msg_.velocity.push_back(0.0);
      realtime_pub_->msg_.effort.push_back(0.0);
    }
    addExtraJoints(controller_nh, realtime_pub_->msg_);
    
    
    
    
    
    // ---- MQTT params ----
    std::string cid;
    if (!controller_nh.getParam("client_id",cid))
    {
      cid = "Client_ID";
      ROS_WARN_STREAM("client id not found under " + controller_nh.getNamespace() + "/client_id . Using defalut client ID: " + cid);
    }

    int n_id = cid.length();
    char client_id[n_id + 1];
    strcpy(client_id, cid.c_str());
    
    std::string host_str;
    if (!controller_nh.getParam("broker_address",host_str))
    {
      host_str = "localhost";
      ROS_WARN_STREAM("broker_address not found under " + controller_nh.getNamespace() + "/broker_address . Using defalut broker address: "+ host_str);
    }

    int n_host = host_str.length();
    char host[n_host + 1];
    strcpy(host, host_str.c_str());
    
    int port;
    if (!controller_nh.getParam("port",port))
    {
      port = 1883;
      ROS_WARN_STREAM("port not found under " + controller_nh.getNamespace() + "/port. Using defalut broker address: "+ std::to_string( port));      
    }    
    
    mosquitto_lib_init();
    
    ROS_WARN_STREAM("Connencting mqtt: "<< client_id << ": " <<host);
    mqtt_client_ = new drapebot::MQTTClient(client_id, host, port);
    ROS_ERROR_STREAM("Connencted to: "<< client_id << ": " <<host);
    
    mosqpp::lib_init();
    
    if (!controller_nh.getParam("mqtt_feedback_topic",mqtt_feedback_topic_))
    {
      mqtt_feedback_topic_ = "mqtt_feedback_topic";
      ROS_WARN_STREAM("mqtt_feedback_topic not found under " + controller_nh.getNamespace() + "/mqtt_feedback_topic . Using defalut broker address: "+ mqtt_feedback_topic_);  
    }
    
    return true;
  }

  void EgmToMQTTCtrl::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void EgmToMQTTCtrl::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      if (realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message:
        // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
        // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<num_hw_joints_; i++){
          realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
          realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
          realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
        }
        realtime_pub_->unlockAndPublish();
      }
    }
    
    
    drapebot::message_struct tmp_j_pos_feedback;
    
    for (unsigned i=0; i<num_hw_joints_; i++)
      tmp_j_pos_feedback.joints_values_[i] = joint_state_[i].getPosition();
    
    tmp_j_pos_feedback.linear_axis_value_ = 0;
    
    
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_1 : " << tmp_j_pos_feedback.joints_values_[0]);
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_2 : " << tmp_j_pos_feedback.joints_values_[1]);
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_3 : " << tmp_j_pos_feedback.joints_values_[2]);
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_4 : " << tmp_j_pos_feedback.joints_values_[3]);
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_5 : " << tmp_j_pos_feedback.joints_values_[4]);
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state Joint_6 : " << tmp_j_pos_feedback.joints_values_[5]);  
    ROS_WARN_STREAM_THROTTLE(5.0,"reading from robot state linax   : " << tmp_j_pos_feedback.linear_axis_value_);
    
    
    void* payload_ = malloc( sizeof(tmp_j_pos_feedback) );
    memcpy(payload_, &tmp_j_pos_feedback, sizeof(tmp_j_pos_feedback));  
    char topic_feedback[mqtt_feedback_topic_.length()+ 1];
    strcpy(topic_feedback, mqtt_feedback_topic_.c_str());
    mqtt_client_->publish(NULL, topic_feedback, sizeof(tmp_j_pos_feedback), payload_); 
    
    { 
      Json::Value root;
      Json::FastWriter writer;
      
      root["J0"] = tmp_j_pos_feedback.joints_values_[0];
      root["J1"] = tmp_j_pos_feedback.joints_values_[1];
      root["J2"] = tmp_j_pos_feedback.joints_values_[2];
      root["J3"] = tmp_j_pos_feedback.joints_values_[3];
      root["J4"] = tmp_j_pos_feedback.joints_values_[4];
      root["J5"] = tmp_j_pos_feedback.joints_values_[5];
      root["E0"] = tmp_j_pos_feedback.linear_axis_value_;
      
      Json::StreamWriterBuilder builder;
      const std::string json_file = Json::writeString(builder, root);
      
      char topic[] = "mqtt_feedback";
      
      char pl[json_file.length()+1];
      strcpy(pl, json_file.c_str());
      
      mqtt_client_->publish(NULL, topic, sizeof(pl), pl);
    }
    
  }

  void EgmToMQTTCtrl::stopping(const ros::Time& /*time*/)
  {}

  void EgmToMQTTCtrl::addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg)
  {

    // Preconditions
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam("extra_joints", list))
    {
      ROS_DEBUG("No extra joints specification found.");
      return;
    }

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Extra joints specification is not an array. Ignoring.");
      return;
    }

    for(std::size_t i = 0; i < list.size(); ++i)
    {
      XmlRpc::XmlRpcValue& elem = list[i];

      if (elem.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << elem.getType() <<
                         "'. Ignoring.");
        continue;
      }

      if (!elem.hasMember("name"))
      {
        ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
        continue;
      }

      const std::string name = elem["name"];
      if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
      {
        ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
        continue;
      }

      const bool has_pos = elem.hasMember("position");
      const bool has_vel = elem.hasMember("velocity");
      const bool has_eff = elem.hasMember("effort");

      const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
      if (has_pos && elem["position"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
        continue;
      }
      if (has_vel && elem["velocity"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
        continue;
      }
      if (has_eff && elem["effort"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
        continue;
      }

      // State of extra joint
      const double pos = has_pos ? static_cast<double>(elem["position"]) : 0.0;
      const double vel = has_vel ? static_cast<double>(elem["velocity"]) : 0.0;
      const double eff = has_eff ? static_cast<double>(elem["effort"])   : 0.0;

      // Add extra joints to message
      msg.name.push_back(name);
      msg.position.push_back(pos);
      msg.velocity.push_back(vel);
      msg.effort.push_back(eff);
    }
  }

}

PLUGINLIB_EXPORT_CLASS( drapebot_controller::EgmToMQTTCtrl, controller_interface::ControllerBase)

#pragma once

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <abb_egm_hardware_interface/egm_state_interface.h>
#include <abb_egm_msgs/EGMState.h>
#include <sensor_msgs/JointState.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>

namespace drapebot_controller
{
class EgmToMQTTCtrl: public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
  EgmToMQTTCtrl() : publish_rate_(0.0) {}

  virtual bool init(hardware_interface::JointStateInterface* hw,
                    ros::NodeHandle&                         root_nh,
                    ros::NodeHandle&                         controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::JointStateHandle> joint_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_hw_joints_; ///< Number of joints present in the JointStateInterface, excluding extra joints
  
  drapebot::MQTTClient* mqtt_client_;
  std::string mqtt_feedback_topic_;

  void addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg);
};

}

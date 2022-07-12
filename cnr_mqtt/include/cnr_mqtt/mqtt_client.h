#pragma once

#include <mosquittopp.h>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#define MAX_PAYLOAD 50
#define DEFAULT_KEEP_ALIVE 60

class mqtt_client : public mosqpp::mosquittopp
{
public:
  mqtt_client (const char *id, const char *host, int port);
  ~mqtt_client();

  void on_connect(int rc);
  void on_message(const struct mosquitto_message *message);
  void on_subscribe(int mid, int qos_count, const int *granted_qos);
  void set_joint_names(std::vector<std::string> jn);
  void set_config(const std::string& config);
  std::string get_config();

  bool new_trajectory_available;
  int n_joints_;  
  std::vector<std::string> joint_names_;
  control_msgs::FollowJointTrajectoryGoal trajectory_msg;
  
private:
  
  std::string configuration_;
  control_msgs::FollowJointTrajectoryGoal transform_trajectory(Json::Value traj);
  
  
};


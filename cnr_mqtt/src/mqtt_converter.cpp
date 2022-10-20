#include "ros/ros.h"
#include "cnr_mqtt/mqtt_client.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  
  ROS_INFO_STREAM("[mqtt converter] -> looking for params under NS: "<< nh.getNamespace());
  
  std::string action_name;
  if(!nh.getParam("action_name",action_name))
  {
    ROS_ERROR_STREAM("action name: not found on namespace: "<<nh.getNamespace()<<". default!: "<<action_name);
    return -1;
  }
  
  std::string MQTT_TOPIC_SUB;
  if(!nh.getParam("mqtt_topic",MQTT_TOPIC_SUB))
  {
    MQTT_TOPIC_SUB = "/robot_1/joint_trajectory";
    ROS_WARN_STREAM("mqtt topic  not found on namespace: "<<nh.getNamespace()<<". default!: "<<MQTT_TOPIC_SUB);
  }
  int n = MQTT_TOPIC_SUB.length();
  char topic[n+ 1];
  strcpy(topic, MQTT_TOPIC_SUB.c_str());
  
  std::string CLIENT_ID;
  if(!nh.getParam("client_id",CLIENT_ID))
  {
    CLIENT_ID = "mqtt_converter";
    ROS_WARN_STREAM("client id not found on namespace: "<<nh.getNamespace()<<". default!: "<<CLIENT_ID);
  }
  int n_id = CLIENT_ID.length();
  char client_id[n_id+ 1];
  strcpy(client_id, CLIENT_ID.c_str());
  
  std::string BROKER_ADDRESS;
  if(!nh.getParam("broker_address",BROKER_ADDRESS))
  {
    BROKER_ADDRESS = "localhost";
    ROS_WARN_STREAM("broker address  not found on namespace: "<<nh.getNamespace()<<". default!: "<<BROKER_ADDRESS);
  }
  int n_ba = BROKER_ADDRESS.length();
  char broker_address[n_ba+ 1];
  strcpy(broker_address, BROKER_ADDRESS.c_str());
  
  int port;
  if(!nh.getParam("port",port))
  {
    port = 1883;
    ROS_WARN_STREAM("port not found on namespace: "<<nh.getNamespace()<<". default!: 1883");
  }
  
  int rate;
  if(!nh.getParam("rate",rate))
  {
    rate = 10;
    ROS_WARN_STREAM("rate not found on namespace: "<<nh.getNamespace()<<". default!: 10");
  }
  
  std::vector<std::string> joint_names;
  if(!nh.getParam("joint_names",joint_names))
  {
    ROS_WARN_STREAM("rate not found on namespace: "<<nh.getNamespace()<<". default!: 10");
  }
  
  ROS_INFO_STREAM(joint_names[0]);
  ROS_INFO_STREAM(std::to_string(port));
  ROS_INFO_STREAM(topic);
  ROS_INFO_STREAM(action_name);
  ROS_INFO_STREAM(broker_address);
  ROS_INFO_STREAM(client_id);
  
  mosquitto_lib_init();

  mqtt_client client(client_id, broker_address, port);
  
  mosqpp::lib_init();
  
  ros::Rate r = rate;
  
  ROS_INFO_STREAM("[mqtt_converter] -> subscribing to "<<topic);
  client.subscribe(NULL, topic);  
  
  client.set_joint_names(joint_names);
  
  client.new_trajectory_available = false;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_trajectory ( action_name, true );
  ROS_INFO_STREAM("waitin for server "<<action_name);
  execute_trajectory.waitForServer();
  ROS_INFO_STREAM(action_name<<" connected ! ");
  
  while(ros::ok())
  {
    ROS_INFO_THROTTLE(2.0,"looping");
    client.loop();
    
    if(client.new_trajectory_available)
    {
      execute_trajectory.sendGoal ( client.trajectory_msg );
      ROS_WARN_STREAM("goal trajectory sent:\n"<<client.trajectory_msg);
      
      std::vector<double> first_point;
      for (auto p : client.trajectory_msg.trajectory.points[0].positions)
        ROS_INFO_STREAM(p);
      
      nh.setParam("initial_traj_pos", client.trajectory_msg.trajectory.points[0].positions );
      ROS_INFO_STREAM("waitin for execution");
      execute_trajectory.waitForResult();
      if ( !execute_trajectory.getResult() )
      {
        ROS_ERROR("some error in trajectory execution. Return!");
        return -1;
      }
      
      client.new_trajectory_available = false;
      
    }
    
    r.sleep();    
    
  }
  

  mosqpp::lib_cleanup();

  return 0;
}


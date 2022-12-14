#include "ros/ros.h"
#include "cnr_mqtt/mqtt_converter_client.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <configuration_msgs/StartConfiguration.h>



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
  
  
  bool handle_deformation;
  if(!nh.getParam("handle_deformation",handle_deformation))
  {
    ROS_WARN_STREAM("handle_deformation not found on namespace: "<<nh.getNamespace()<<". default: false");
    handle_deformation = false;
  }
  std::string staring_configuration;
  if(!nh.getParam("staring_configuration",staring_configuration))
  {
    staring_configuration = "watch";
    ROS_WARN_STREAM("staring_configuration  not found on namespace: "<<nh.getNamespace()<<". default!: "<<staring_configuration);
  }
  
  ROS_INFO_STREAM(joint_names[0]);
  ROS_INFO_STREAM(std::to_string(port));
  ROS_INFO_STREAM(topic);
  ROS_INFO_STREAM(action_name);
  ROS_INFO_STREAM(broker_address);
  ROS_INFO_STREAM(client_id);
  
  cnr::drapebot_converter::MQTTDrapebotClientHw client(client_id, broker_address, port);
  
  std::string robot_hw_ns;
  if(!nh.getParam("robot_hw_ns",robot_hw_ns))
  {
    robot_hw_ns = "robot1_mqtt_hw";
    ROS_WARN_STREAM("robot_hw_ns not found on namespace: "<<nh.getNamespace()<<". default!: "<<robot_hw_ns);
  }
  nh.setParam(robot_hw_ns+"/last_point_available",false);
  
  
  ros::Rate r = rate;
  
  ROS_INFO_STREAM("[mqtt_converter] -> subscribing to "<<topic);
  
  if (client.subscribe(NULL, topic, 1) != 0)
  {
    ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << topic );
    return -1;
  }
  
  client.set_config(staring_configuration);
  client.set_joint_names(joint_names);
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_trajectory ( action_name, true );
  ROS_INFO_STREAM("waitin for server "<<action_name);
  execute_trajectory.waitForServer();
  ROS_INFO_STREAM(action_name<<" connected ! ");
  
  ros::ServiceClient configuration_srv = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  if (handle_deformation)
  {
    ROS_INFO_STREAM("waitin for server /configuration_manager/start_configuration");
    configuration_srv.waitForExistence();
    ROS_INFO_STREAM("/configuration_manager/start_configuration connected ! ");
  }
  
  while(ros::ok())
  {
    ROS_INFO_THROTTLE(5.0,"looping");
    client.loop();




    if(client.isNewMessageAvailable())
    {
      control_msgs::FollowJointTrajectoryGoal trajectory_msg;
      client.getLastReceivedMessage(trajectory_msg);
      
      if (handle_deformation)
      {
        configuration_msgs::StartConfiguration start;
        start.request.start_configuration = client.get_config();
        start.request.strictness = 1;
        configuration_srv.call(start);
      }
            
      execute_trajectory.sendGoal ( trajectory_msg );
      ROS_INFO_STREAM(BLUE<<"goal trajectory sent:\n"<<trajectory_msg);
      
      std::vector<double> first_point;
      ROS_INFO_STREAM("first traj point");
      for (auto p : trajectory_msg.trajectory.points[0].positions)
        ROS_INFO_STREAM(p);
      
      ROS_INFO_STREAM("last traj point");      
      for (auto p : trajectory_msg.trajectory.points.back().positions)
        ROS_INFO_STREAM(p);
      nh.setParam(robot_hw_ns+"/last_traj_point",trajectory_msg.trajectory.points.back().positions);
      nh.setParam(robot_hw_ns+"/last_point_available",true);
      
      ROS_INFO_STREAM("waitin for execution");
      
      actionlib::SimpleClientGoalState as = execute_trajectory.getState();
      
      while(as != actionlib::SimpleClientGoalState::SUCCEEDED )
      {
        as = execute_trajectory.getState();
        
        if(as == actionlib::SimpleClientGoalState::ACTIVE) 
          ROS_INFO_STREAM_THROTTLE(2.0,GREEN<<"executing trajectory. State :  ACTIVE !" );
        else if(as == actionlib::SimpleClientGoalState::SUCCEEDED) 
          ROS_INFO_STREAM(RED<<"executing trajectory. State :  SUCCEEDED !" );
        else if(as == actionlib::SimpleClientGoalState::ABORTED) 
        {
          ROS_INFO_STREAM(YELLOW<<"executing trajectory. State :  ABORTED !" );
          ROS_ERROR("Trajectory not completely executed . Aborting");
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::LOST) 
        {
          ROS_INFO_STREAM(BLUE<<"executing trajectory. State :  LOST !" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::PENDING) 
          ROS_INFO_STREAM(MAGENTA<<"executing trajectory. State :  PENDING !" );
        else if(as == actionlib::SimpleClientGoalState::RECALLED) 
        {
          ROS_INFO_STREAM(CYAN<<"executing trajectory. State :  RECALLED !" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::REJECTED)
        {
          ROS_INFO_STREAM(WHITE<<"executing trajectory. State :  REJECTED!" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::PREEMPTED)
        {
          ROS_INFO_STREAM(CYAN<<"executing trajectory. State :  PREEMPTED!" );
          break;
        }
        
        client.loop();
        
        if(client.isNewMessageAvailable())
        {   
          control_msgs::FollowJointTrajectoryGoal trajectory_msg;
          client.getLastReceivedMessage(trajectory_msg);
          execute_trajectory.cancelGoal();
          execute_trajectory.cancelAllGoals();
          execute_trajectory.sendGoal ( trajectory_msg );
          ROS_INFO_STREAM(BOLDGREEN<<"New trajectory received. Goal changed ! ");
          break;
        }  
      }

      if ( !execute_trajectory.getResult() )
      {
        ROS_ERROR("some error in trajectory execution. Return!");
        return -1;
      }
      ROS_INFO_STREAM(GREEN << "Trajectory executed correctly ! ");
      
    }
    
    r.sleep();    
    
  }

  return 0;
}


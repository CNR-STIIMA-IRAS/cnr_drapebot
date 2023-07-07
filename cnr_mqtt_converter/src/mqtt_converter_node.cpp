#include "ros/ros.h"
#include <cnr_mqtt_converter/mqtt_converter_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <configuration_msgs/StartConfiguration.h>
#include <std_srvs/Trigger.h>
#include <rosdyn_core/primitives.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>



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
  if(!nh.getParam("robot_joint_names",joint_names))
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
  
  std::string robot_hw_ns;
  if(!nh.getParam("robot_hw_ns",robot_hw_ns))
  {
    robot_hw_ns = "robot1_mqtt_hw";
    ROS_WARN_STREAM("robot_hw_ns not found on namespace: "<<nh.getNamespace()<<". default!: "<<robot_hw_ns);
  }
  nh.setParam(robot_hw_ns+"/last_point_available",false);
  
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << std::to_string(port));
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << topic);
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << action_name);
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << broker_address);
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << client_id);
  
  cnr::drapebot_converter::MQTTClient client(client_id, broker_address, port);
  
  ros::Rate r = rate;
  
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << "[mqtt_converter] -> subscribing to "<<topic);
  
  if (client.subscribe(NULL, topic, 1) != 0)
  {
    ROS_ERROR_STREAM("[ " << robot_hw_ns << " ]" << "Error on Mosquitto subscribe topic: " << topic );
    return -1;
  }
  
  client.set_config(staring_configuration);
  client.set_joint_names(joint_names);
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_trajectory ( action_name, true );
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << "waitin for server "<<action_name);
  execute_trajectory.waitForServer();
  ROS_INFO_STREAM("[ " << robot_hw_ns << " ] - " << action_name<<" connected ! ");
  
  ros::ServiceClient configuration_srv = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient reset_pose_estimation = nh.serviceClient<std_srvs::Trigger>("/reset_pose_estimation");
    
  if (handle_deformation)
  {
    ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " waitin for server /configuration_manager/start_configuration");
    configuration_srv.waitForExistence();
    ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " /configuration_manager/start_configuration connected ! ");
    ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " waitin for server /reset_pose_estimation");
    reset_pose_estimation.waitForExistence();
    ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " /reset_pose_estimation connected ! ");

  }
  
  
  
//   TO publish goal pose
  std::string base_link, tool_link, target_pose;
  if(!nh.getParam("base_link",base_link))
    ROS_ERROR_STREAM("base link not found on namespace: "<<nh.getNamespace()<<". retunr!: ");
  if(!nh.getParam("tool_link",tool_link))
    ROS_WARN_STREAM("tool_link not found on namespace: " <<nh.getNamespace()<<". retunr!: ");
  if(!nh.getParam("target_pose",target_pose))
    ROS_WARN_STREAM("target_pose not found on namespace: "<<nh.getNamespace()<<". retunr!: ");
  
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_ERROR("Urdf robot_description '%s' does not exist",(nh.getNamespace()+"/robot_description").c_str());
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain_bt = rosdyn::createChain(urdf_model, base_link, tool_link, gravity);
  Eigen::Affine3d T_bt ;
  tf::Pose goal_tf;
  static tf::TransformBroadcaster br;
//   end

  while(ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(5.0,"[ " << robot_hw_ns << " ]" << " - looping");
    client.loop();

    if(client.isNewMessageAvailable())
    {
      control_msgs::FollowJointTrajectoryGoal trajectory_msg;
      bool cooperative_traj=false;
      client.getLastReceivedMessage(trajectory_msg,cooperative_traj);
//       try
//       {
//       cooperative_traj=client.isTrajCooperative();
//       }
//       catch(const std::exception& e)
//       {
//         ROS_ERROR_STREAM("[ "<<robot_hw_ns<<" ] Exception thrown in isTrajCooperative: " <<  e.what() );
//       }
      
      
      std_srvs::Trigger reset_pose;
      configuration_msgs::StartConfiguration start;
      if (cooperative_traj)
      {
        ros::Duration(0.1).sleep();
        
        Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(trajectory_msg.trajectory.points.front().positions.data(), trajectory_msg.trajectory.points.front().positions.size());
        T_bt = chain_bt->getTransformation(vec);
        tf::Pose human_tf;
        tf::poseEigenToTF (T_bt, human_tf);
        for(int i=0;i<10;i++)
        {
//           ROS_INFO_STREAM("publishing tf human");
//           br.sendTransform(tf::StampedTransform(human_tf, ros::Time::now(), base_link, "human_trg_pose"));
//           ros::Duration(0.05).sleep();
        }
        ROS_INFO_STREAM(BOLDYELLOW<<"Deformation active . ");
        start.request.start_configuration = "planner_def";
      }
      else
      {
        ROS_INFO_STREAM(BOLDWHITE<<"Executing nominal trajectory ");
        start.request.start_configuration = "planner";
      }
      start.request.strictness = 1;
      configuration_srv.call(start);
      
//       if (cooperative_traj)
//       {
//         ROS_INFO_STREAM(CYAN<<"resetting human estimation pose ");
//         reset_pose_estimation.call(reset_pose);
//       }
      
      
      ROS_INFO_STREAM("trajectory to be executed");
      ROS_INFO_STREAM(trajectory_msg );
      
      execute_trajectory.sendGoal ( trajectory_msg );
      ros::Duration(0.1).sleep();
      
      std::vector<double> first_point;
      ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " - first traj point");
      for (auto p : trajectory_msg.trajectory.points[0].positions)
        ROS_INFO_STREAM(p);
      
      ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " - last traj point");      
      for (auto p : trajectory_msg.trajectory.points.back().positions)
        ROS_INFO_STREAM(p);
      nh.setParam(robot_hw_ns+"/last_traj_point",trajectory_msg.trajectory.points.back().positions);
      nh.setParam(robot_hw_ns+"/last_point_available",true);
      
      Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(trajectory_msg.trajectory.points.back().positions.data(), trajectory_msg.trajectory.points.back().positions.size());
      T_bt = chain_bt->getTransformation(vec);
      tf::poseEigenToTF (T_bt, goal_tf);
      
      ROS_INFO_STREAM("[ " << robot_hw_ns << " ]" << " - waitin for execution");
      
      actionlib::SimpleClientGoalState as = execute_trajectory.getState();
      
      br.sendTransform(tf::StampedTransform(goal_tf, ros::Time::now(), base_link, target_pose));

      while(as != actionlib::SimpleClientGoalState::SUCCEEDED )
      {

        br.sendTransform(tf::StampedTransform(goal_tf, ros::Time::now(), base_link, target_pose));
        
        as = execute_trajectory.getState();
        
        if(as == actionlib::SimpleClientGoalState::ACTIVE) 
          ROS_INFO_STREAM_THROTTLE(1.0,YELLOW<<"executing trajectory. State :  ACTIVE !" );
        else if(as == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO_STREAM(GREEN<< "[ " << robot_hw_ns << " ]executing trajectory. State :  SUCCEEDED !" );
          ROS_INFO_STREAM(GREEN << "[ " << robot_hw_ns << " ]Trajectory executed correctly ! ");
        }
        else if(as == actionlib::SimpleClientGoalState::ABORTED) 
        {
          ROS_INFO_STREAM(YELLOW<< "[ " << robot_hw_ns << " ]executing trajectory. State :  ABORTED !" );
          ROS_ERROR("Trajectory not completely executed . Aborting");
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::LOST) 
        {
          ROS_INFO_STREAM(BLUE<< "[ " << robot_hw_ns << " ]executing trajectory. State :  LOST !" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::PENDING) 
          ROS_INFO_STREAM(MAGENTA<< "[ " << robot_hw_ns << " ]executing trajectory. State :  PENDING !" );
        else if(as == actionlib::SimpleClientGoalState::RECALLED) 
        {
          ROS_INFO_STREAM(CYAN<< "[ " << robot_hw_ns << " ]executing trajectory. State :  RECALLED !" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::REJECTED)
        {
          ROS_INFO_STREAM(WHITE<< "[ " << robot_hw_ns << " ]executing trajectory. State :  REJECTED!" );
          break;
        }
        else if(as == actionlib::SimpleClientGoalState::PREEMPTED)
        {
          ROS_INFO_STREAM(CYAN<< "[ " << robot_hw_ns << " ]executing trajectory. State :  PREEMPTED!" );
          break;
        }
        
        client.loop();
        
        if(client.isNewMessageAvailable())
        {
          execute_trajectory.cancelGoal();
          execute_trajectory.cancelAllGoals();
          ROS_INFO_STREAM(BOLDGREEN<< "[ " << robot_hw_ns << " ]New trajectory received. Goal changed ! ");
          break;
        }
      }

      if ( !execute_trajectory.getResult() )
      {
        ROS_ERROR_STREAM("[ " << robot_hw_ns << " ]some error in trajectory execution. Return!");
        return -1;
      }
      
    }
    
    r.sleep();    
    
  }

  return 0;
}


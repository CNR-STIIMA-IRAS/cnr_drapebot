#include "cnr_mqtt/mqtt_client.h"

mqtt_client::mqtt_client(const char *id, const char *host, int port) : mosquittopp(id)
{
  int keepalive = DEFAULT_KEEP_ALIVE;
  connect(host, port, keepalive);
}

mqtt_client::~mqtt_client()
{
}

void mqtt_client::on_connect(int rc)
{
  if (!rc)
  {
    ROS_INFO_STREAM( "Connected - code " << rc );
  }
}

void mqtt_client::set_joint_names(std::vector<std::string> jn)
{
  for(auto n : jn)
  {
    joint_names_.push_back(n);
    ROS_INFO_STREAM(n);
  }
  n_joints_ = joint_names_.size();
}


void mqtt_client::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  std::cout << "Subscription succeeded." << std::endl;
}

void mqtt_client::on_message(const struct mosquitto_message *message)
{
  char buf[message->payloadlen];
  memcpy(buf, message->payload, message->payloadlen);
  
  Json::Reader reader;
  Json::Value root;
  
  reader.parse(buf,root);

//   std::cout << buf << std::endl;
  
  ROS_INFO_STREAM("Json Trajectory from MotionPlanner: \n" << buf);
  
  trajectory_msg = transform_trajectory(root);
  
}


control_msgs::FollowJointTrajectoryGoal mqtt_client::transform_trajectory(Json::Value traj)
{
  control_msgs::FollowJointTrajectoryGoal ag;
  
//   ROS_INFO_STREAM("joint number: "<<std::to_string(n_joints_));
  
  ag.trajectory.points.resize(traj.size());
  
  ag.trajectory.header.frame_id = joint_names_[0];
  
  for(auto n : joint_names_)
  {
    ag.trajectory.joint_names.push_back(n);
    ROS_INFO_STREAM(ag.trajectory.joint_names.back());
  }
    
  for (int i=0; i<traj.size();i++)
  {
    
    std::string P = "P"+  std::to_string((i+1));
    
    ros::Duration time_from_start( std::stod( traj[P]["time"].asString() ));
    
    ag.trajectory.points[i].time_from_start = time_from_start;
    
    ag.trajectory.points[i].positions.resize(n_joints_);
    ag.trajectory.points[i].velocities.resize(n_joints_);
    ag.trajectory.points[i].accelerations.resize(n_joints_);
    
    for(int jj=0;jj<n_joints_;jj++)
    {
      std::string jn = "J";
      jn+= std::to_string(jj);
      ROS_INFO_STREAM(jn);

      double j = std::stod( traj[P][jn]["value"].asString() );
      
      ag.trajectory.points[i].positions[jj] = j;
      
      ag.trajectory.points[i].velocities[jj]=0;
      ag.trajectory.points[i].accelerations[jj]=0;
    }
    
//     ag.trajectory.points[i].positions[0] = traj[P]["J0"]["value"].asDouble();
//     ag.trajectory.points[i].positions[1] = traj[P]["J1"]["value"].asDouble();
//     ag.trajectory.points[i].positions[2] = traj[P]["J2"]["value"].asDouble();
//     ag.trajectory.points[i].positions[3] = traj[P]["J3"]["value"].asDouble();
//     ag.trajectory.points[i].positions[4] = traj[P]["J4"]["value"].asDouble();
//     ag.trajectory.points[i].positions[5] = traj[P]["J5"]["value"].asDouble();
//     ag.trajectory.points[i].positions[6] = traj[P]["J6"]["value"].asDouble();
//     
//     for(int kk =0; kk<ag.trajectory.points[i].positions.size();kk++)
//     {
//       ag.trajectory.points[i].velocities[i]=0;
//       ag.trajectory.points[i].accelerations[i]=0;
//     }
    
  }
  
  ROS_INFO_STREAM("goal trajectory: \n"<<ag);
  
  new_trajectory_available = true;
  
  return ag;
}

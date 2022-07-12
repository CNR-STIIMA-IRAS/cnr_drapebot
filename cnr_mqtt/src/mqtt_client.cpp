#include "cnr_mqtt/mqtt_client.h"

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

void mqtt_client::set_config(const std::string& config)
{
  configuration_ = config;
}

std::string mqtt_client::get_config()
{
  return configuration_;
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
  
  ROS_INFO_STREAM(GREEN<<"Json Trajectory from MotionPlanner: \n" << buf);
  
  trajectory_msg = transform_trajectory(root);
  
}


control_msgs::FollowJointTrajectoryGoal mqtt_client::transform_trajectory(Json::Value traj)
{
  control_msgs::FollowJointTrajectoryGoal ag;
  
  ag.trajectory.points.resize(traj.size());
  
  ag.trajectory.header.frame_id = joint_names_[0];
  
  for(auto n : joint_names_)
  {
    ag.trajectory.joint_names.push_back(n);
    ROS_DEBUG_STREAM(ag.trajectory.joint_names.back());
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
      ROS_DEBUG_STREAM(jn);
      
      double jp = std::stod( traj[P][jn]["value"].asString() );
      double jv = std::stod( traj[P][jn]["velocity"].asString() );
      
      ag.trajectory.points[i].positions[jj] = jp;
      ag.trajectory.points[i].velocities[jj]= jv;
      ag.trajectory.points[i].accelerations[jj]=0;
    }
  }
  
  ROS_DEBUG_STREAM("goal trajectory: \n"<<ag);
  
  new_trajectory_available = true;
  
  return ag;
}

/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>


#include <cnr_controller_interface_params/cnr_controller_interface_params.h>
// #include <cnr_hardware_interface/internal/vector_to_string.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>
#include <cnr_mqtt_hardware_interface/cnr_mqtt_robot_hw.h>

#include <jsoncpp/json/json.h>

PLUGINLIB_EXPORT_CLASS(cnr_hardware_interface::MQTTRobotHW, hardware_interface::RobotHW)

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


mqtt_client::mqtt_client(const char *id, const char *host, int port, int keepalive) : mosquittopp(id)
{ 
  first_message_received = false;
  J1 = 0;
  J2 = 0;
  J3 = 0;
  J4 = 0;
  J5 = 0;
  J6 = 0;
  E0 = 0;
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

void mqtt_client::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
//         std::cout << "Subscription succeeded." << std::endl;
}

void mqtt_client::on_message(const struct mosquitto_message *message)
{
  
  message_struct* buf = new message_struct;
  memcpy(buf, message->payload, message->payloadlen);
  
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J1: "<< buf->J1);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J2: "<< buf->J2);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J3: "<< buf->J3);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J4: "<< buf->J4);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J5: "<< buf->J5);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback J6: "<< buf->J6);
  ROS_INFO_STREAM_THROTTLE(5.0,"feedback E0: "<< buf->E0);
  ROS_INFO_STREAM_THROTTLE(5.0,"count      : "<< buf->count);
  
  J1 = buf->J1;
  J2 = buf->J2;
  J3 = buf->J3;
  J4 = buf->J4;
  J5 = buf->J5;
  J6 = buf->J6;
  E0 = buf->E0;
  
  msg_count_fb = buf->count;
  
  first_message_received = true;
}

bool mqtt_client::get_first_message_status()
{
  return first_message_received ;
}

void mqtt_client::publish_with_tracking(const std::string cmd_topic, message_struct& m)
{
  msg_count_cmd +=1;
  m.count = msg_count_cmd;
  
  size_t message_size_ = sizeof(m);
  void* payload_ = malloc( message_size_ );
  memcpy(payload_, &m, message_size_);  
  
  int n = cmd_topic.length();
  char topic[n+ 1];
  strcpy(topic, cmd_topic.c_str());
  
  publish(NULL, topic, message_size_, payload_);
  
}


int mqtt_client::get_msg_count_cmd()
{
  return msg_count_cmd;
}

int mqtt_client::get_msg_count_fb()
{
  return msg_count_fb;
}





void mqtt_to_vector(const mqtt_client * client,std::vector<double> & ret)
{
  if(ret.size()!=7)
    ret.resize(7,0);
  
  ret.at(1) = client->J1;
  ret.at(2) = client->J2;
  ret.at(3) = client->J3;
  ret.at(4) = client->J4;
  ret.at(5) = client->J5;
  ret.at(6) = client->J6;
  ret.at(0) = client->E0;
}


void vector_to_mqtt(const std::vector<double>& v, mqtt_client * client)
{
  assert(v.size()==7);

  client->J1 = v.at(1);
  client->J2 = v.at(2);
  client->J3 = v.at(3);
  client->J4 = v.at(4);
  client->J5 = v.at(5);
  client->J6 = v.at(6);
  client->E0 = v.at(0);
}

void vector_to_mqtt_msg(const std::vector<double>& v, message_struct& m)
{
  assert(v.size()==7);

  m.J1 = v.at(1);    
  m.J2 = v.at(2);    
  m.J3 = v.at(3);    
  m.J4 = v.at(4);    
  m.J5 = v.at(5);    
  m.J6 = v.at(6);    
  m.E0 = v.at(0);}

void print_vector(const std::string& msg, const std::vector<double>& v, const char* color = "\033[0m" )
{
  assert(v.size()==7);
  for(size_t i=0;i< v.size();i++)
  {
    ROS_INFO_STREAM( color << msg << " " << (i==0u? "E" : "J") << i << ": "  << v.at(i));
  }
}
void print_vector_throttle(const std::string& msg, const std::vector<double>& v ,double throttle = 1.0, const char* color = "\033[0m")
{
  assert(v.size()==7);
  for(size_t i=0;i< v.size();i++)
  {
    ROS_INFO_STREAM_THROTTLE(throttle, color << msg << " " << (i==0u? "E" : "J") << i << ": "  << v.at(i));
  }
}

void print_message_struct_throttle(cnr_logger::TraceLogger logger, const std::string& msg, const message_struct& m, double throttle = 1.0)
{
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J1);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J2);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J3);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J4);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J5);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.J6);
  CNR_INFO_THROTTLE(logger,throttle , msg << m.E0);
}


namespace cnr_hardware_interface
{

void setParam(MQTTRobotHW* hw, const std::string& ns)
{
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/name", hw->resourceNames());
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/position", hw->m_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/velocity", hw->m_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/feedback/effort", hw->m_eff);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/name", hw->resourceNames());
  hw->m_robothw_nh.setParam("status/" + ns + "/command/position", hw->m_cmd_pos);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/velocity", hw->m_cmd_vel);
  hw->m_robothw_nh.setParam("status/" + ns + "/command/effort", hw->m_cmd_eff);
}

MQTTRobotHW::MQTTRobotHW()
  : m_msg(nullptr)
{
  m_set_status_param = boost::bind(setParam, this, _1);
}

MQTTRobotHW::~MQTTRobotHW()
{
  if (!m_shutted_down)
  {
    shutdown();
  }
}

void MQTTRobotHW::initialJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  m_msg  = new sensor_msgs::JointState();
  *m_msg = *msg;
}

void MQTTRobotHW::wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  if (msg->header.frame_id.compare(m_frame_id))
  {
    CNR_WARN_THROTTLE(m_logger,1,"wrench has wrong frame_id. expected = "<<m_frame_id<<", received = "<<msg->header.frame_id);
    return;
  }
  m_ft_sensor.at(0)=msg->wrench.force.x;
  m_ft_sensor.at(1)=msg->wrench.force.y;
  m_ft_sensor.at(2)=msg->wrench.force.z;
  m_ft_sensor.at(3)=msg->wrench.torque.x;
  m_ft_sensor.at(4)=msg->wrench.torque.y;
  m_ft_sensor.at(5)=msg->wrench.torque.z;

  CNR_TRACE_THROTTLE(m_logger,10,"received a wrench");
}


void MQTTRobotHW::trajCb(const sensor_msgs::JointState::ConstPtr &msg)
{  
  m_nom_traj.at(0) = msg->position[0];
  m_nom_traj.at(1) = msg->position[1];
  m_nom_traj.at(2) = msg->position[2];
  m_nom_traj.at(3) = msg->position[3];
  m_nom_traj.at(4) = msg->position[4];
  m_nom_traj.at(5) = msg->position[5];
  m_nom_traj.at(6) = msg->position[6];

  CNR_TRACE_THROTTLE(m_logger,10,"received a wrench");
}


bool MQTTRobotHW::doInit()
{
  CNR_TRACE_START(m_logger);

  CNR_WARN(m_logger, "Resources (" << resourceNumber() << ") ");
  m_pos.resize(resourceNumber());
  m_vel.resize(resourceNumber());
  m_eff.resize(resourceNumber());
  
  m_cmd_pos.resize(resourceNumber());
  m_old_pos.resize(resourceNumber());
  m_start_pos.resize(resourceNumber());
  m_delta_pos.resize(resourceNumber());
  m_cmd_vel.resize(resourceNumber());
  m_cmd_eff.resize(resourceNumber());

  std::fill(m_pos.begin(), m_pos.end(), 0.0);
  std::fill(m_cmd_pos.begin(), m_cmd_pos.end(), 0.0);
  std::fill(m_old_pos.begin(), m_old_pos.end(), 0.0);
  std::fill(m_start_pos.begin(), m_start_pos.end(), 0.0);
  std::fill(m_delta_pos.begin(), m_delta_pos.end(), 0.0);
  std::fill(m_vel.begin(), m_vel.end(), 0.0);
  std::fill(m_eff.begin(), m_eff.end(), 0.0);

  m_ft_sensor.resize(6);
  std::fill(m_ft_sensor.begin(), m_ft_sensor.end(), 0.0);

  if (m_robothw_nh.hasParam("initial_position"))
  {
    m_robothw_nh.getParam("initial_position", m_pos);
    std::string ss;
    for (auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(m_logger, "Initial Position: <" << ss << ">");
  }
  else if (m_robothw_nh.hasParam("initial_position_from"))
  {
    std::string position_from;
    m_robothw_nh.getParam("position_from", position_from);

    CNR_DEBUG(m_logger, "Position From: '" << position_from << "'");
    std::string position_ns = "/" + position_from + "/status/shutdown_configuration/position";
    if (!m_robothw_nh.hasParam(position_ns))
    {
      CNR_ERROR(m_logger, "The param '" + position_ns + "' does not exit. pos superimposed to zero");
    }
    m_robothw_nh.getParam(position_ns, m_pos);
    std::string ss;
    for (auto const & p : m_pos) ss += std::to_string(p) + ", ";
    CNR_DEBUG(m_logger, "Initial Position: <" << ss << ">");
  }

  double timeout = 10;
  if (!m_robothw_nh.getParam("feedback_joint_state_timeout", timeout))
  {
    CNR_WARN(m_logger, "The param '" << m_robothw_nh.getNamespace() << "/feedback_joint_state_timeout' not defined, set equal to 10");
    timeout = 10;
  }

  m_cmd_pos = m_pos;
  m_cmd_vel = m_vel;
  m_cmd_pos = m_eff;
  m_delta_pos = m_pos;
  
  for(size_t i=0;i<resourceNumber();i++)
  {
    std::string joint_name = resourceNames().at(i);
    //auto i = &joint_name - &m_resource_names[0];

    hardware_interface::JointStateHandle state_handle(joint_name, &(m_pos.at(i)), &(m_vel.at(i)), &(m_eff.at(i)));

    m_js_jh.registerHandle(state_handle);

    m_p_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))));
    m_v_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))));
    m_e_jh.registerHandle(hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))));

    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle, &(m_cmd_pos.at(i)), &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle, &(m_cmd_vel.at(i)), &(m_cmd_eff.at(i))));
  }
  
  std::string wrench_name="wrench";
  if (!m_robothw_nh.getParam("wrench_resourse",wrench_name))
  {
    wrench_name="wrench";
    CNR_TRACE(m_logger,"using defalut wrench_resourse name: wrench");
  }

  if (!m_robothw_nh.getParam("frame_id",m_frame_id))
  {
    m_frame_id="tool0";
    CNR_TRACE(m_logger,"using defalut frame_id name: tool0");
  }

  std::string wrench_topic="fake_wrench";
  if (!m_robothw_nh.getParam("wrench_topic",wrench_topic))
  {
    wrench_topic="fake_wrench";
    CNR_TRACE(m_logger,"using defalut wrench_topic name: fake_wrench");
  }
  m_wrench_sub=m_robothw_nh.subscribe(wrench_topic,1,&MQTTRobotHW::wrenchCb,this);


  CNR_TRACE(m_logger,"Sensor handle");
  hardware_interface::ForceTorqueSensorHandle sensor_handle(wrench_name
      , m_frame_id
      , &(m_ft_sensor.at(0))
      , &(m_ft_sensor.at(3)));

  CNR_TRACE(m_logger,"Register handle");
  
  m_ft_jh.registerHandle(sensor_handle);

  CNR_TRACE(m_logger,"Register Interface");
  registerInterface(&m_js_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_e_jh);
  registerInterface(&m_pve_jh);
  registerInterface(&m_ve_jh);
  registerInterface(&m_ft_jh);

  m_p_jh_active = m_v_jh_active = m_e_jh_active = false;
  
  CNR_TRACE(m_logger,"Pose Target");
  
  std::string pose_target;
  if (!m_robothw_nh.getParam("nominal_trajectory_topic",pose_target))
  {
    pose_target="/joint_pos_target";
    CNR_WARN(m_logger,"default noimnal trajectory topic : joint_pos_target");
  }
  m_traj_sub=m_robothw_nh.subscribe(pose_target,1,&MQTTRobotHW::trajCb,this);
  
  m_nom_traj.resize(7,0);
  m_nom_traj.at(0) = m_pos[1];
  m_nom_traj.at(1) = m_pos[2];
  m_nom_traj.at(2) = m_pos[3];
  m_nom_traj.at(3) = m_pos[4];
  m_nom_traj.at(4) = m_pos[5];
  m_nom_traj.at(5) = m_pos[6];
  m_nom_traj.at(6) = m_pos[0];
  
  if (!m_robothw_nh.getParam("use_delta_target_pos",use_delta_target_pos_))
  {
    use_delta_target_pos_=false;
    CNR_TRACE(m_logger,"default delta_target_pose: false");
  }
  if(use_delta_target_pos_)
  {
    if (!m_robothw_nh.getParam("delta_pos_from_start",delta_pos_from_start_))
    {
      delta_pos_from_start_=true;
      CNR_TRACE(m_logger,"default delta_pos_from_start: true");
    }
  }
  
//   ---- MQTT params ----
  CNR_TRACE(m_logger," MQTT PARAMS");
  std::string cid;
  if (!m_robothw_nh.getParam("client_id",cid))
  {
    cid="Client_ID";
    CNR_TRACE(m_logger,"using defalut client ID: Client_ID");
  }
  
  CNR_TRACE(m_logger," MQTT PARAMS 2");

  int n_id = cid.length();
  char client_id[n_id + 1];
  strcpy(client_id, cid.c_str());
  
  std::string host_str;
  if (!m_robothw_nh.getParam("host",host_str))
  {
    host_str="localhost";
    CNR_TRACE(m_logger,"using defalut host: localhost");
  }
  
  CNR_TRACE(m_logger," MQTT PARAMS 3");

  int n_host = host_str.length();
  char host[n_host + 1];
  strcpy(host, host_str.c_str());
  
  int port;
  if (!m_robothw_nh.getParam("port",port))
  {
    port=1883;
    CNR_TRACE(m_logger,"using defalut port: 1883");
  }
  
  CNR_TRACE(m_logger," MQTT PARAMS 4");

  
  if (!m_robothw_nh.getParam("command_mqtt_topic",m_mqtt_command_topic))
  {
    m_mqtt_command_topic="mqtt_command_topic";
    CNR_TRACE(m_logger,"using topic out : mqtt_command_topic");
  }
  
  CNR_TRACE(m_logger," MQTT PARAMS mosquitto_lib_init");

  mosquitto_lib_init();
  
  CNR_WARN(m_logger,"connencting mqtt: "<<client_id<<":"<<host);
  m_client = new mqtt_client(client_id, host, port);
  CNR_INFO(m_logger,"connencted to: "<<client_id<<":"<<host);
  
  CNR_TRACE(m_logger," MQTT PARAMS lib_init");
  mosqpp::lib_init();
  
  if (!m_robothw_nh.getParam("feedback_mqtt_topic",m_mqtt_feedback_topic))
  {
    m_mqtt_feedback_topic="mqtt_feedback_topic";
    CNR_TRACE(m_logger,"using topic out : mqtt_feedback_topic");
  }
  
  if (!m_robothw_nh.getParam("out_feedback_mqtt_topic",m_mqtt_out_feedback_topic))
  {
    m_mqtt_out_feedback_topic="out_mqtt_feedback_topic";
    CNR_TRACE(m_logger,"using topic out : out_mqtt_feedback_topic");
  }
   
  if (!m_robothw_nh.getParam("use_real_robot",USE_REAL_ROBOT))
  {
    USE_REAL_ROBOT = false;
    CNR_WARN(this->m_logger, "\n\n USING SIMULATED ROBOT !!. SET PARAM >> use_real_robot: true << in the config to use the real robot");
    CNR_TRACE(m_logger,"\n\n USING SIMULATED ROBOT !!. SET PARAM >> use_real_robot: true << in the config to use the real robot");
  }
  
  if (!m_robothw_nh.getParam("verbose",verbose_))
  {
    verbose_ = true;
    CNR_TRACE(m_logger,"default verbose: true");
  }
  
  std::string v = USE_REAL_ROBOT ? "ACHTUNG ! ! ! \n USING REAL ROBOT ! \n be careful, be nice please" : "using fake robot." ;
  if (USE_REAL_ROBOT)
    CNR_INFO(m_logger,RED<<"\n\n ################# \n "<< v <<" \n################# \n\n");
  else
    CNR_INFO(m_logger,BLUE<<"\n"<<v<<"\n");
  
  int n = m_mqtt_feedback_topic.length();
  char topic[n+ 1];
  strcpy(topic, m_mqtt_feedback_topic.c_str());
  m_client->subscribe(NULL, topic);
  CNR_INFO(m_logger,"[cnr_mqtt_hardware_interface - "<< m_robothw_nh.getNamespace() << "] Subscribed to : "<<topic);  
  
  if(USE_REAL_ROBOT)
  {
    ROS_INFO_STREAM("start waiting");
    while ( !m_client->get_first_message_status() )
    {
      CNR_WARN_THROTTLE(m_logger,2.0,"waiting for first feedback message");
      m_client->loop();
      ros::Duration(0.005).sleep();
    }
  }
  else
  {
    vector_to_mqtt(m_pos,m_client);
  }
  
  mqtt_to_vector(m_client,m_cmd_pos);
  mqtt_to_vector(m_client,m_pos);
  m_start_pos = m_pos;
  
  if(verbose_)
  {
    print_vector("INITIAL POSITION" , m_cmd_pos  , BLUE   );
    print_vector("STARTING POSITION", m_start_pos, MAGENTA);
  }
  
  m_start_pos = m_pos;
  m_old_pos = m_pos;
  
  cmd_pos_pub_ = m_robothw_nh.advertise<sensor_msgs::JointState>("cmd_joint_pos",5);
  fb_pos_pub_  = m_robothw_nh.advertise<sensor_msgs::JointState>("fb_joint_pos",5);

  
  CNR_RETURN_TRUE(m_logger);
}


bool MQTTRobotHW::doWrite(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);

  // ----------- BYTE MQTT MSG -------------------
  {
    
    for(int jj=0;jj<m_cmd_pos.size();jj++)
    {
      if (delta_pos_from_start_)
        m_delta_pos[jj]=m_cmd_pos[jj]-m_start_pos[jj];
      else
        m_delta_pos[jj]=m_cmd_pos[jj]-m_old_pos[jj];
      m_old_pos[jj]=m_cmd_pos[jj];
    }
    
    message_struct m;
    
    std::string st = MAGENTA;
    
    if(use_delta_target_pos_)
    {
      vector_to_mqtt_msg(m_delta_pos,m);
      
      CNR_INFO_THROTTLE(m_logger,10.0, CYAN    << "using relative cmd position");
      st += "[MQTTRobotHW - " + m_robothw_nh.getNamespace() + "] : delta command : ";
    }
    else
    {
      vector_to_mqtt_msg(m_cmd_pos,m);
      
      CNR_INFO_THROTTLE(m_logger,10.0, CYAN    << "using absolute cmd position");
      st += "[MQTTRobotHW - " + m_robothw_nh.getNamespace() + "] : command : "; 
    }
    
    
    if(verbose_)
    {
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J1);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J2);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J3);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J4);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J5);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.J6);
      CNR_INFO_THROTTLE(m_logger, 2.0, st << m.E0);
    }
    
    m_client->publish_with_tracking(m_mqtt_command_topic,m);
    
    CNR_INFO_THROTTLE(m_logger,2.0,BLUE<<" msg_count : "<< m_client->get_msg_count_cmd());
    
    int n = m_mqtt_command_topic.length();
    char topic[n+ 1];
    strcpy(topic, m_mqtt_command_topic.c_str());
    ROS_INFO_STREAM_THROTTLE(10.0,"[MQTTRobotHW - " + m_robothw_nh.getNamespace() + "] publishing command on : "<<topic);
    
    sensor_msgs::JointState js;
    
    js.name.push_back("J1");
    js.name.push_back("J2");
    js.name.push_back("J3");
    js.name.push_back("J4");
    js.name.push_back("J5");
    js.name.push_back("J6");
    js.name.push_back("E0");

    js.position.push_back(m.J1);
    js.position.push_back(m.J2);
    js.position.push_back(m.J3);
    js.position.push_back(m.J4);
    js.position.push_back(m.J5);
    js.position.push_back(m.J6);
    js.position.push_back(m.E0);
    
    js.header.stamp = ros::Time::now();
    
    cmd_pos_pub_.publish(js);
    
  }
    
  // ----------- JSON MQTT MSG -------------------
  {
    Json::Value root;
    Json::FastWriter writer;
    
    root["J0"] = m_cmd_pos.at(1);
    root["J1"] = m_cmd_pos.at(2);
    root["J2"] = m_cmd_pos.at(3);
    root["J3"] = m_cmd_pos.at(4);
    root["J4"] = m_cmd_pos.at(5);
    root["J5"] = m_cmd_pos.at(6);
    root["E0"] = m_cmd_pos.at(0);
    
    Json::StreamWriterBuilder builder;
    const std::string json_file = Json::writeString(builder, root);
    
    char topic[] = "mqtt_command";
    char pl[json_file.length()+1];
    strcpy(pl, json_file.c_str());
    
    m_client->publish(NULL, topic, sizeof(pl), pl);
  }
  
  
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_logger);
}

bool MQTTRobotHW::doPrepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                  const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(m_logger);
  bool p_jh_active, v_jh_active, e_jh_active;

  p_jh_active = m_p_jh_active;
  v_jh_active = m_v_jh_active;
  e_jh_active = m_e_jh_active;

  for (const hardware_interface::ControllerInfo& controller : stop_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active = false;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active = false;
        e_jh_active = false;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active = false;
        v_jh_active = false;
        e_jh_active = false;
      }
    }
  }

  std::vector<std::string> resources;
  for (const hardware_interface::ControllerInfo& controller : start_list)
  {
    for (const hardware_interface::InterfaceResources& res : controller.claimed_resources)
    {
      resources.push_back(res.hardware_interface);
      CNR_DEBUG(m_logger, "Claimed resource: " << res.hardware_interface);
      p_jh_active = (res.hardware_interface == "hardware_interface::PositionJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : p_jh_active;

      v_jh_active = (res.hardware_interface == "hardware_interface::VelocityJointInterface")
                    || (res.hardware_interface == "hardware_interface::VelEffJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : v_jh_active;

      e_jh_active = (res.hardware_interface == "hardware_interface::EffortJointInterface")
                    || (res.hardware_interface == "hardware_interface::VelEffJointInterface")
                    || (res.hardware_interface == "hardware_interface::PosVelEffJointInterface")
                    ? true : e_jh_active;
    }
  }
  m_p_jh_active = p_jh_active;
  m_v_jh_active = v_jh_active;
  m_e_jh_active = e_jh_active;
  CNR_DEBUG(m_logger, " Pos joint handle active? " << m_p_jh_active);
  CNR_DEBUG(m_logger, " Vel joint handle active? " << m_v_jh_active);
  CNR_DEBUG(m_logger, " Eff joint handle active? " << m_e_jh_active);
  CNR_RETURN_TRUE(m_logger, "Active hardware interfaces: " + cnr::control::to_string(resources));

}

bool MQTTRobotHW::doRead(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  m_client->loop();  
  
  Json::Value root;
  Json::FastWriter writer;
  
  if (USE_REAL_ROBOT)
  {
    mqtt_to_vector(m_client,m_pos);
  }
  else
  {
    m_pos = m_cmd_pos;
  }
  
  
  if(verbose_)
  {
    print_vector_throttle("[MQTTRobotHW - "+ m_robothw_nh.getNamespace() + "] : robot state joint ", m_pos,2.0);
  }
  
  for(size_t i=0;i<m_pos.size();i++)
  {
    root["J" +std::to_string(i)]["current_value"] = m_pos.at(i);
    root["J" +std::to_string(i)]["command_value"] = m_cmd_pos.at(i);  
    root["J" +std::to_string(i)]["reference_value"] = m_nom_traj.at(i);
  }
  
  Json::StreamWriterBuilder builder;
  const std::string json_file = Json::writeString(builder, root);
    
  int n = m_mqtt_out_feedback_topic.length();
  char topic[n+ 1];
  strcpy(topic, m_mqtt_out_feedback_topic.c_str());
  
  int len = json_file.length();
  char pl[len+1];
  strcpy(pl, json_file.c_str());
  m_client->publish(NULL, topic, sizeof(pl), pl);
  CNR_INFO_THROTTLE(m_logger,2.0,GREEN<<"[MQTTRobotHW - "+ m_robothw_nh.getNamespace() + "] publishing in loop on : "<<topic);
  
  sensor_msgs::JointState js;
  js.name.clear();
  js.name.push_back("J1");
  js.name.push_back("J2");
  js.name.push_back("J3");
  js.name.push_back("J4");
  js.name.push_back("J5");
  js.name.push_back("J6");
  js.name.push_back("E0");

  js.position = m_pos;
  
  js.header.stamp = ros::Time::now();
  
  fb_pos_pub_.publish(js);
  
  CNR_RETURN_TRUE(m_logger);
}

bool MQTTRobotHW::doCheckForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  std::stringstream report;
  CNR_TRACE_START(m_logger);
  // Each controller can use more than one hardware_interface for a single joint (e.g.: position, velocity, effort).
  // One controller can control more than one joint.
  // A joint can be used only by a controller.

  std::vector<bool> global_joint_used(resourceNumber());
  std::fill(global_joint_used.begin(), global_joint_used.end(), false);

  for (hardware_interface::ControllerInfo controller : info)
  {
    std::vector<bool> single_controller_joint_used(resourceNumber());
    std::fill(single_controller_joint_used.begin(), single_controller_joint_used.end(), false);

    for (hardware_interface::InterfaceResources res : controller.claimed_resources)
    {
      for (std::string name : res.resources)
      {
        for (unsigned int iJ = 0; iJ < resourceNumber(); iJ++)
        {
          if (!name.compare(resourceNames().at(iJ)))
          {
            if (global_joint_used.at(iJ)) // if already used by another
            {
              addDiagnosticsMessage("ERROR", "Joint " + name + " is already used by another controller", {{"Transition", "switching"}}, &report);
              CNR_ERROR(m_logger, report.str());
              CNR_RETURN_TRUE(m_logger, "Joint " + name + " is already used by another controller");
            }
            else
            {
              single_controller_joint_used.at(iJ) = true;
            }
          }
        }
      }
    }
    for (unsigned int iJ = 0; iJ < resourceNumber(); iJ++)
    {
      global_joint_used.at(iJ) = global_joint_used.at(iJ) || single_controller_joint_used.at(iJ);
    }

  }
  CNR_RETURN_FALSE(m_logger);
}



}

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <cnr_mqtt_hardware_interface/json.hpp>
#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>

double arb_alpha = 0.0;

void alphaCallback(const std_msgs::Float32& msg)
{
  arb_alpha = msg.data; 
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"cnr_msgs_visualization");
  ros::NodeHandle nh;
  ros::Rate r(250); // 250 hz
  
  std::string alpha_topic;
  if (!nh.getParam("/mqtt_hw/gt_traj_arb/alpha_topic",alpha_topic))
  {
    alpha_topic = "/alpha";
    ROS_WARN_STREAM("The topic /mqtt_hw/gt_traj_arb/alpha_topic . Using default topic name: " + alpha_topic);
  }

  ros::Subscriber alpha_sub = nh.subscribe(alpha_topic, 1, alphaCallback);
  

  //---- MQTT params ----
  std::string client_id = "mqtt_ros_msgs_repeater_for_visualization";

  std::string host_str;
  if (!nh.getParam("/mqtt_hw/host",host_str))
  {
    host_str = "localhost";
    ROS_WARN_STREAM("broker_address not found under " + nh.getNamespace() + "mqtt_to_joint_position_controller/broker_address . Using defalut broker address: "+ host_str);
  }

  host_str = "localhost";

  int port;
  if (!nh.getParam("/mqtt_hw/port",port))
  {
    port = 1883;
    ROS_WARN_STREAM("port not found under " + nh.getNamespace() + "mqtt_to_joint_position_controller/port. Using defalut port: "+ std::to_string( port));      
  }    

  bool use_json;
  if (!nh.getParam("/mqtt_hw/use_json",use_json))
  {
    use_json = true;
    ROS_WARN_STREAM("use json flag not found " + nh.getNamespace() + "mqtt_to_joint_position_controller/use_json. Using defalut json flag: true " );      
  }   

  ROS_INFO_STREAM("Connencting mqtt: " << client_id << ", host: " << host_str << ", port: " << port);

  cnr::drapebot::MQTTDrapebotClientHw mqtt_drapebot_client(client_id.c_str(), host_str.c_str(), port, use_json);
  
  ROS_INFO_STREAM("Connencted to MQTT client name: "<< client_id << " address: " << host_str);
  
  std::string mqtt_alpha_topic;
  if (!nh.getParam("/msgs_visualization/mqtt_alpha_topic", mqtt_alpha_topic))
  {
    mqtt_alpha_topic = "/mqtt_alpha_topic";
    ROS_WARN_STREAM("mqtt_alpha_topic not found under /msgs_visualization/mqtt_alpha_topic . Using defalut mqtt command topic name: "+ mqtt_alpha_topic);  
  }

  int rc = 0;
  while(ros::ok())
  {
    nlohmann::json data;

    data["mqtt_alpha"] = arb_alpha;
    const std::string json_file = data.dump();
    
    int payload_len = json_file.length() + 1;
    char* payload = new char[ payload_len ];
    strcpy(payload, json_file.c_str());

    mqtt_drapebot_client.publish(payload, payload_len, mqtt_alpha_topic.c_str());    

    if ( rc != 0)
      ROS_ERROR_STREAM("MQTT publish function returned: " << rc);
    
    delete payload;

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
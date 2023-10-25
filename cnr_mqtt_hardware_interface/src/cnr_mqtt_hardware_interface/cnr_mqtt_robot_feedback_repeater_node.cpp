#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"mqtt_to_ros_abb_feedback_repeater");
    ros::NodeHandle nh;
    ros::Rate r(250); // 250 hz
    
    std::string feedback_ros_topic;
    if (!nh.getParam("/mqtt_hw/feedback_ros_topic",feedback_ros_topic))
    {
      feedback_ros_topic = "/abb/feedback/joint_states";
      
      ROS_WARN_STREAM("Ros ABB feedback topic " + nh.getNamespace() + "/mqtt_hw/feedback_ros_topic . Using default topic name: " + feedback_ros_topic);
    }


    ros::Publisher jointStatePublisher = nh.advertise<sensor_msgs::JointState>(feedback_ros_topic, 1);

    // ---- MQTT params ----
    std::string client_id = "mqtt_robot_feedback_repeater_node";

    std::string host_str;
    if (!nh.getParam("/mqtt_hw/host",host_str))
    {
      host_str = "localhost";
      ROS_WARN_STREAM("broker_address not found under " + nh.getNamespace() + "mqtt_hw/host . Using defalut broker address: "+ host_str);
    }

    int port;
    if (!nh.getParam("/mqtt_hw/port",port))
    {
      port = 1883;
      ROS_WARN_STREAM("port not found under " + nh.getNamespace() + "mqtt_hw/port. Using defalut broker address: "+ std::to_string( port));      
    }    

    bool use_json;
    if (!nh.getParam("/mqtt_hw/use_json",use_json))
    {
      use_json = true;
      ROS_WARN_STREAM("use json flag not found " + nh.getNamespace() + "mqtt_hw/use_json. Using defalut json flag: true " );      
    }   

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);

    
    cnr::drapebot::MQTTDrapebotClientHw mqtt_drapebot_client(client_id.c_str(), host_str.c_str(), port, use_json);
    
    ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);
    
    std::string mqtt_feedback_topic;
    if (!nh.getParam("/mqtt_hw/feedback_mqtt_topic", mqtt_feedback_topic))
    {
      mqtt_feedback_topic = "mqtt_feedback_topic";
      ROS_WARN_STREAM("mqtt_feedback_topic not found under " + nh.getNamespace() + "mqtt_hw/mqtt_feedback_topic . Using defalut broker address: "+ mqtt_feedback_topic);  
    }

    ROS_INFO_STREAM("Subscribing to: " << mqtt_feedback_topic);
    if (mqtt_drapebot_client.subscribe(NULL, mqtt_feedback_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_feedback_topic );
      return -1;
    }


    std::vector<std::string> joint_names;
    if(!nh.getParam("/mqtt_hw/joint_names",joint_names))
    {
      ROS_WARN_STREAM("joint_names: " << nh.getNamespace() );
    }

    sensor_msgs::JointState jointState;
   
    for( std::string& name : joint_names )
      jointState.name.push_back(name);
 
    jointState.position.resize(jointState.name.size());
    jointState.velocity.resize(jointState.name.size());
    jointState.effort.resize(jointState.name.size());


    cnr::drapebot::drapebot_msg_hw last_msg;

    while(ros::ok())
    {
      if (mqtt_drapebot_client.loop() != 0 )
      {
        ROS_ERROR_STREAM("Error on Mosquitto loop function");
        return -1;
      }

      if (!mqtt_drapebot_client.isFirstMsgRec())
        ROS_WARN("First feedback message not received yet");
      else
      {
        mqtt_drapebot_client.getLastReceivedMessage(last_msg);      

        jointState.position.at(0) = last_msg.J1;
        jointState.position.at(1) = last_msg.J2;
        jointState.position.at(2) = last_msg.J3;
        jointState.position.at(3) = last_msg.J4;
        jointState.position.at(4) = last_msg.J5;
        jointState.position.at(5) = last_msg.J6;
        jointState.position.at(6) = last_msg.E0;

        jointStatePublisher.publish(jointState);        
      }
      
      r.sleep();
    }

}
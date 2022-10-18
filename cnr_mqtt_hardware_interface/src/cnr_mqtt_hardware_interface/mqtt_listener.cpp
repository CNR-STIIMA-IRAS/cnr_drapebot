#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_mqtt_publisher");
    ros::NodeHandle nh;
    
    int rate = 25;
    
    ros::Rate r(rate); // 10 hz

    // ---- MQTT params ----
    std::string client_id = "test_mqtt";
//     std::string host_str = "192.168.12.204";
    std::string host_str = "localhost";
    int port = 1883;
    std::string mqtt_command_topic = "/robot_1/command";
    std::string mqtt_feedback_topic = "/robot_1/feedback";
    
//     std::string mqtt_command_topic = "/robot_1/feedback";
//     std::string mqtt_feedback_topic = "/robot_1/command";

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClientHw mqtt_drapebot_client_(client_id.c_str(), host_str.c_str(), port);
    ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);

    if (mqtt_drapebot_client_.subscribe(NULL, mqtt_command_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_command_topic );
      return -1;
    }
    
    cnr::drapebot::drapebot_msg_hw m_;
    
    m_.E0 = 0.0;
    m_.J1 = -M_PI/2;
    m_.J2 = M_PI/2;
    m_.J3 = 0.0;
    m_.J4 = 0.0;
    m_.J5 = 0.0;
    m_.J6 = -1.0;
    
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("sine", 1000);
    double t=0.0;
    
    while (ros::ok())
    {
      
//       m_.J1 = sin(2*M_PI*t);
      m_.J1 = 5.0;
      
      t+=1.0/rate;
      ROS_INFO_STREAM_THROTTLE(.5,"joints: " << m_.E0  << ", " 
                                             << m_.J1  << ", " 
                                             << m_.J2  << ", " 
                                             << m_.J3  << ", " 
                                             << m_.J4  << ", " 
                                             << m_.J5  << ", " 
                                             << m_.J6  
      
      );
      
      mqtt_drapebot_client_.publish_with_tracking(mqtt_feedback_topic, m_ );
      
      std_msgs::Float64 msg;
      msg.data = m_.J1;
      
      chatter_pub.publish(msg);
      
      if (mqtt_drapebot_client_.loop() != 0 )
      {
          ROS_ERROR_STREAM("Error on Mosquitto loop function");
          return -1;
      }
      
      r.sleep();
    }

    return 0;
}

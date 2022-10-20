#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_mqtt_cmd");
    ros::NodeHandle nh;
    
    int rate = 25;
    
    ros::Rate r(rate); // 10 hz

    // ---- MQTT params ----
    std::string client_id_1 = "test_mqtt_cmd1";
    std::string client_id_2 = "test_mqtt_cmd2";
//     std::string host_str = "192.168.12.204";
    std::string host_str = "localhost";
    int port = 1883;
   
    std::string mqtt_command_topic_1 = "/robot_1/command";
    std::string mqtt_feedback_topic_1 = "/robot_1/feedback";
    
    std::string mqtt_command_topic_2 = "/robot_2/command";
    std::string mqtt_feedback_topic_2 = "/robot_2/feedback";

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id_1 << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClientHw mqtt_drapebot_client_1(client_id_1.c_str(), host_str.c_str(), port);
    ROS_INFO_STREAM("Connencted to: "<< client_id_1 << ": " << host_str);

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id_2 << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClientHw mqtt_drapebot_client_2(client_id_2.c_str(), host_str.c_str(), port);
    ROS_INFO_STREAM("Connencted to: "<< client_id_2 << ": " << host_str);

    if (mqtt_drapebot_client_1.subscribe(NULL, mqtt_feedback_topic_1.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_feedback_topic_1 );
      return -1;
    }
    if (mqtt_drapebot_client_2.subscribe(NULL, mqtt_feedback_topic_2.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_feedback_topic_2 );
      return -1;
    }
    
    cnr::drapebot::drapebot_msg_hw m_1;
    m_1.E0 = 1.0;
    m_1.J1 = 1.0;
    m_1.J2 = 1.0;
    m_1.J3 = 1.0;
    m_1.J4 = 1.0;
    m_1.J5 = 1.0;
    m_1.J6 = 1.0;
    
    cnr::drapebot::drapebot_msg_hw m_2;
    m_2.E0 = 2.0;
    m_2.J1 = 2.0;
    m_2.J2 = 2.2;
    m_2.J3 = 2.0;
    m_2.J4 = 2.0;
    m_2.J5 = 2.0;
    m_2.J6 = 2.0;
    
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("sine", 1000);
    double t=0.0;
    
    cnr::drapebot::drapebot_msg_hw m_1_fb;
    cnr::drapebot::drapebot_msg_hw m_2_fb;
    
    while (ros::ok())
    {
      
      if (mqtt_drapebot_client_1.loop() != 0 )
      {
          ROS_ERROR_STREAM("Error on Mosquitto loop function");
          return -1;
      }
      
      if (mqtt_drapebot_client_2.loop() != 0 )
      {
          ROS_ERROR_STREAM("Error on Mosquitto loop function");
          return -1;
      }
      
      if(mqtt_drapebot_client_1.isNewMessageAvailable())
      {
        mqtt_drapebot_client_1.getLastReceivedMessage(m_1_fb);
        
        ROS_INFO_STREAM_THROTTLE(1.,"fb joints: " << m_1_fb.E0  << ", " 
                                          << m_1_fb.J1  << ", " 
                                          << m_1_fb.J2  << ", " 
                                          << m_1_fb.J3  << ", " 
                                          << m_1_fb.J4  << ", " 
                                          << m_1_fb.J5  << ", " 
                                          << m_1_fb.J6  );      
      }
      else
        ROS_WARN_THROTTLE(1.0,"no new feedback message available ... not good");

      if(mqtt_drapebot_client_2.isNewMessageAvailable())
      {
        mqtt_drapebot_client_2.getLastReceivedMessage(m_2_fb);
        
        ROS_INFO_STREAM_THROTTLE(1.,"fb joints: " << m_2_fb.E0  << ", " 
                                          << m_2_fb.J1  << ", " 
                                          << m_2_fb.J2  << ", " 
                                          << m_2_fb.J3  << ", " 
                                          << m_2_fb.J4  << ", " 
                                          << m_2_fb.J5  << ", " 
                                          << m_2_fb.J6  );
      }
      else
        ROS_WARN_THROTTLE(1.0,"no new feedback message available ... not good");
      
      
//       mqtt_drapebot_client_1.publish_with_tracking(mqtt_command_topic_1, m_1 );
//       mqtt_drapebot_client_2.publish_with_tracking(mqtt_command_topic_2, m_2 );
      
      std_msgs::Float64 msg;
      msg.data = m_1.J1;
      
      chatter_pub.publish(msg);
      

      
      r.sleep();
    }

    return 0;
}


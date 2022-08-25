#include <ros/ros.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_mqtt_publisher");
    ros::NodeHandle nh;
    ros::Rate r(10); // 10 hz

    // ---- MQTT params ----
    std::string client_id = "egm_joint_state_to_mqtt_controller";
    std::string host_str = "192.168.125.1";
    int port = 1883;
    std::string mqtt_feedback_topic = "/robot_1/feedback";

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClient mqtt_drapebot_client_(client_id.c_str(), host_str.c_str(), port);
    ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);

    if (mqtt_drapebot_client_.subscribe(NULL, mqtt_feedback_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_feedback_topic );
      return -1;
    }

    char data[] = "a";
    int payload_len_ = sizeof(data);
    void* payload_ = malloc(payload_len_);
    memcpy(payload_,data,payload_len_);

    while (ros::ok())
    {
        int rc = mqtt_drapebot_client_.publish(payload_, payload_len_, mqtt_feedback_topic.c_str() );
        if ( rc != 0)
            ROS_ERROR_STREAM("returned " << rc);

        if (mqtt_drapebot_client_.loop() != 0 )
        {
            ROS_ERROR_STREAM("Error on Mosquitto loop function");
            return -1;
        }
        r.sleep();
    }

    return 0;
}
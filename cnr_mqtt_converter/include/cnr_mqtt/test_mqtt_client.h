#pragma once
#include <mosquittopp.h>
#include <ros/ros.h>

#define MAX_PAYLOAD 50
#define DEFAULT_KEEP_ALIVE 60

struct message_struct {
    int n_joints;
    double joint_values[6];
    char joint_names[6][30];  
} mqtt_msg;   

// struct message_struct {
//     int n_joints;
//     double *joint_values;
//     char **joint_names;  
// } mqtt_msg;   


class test_mqtt_client : public mosqpp::mosquittopp
{
public:
    test_mqtt_client (const char *id, const char *host, int port);
    ~test_mqtt_client();

    void on_connect(int rc);
    void on_message(const struct mosquitto_message *message);
    void on_subscribe(int mid, int qos_count, const int *granted_qos);
    
    double joint_values[6];
};



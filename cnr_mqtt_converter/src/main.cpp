#include "ros/ros.h"
#include "cnr_mqtt/test_mqtt_client.h"


#define CLIENT_ID "Client_ID2"
#define BROKER_ADDRESS "localhost"
#define MQTT_PORT 1883;
#define MQTT_TOPIC_PUB "feedback"
#define MQTT_TOPIC_SUB "command"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  int rc;

  char client_id[] = CLIENT_ID;
  char host[] = BROKER_ADDRESS;
  int port = MQTT_PORT;
  
  mosquitto_lib_init();

  test_mqtt_client client(client_id, host, port);
  
  mosqpp::lib_init();

  if (argc > 1)
      strcpy (host, argv[1]);
  
  ros::Rate r = 10;
  client.subscribe(NULL, MQTT_TOPIC_SUB);  
  
  int n_joints = 6;
  
  message_struct m;
  
  m.n_joints = n_joints;
  
  std::vector<std::string> joint_names;
  
  for (int i=0;i<n_joints;i++)
  {    
    joint_names.push_back("joint_"+std::to_string(i));
  }  
  
  srand( (unsigned)time( NULL ) );
  
  for(int i =0; i<n_joints; i++)
  {
    strcpy(m.joint_names[i], joint_names[i].c_str());
    m.joint_values[i] = ((double) rand() / (RAND_MAX)) * 2 - 1;    
  }
  
  size_t message_size = sizeof(m);
  
  void* payload = malloc( message_size );
  
  while(ros::ok())
  {
    client.loop();
    
    for(int i =0; i<n_joints; i++)
      m.joint_values[i] = client.joint_values[i];    
    
    memcpy(payload, &m, message_size);  
    
    ROS_INFO_STREAM("size payload: "<< sizeof(payload));
    ROS_INFO_STREAM("size m: "<< sizeof(m));
    
    client.publish(NULL, MQTT_TOPIC_PUB, message_size, payload);
    
    r.sleep();
  }
  

  mosqpp::lib_cleanup();

  return 0;
}


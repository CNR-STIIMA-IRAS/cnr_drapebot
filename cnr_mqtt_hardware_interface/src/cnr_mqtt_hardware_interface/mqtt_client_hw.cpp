#include <ros/ros.h>
#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>


namespace  cnr
{
  namespace drapebot
  {

    void DrapebotMsgDecoderHw::vec_to_msg(std::vector<double> v, cnr::drapebot::drapebot_msg_hw* msg)
    {
      msg->J1 = v.at(0);
      msg->J2 = v.at(1);
      msg->J3 = v.at(2);
      msg->J4 = v.at(3);
      msg->J5 = v.at(4);
      msg->J6 = v.at(5);
      msg->E0 = v.at(6);
    }
    
    
    
    void DrapebotMsgDecoderHw::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
    {
      
      int n_joints = MSG_LENGTH; // TODO::verify
      
      int message_size = n_joints * sizeof(mqtt_msg_->E0) + sizeof(mqtt_msg_->count); 
      
      std::vector<double> joints;
      
      if ( msg->payloadlen == message_size )
      {
        
        for(size_t id=0; id< n_joints*sizeof(mqtt_msg_->E0)/sizeof(double); id++)        
        {
          double j;
          memcpy(&j, (char *)msg->payload + id * (sizeof(double)), sizeof(double));
          joints.push_back(j);
        }
        
        vec_to_msg(joints,mqtt_msg_);
        
        unsigned long int count;
        memcpy(&count, (char *)msg->payload + n_joints * sizeof(mqtt_msg_->E0), sizeof(unsigned long int));  
        mqtt_msg_->count = count;
        
        setNewMessageAvailable(true); 
        setDataValid(true);
      }
      else
      {
        ROS_WARN("The message received from MQTT has a wrong length");
        setNewMessageAvailable(false);
        setDataValid(false);
      }
                  
    }
    
    void DrapebotMsgEncoderHw::on_publish(struct mosquitto *mosq, void *obj, int mid)
    {
      // Nothing to do here
    }
    

    MQTTDrapebotClientHw::MQTTDrapebotClientHw(const char *id, const char *host, int port, int keepalive)
    {
      try
      {
        mqtt_msg_enc_ = new cnr::drapebot::drapebot_msg_hw;
        mqtt_msg_dec_ = new cnr::drapebot::drapebot_msg_hw;

        drapebot_msg_hw_encoder_ = new cnr::drapebot::DrapebotMsgEncoderHw(mqtt_msg_enc_);
        drapebot_msg_hw_decoder_ = new cnr::drapebot::DrapebotMsgDecoderHw(mqtt_msg_dec_);

        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, drapebot_msg_hw_encoder_, drapebot_msg_hw_decoder_);
        
        msg_count_cmd = 0;
        first_message_received_ = false;
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in MQTTDrapebotClientHw constructor: " <<  e.what() );
      }
    }

    MQTTDrapebotClientHw::~MQTTDrapebotClientHw()
    {  
      delete mqtt_msg_dec_;
      delete mqtt_msg_enc_;
      delete drapebot_msg_hw_decoder_;
      delete drapebot_msg_hw_encoder_;
      delete mqtt_client_;
    }
    
    void MQTTDrapebotClientHw::publish_with_tracking(const std::string& cmd_topic, drapebot_msg_hw& m)
    {
      msg_count_cmd += 1;
      m.count = msg_count_cmd;
    
      int message_size_ = sizeof(m);
            
      void* payload_ = malloc( message_size_ );
      memcpy(payload_, &m, message_size_);  
      
      
      int n = cmd_topic.length();
      char topic[n+ 1];
      strcpy(topic, cmd_topic.c_str());
            
      publish(payload_, message_size_, topic);
    }

    int MQTTDrapebotClientHw::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      
      else
        return -1;
    }

    int MQTTDrapebotClientHw::loop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop();
      else
        return -1;
    }

    int MQTTDrapebotClientHw::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      else
        return -1;
    }
    
    int MQTTDrapebotClientHw::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);
      else
        return -1;
    }

    int MQTTDrapebotClientHw::publish(const void* payload, int& payload_len, const char* topic_name)
    {        
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      else
        return -1;
    }
    bool MQTTDrapebotClientHw::getLastReceivedMessage(cnr::drapebot::drapebot_msg_hw& last_msg)
    {
      if (drapebot_msg_hw_decoder_->isNewMessageAvailable() )
      {
        
        last_msg = *mqtt_msg_dec_;
        
        drapebot_msg_hw_decoder_->setNewMessageAvailable(false);
        return true;
      }
      else
      {
        ROS_WARN("New message not available.");
        return false;
      }
      return true;
    }

    bool MQTTDrapebotClientHw::isNewMessageAvailable()
    {
      if (drapebot_msg_hw_decoder_ != NULL)
        return drapebot_msg_hw_decoder_->isNewMessageAvailable();
      else
      {
        ROS_ERROR("drapebot_msg_hw_decoder_ == NULL . return false");
        return false;
      }
    }

    bool MQTTDrapebotClientHw::isDataValid()
    {
      if (drapebot_msg_hw_decoder_ != NULL)
        return drapebot_msg_hw_decoder_->isDataValid();
      else
        return false;
    }
    
    
    bool MQTTDrapebotClientHw::getFirstMessageStatus()
    {
      return first_message_received_;
    }
    

  }
}

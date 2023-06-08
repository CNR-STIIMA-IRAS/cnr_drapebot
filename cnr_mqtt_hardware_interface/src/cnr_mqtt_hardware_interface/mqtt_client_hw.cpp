#include <ros/ros.h>
#include <cnr_mqtt_hardware_interface/mqtt_client_hw.h>
#include <cnr_mqtt_hardware_interface/json.hpp>


namespace  cnr
{
  namespace drapebot
  {

    void DrapebotMsgDecoderHw::vec_to_msg(const std::vector<double>& v, cnr::drapebot::drapebot_msg_hw* msg)
    {
      msg->J1 = v.at(0);
      msg->J2 = v.at(1);
      msg->J3 = v.at(2);
      msg->J4 = v.at(3);
      msg->J5 = v.at(4);
      msg->J6 = v.at(5);
      msg->E0 = v.at(6);
    }
    
    
    void DrapebotMsgDecoderHw::on_message(const struct mosquitto_message *msg)
    {

      if(use_json_)
      {   
        char buf[msg->payloadlen];
        memcpy(buf, msg->payload, msg->payloadlen);

        std::string buf_str(buf);
        
        if(mtx_.try_lock_for(std::chrono::milliseconds(4)))
        {        
          nlohmann::json data = nlohmann::json::parse(buf_str);

          mqtt_msg_->J1 = data["J0"];
          mqtt_msg_->J2 = data["J1"];
          mqtt_msg_->J3 = data["J2"];
          mqtt_msg_->J4 = data["J3"];
          mqtt_msg_->J5 = data["J4"];
          mqtt_msg_->J6 = data["J5"];
          mqtt_msg_->E0 = data["E0"];        
          mqtt_msg_-> count = data["count"];

          if (!first_message_rec_)
            first_message_rec_ = true;

          mtx_.unlock();
        }
        else
          ROS_WARN("Can't lock mutex in DrapebotMsgDecoderHw::on_message.");

      }
      else
      {
        ROS_DEBUG_STREAM_THROTTLE(2.0,"using basic mqtt to decode message");
        
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
          
          if(mtx_.try_lock_for(std::chrono::milliseconds(4)))
          {
            unsigned long int count;
            memcpy(&count, (char *)msg->payload + n_joints * sizeof(mqtt_msg_->E0), sizeof(unsigned long int));  
            mqtt_msg_->count = count;
            
            if (!first_message_rec_)
              first_message_rec_ = true;

            mtx_.unlock();
          }
        }
        else
        {
          ROS_WARN("The message received from MQTT has a wrong length");
          setNewMessageAvailable(false);
          setDataValid(false);
        }
      }
    }
    
    void DrapebotMsgEncoderHw::on_publish(int mid)
    {
      // Nothing to do here
    }
    

    MQTTDrapebotClientHw::MQTTDrapebotClientHw(const char *id, const char *host, int port, int keepalive, bool use_json)
    {
      try
      {
        mqtt_msg_enc_ = new cnr::drapebot::drapebot_msg_hw;
        mqtt_msg_dec_ = new cnr::drapebot::drapebot_msg_hw;

        drapebot_msg_hw_encoder_ = new cnr::drapebot::DrapebotMsgEncoderHw(mqtt_msg_enc_);
        drapebot_msg_hw_decoder_ = new cnr::drapebot::DrapebotMsgDecoderHw(mqtt_msg_dec_, use_json);

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
      
      int rc = publish(payload_, message_size_, topic);
      if ( rc != 0)
        ROS_ERROR_STREAM("MQTTDrapebotClientHw::publish_with_tracking returned code:" << rc);
        
    }
    

    int MQTTDrapebotClientHw::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      
      else
        return -1;
    }

    int MQTTDrapebotClientHw::loop(int timeout)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop(timeout);
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
        if (drapebot_msg_hw_decoder_ != NULL)
        {
          if (!drapebot_msg_hw_decoder_->isFirstMsgRec())
          {
            ROS_WARN_THROTTLE(2.0,"First message not received yet.");
            return false;
          }  

        // if (drapebot_msg_hw_decoder_->isNewMessageAvailable() )
        // {
          if(drapebot_msg_hw_decoder_->mtx_.try_lock_for(std::chrono::milliseconds(4)))
          {
            last_msg.E0 = mqtt_msg_dec_->E0;
            last_msg.J1 = mqtt_msg_dec_->J1;
            last_msg.J2 = mqtt_msg_dec_->J2;
            last_msg.J3 = mqtt_msg_dec_->J3;
            last_msg.J4 = mqtt_msg_dec_->J4;
            last_msg.J5 = mqtt_msg_dec_->J5;
            last_msg.J6 = mqtt_msg_dec_->J6;
            last_msg.count = mqtt_msg_dec_->count;

            
            //drapebot_msg_hw_decoder_->setNewMessageAvailable(false);
            
            drapebot_msg_hw_decoder_->mtx_.unlock();
            return true;
          }
          else
          {
            ROS_WARN("Can't lock mutex in MQTTDrapebotClientHw::getLastReceivedMessage. Last message received from MQTT not recovered.");
            return false;
          }

        // }
        // else
        // {
        //   ROS_WARN("New message not available.");
        //   return false;
        // }
        // return true;
        }
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

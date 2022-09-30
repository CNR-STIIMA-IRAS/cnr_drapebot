/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2022 National Council of Research of Italy (CNR)
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


#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <mqtt_to_joint_position_controller/mqtt_to_joint_position_controller.h>


namespace drapebot_controller
{
  
  MQTTToPositionController::MQTTToPositionController()
  {

  }


  MQTTToPositionController::~MQTTToPositionController()
  {
    delete mqtt_drapebot_client_;  
  }


  bool MQTTToPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n)
  {
    try
    {
      ctrl_.init(hw,n); 

      // ---- MQTT params ----

      std::string client_id;
      if (!n.getParam("client_id",client_id))
      {
        client_id = "Client_ID";
        ROS_WARN_STREAM("client id not found under " + n.getNamespace() + "/client_id . Using defalut client ID: " + client_id);
      }
      
      std::string host_str;
      if (!n.getParam("broker_address",host_str))
      {
        host_str = "localhost";
        ROS_WARN_STREAM("broker_address not found under " + n.getNamespace() + "/broker_address . Using defalut broker address: "+ host_str);
      }
  
      int port;
      if (!n.getParam("port",port))
      {
        port = 1883;
        ROS_WARN_STREAM("port not found under " + n.getNamespace() + "/port. Using defalut broker address: "+ std::to_string(port));      
      }
      
      if (!n.getParam("mqtt_command_topic",mqtt_command_topic_))
      {
        mqtt_command_topic_ = "mqtt_command_topic";
        ROS_WARN_STREAM("mqtt_command_topic not found under " + n.getNamespace() + "/mqtt_command_topic . Using defalut broker address: "+ mqtt_command_topic_);      
      }
      
      ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
      mqtt_drapebot_client_ = new cnr::drapebot::MQTTDrapebotClient(client_id.c_str(), host_str.c_str(), port);
      ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);
          
      j_pos_command_.resize(MSG_LENGTH-1); // The seventh axis is not necessary in DrapeCell setup
      j_pos_command_ = *ctrl_.commands_buffer_.readFromNonRT();
      
      ROS_INFO_STREAM("mqtt_command_topic : " << mqtt_command_topic_);
              
      first_cycle_ = true;
      
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM( "Thrown exception: " << e.what() );
      return false;
    }

    ROS_INFO_STREAM("Controller MQTTToPositionController Initialized ! ");
    
    return true;
  }


  void MQTTToPositionController::starting(const ros::Time& time)
  { 
    ctrl_.starting(time);
    if (mqtt_drapebot_client_->subscribe(NULL, mqtt_command_topic_.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_command_topic_ );
      return;
    }

    ROS_INFO_STREAM("Subscribing topic: "<< mqtt_command_topic_);
  }


  void MQTTToPositionController::stopping(const ros::Time& time) 
  {
    ctrl_.stopping(time);
    if ( mqtt_drapebot_client_->unsubscribe(NULL, mqtt_command_topic_.c_str()) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto unsubscribe topic: " << mqtt_command_topic_);
      return;
    } 
    ROS_INFO_STREAM("Unsubscribing topic: "<< mqtt_command_topic_);
  }
  

  void MQTTToPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    if (mqtt_drapebot_client_->loop() != 0 )
    {
      ROS_ERROR_STREAM("Error on Mosquitto loop function");
      return;
    }
    
    if (first_cycle_)
    {
      first_cycle_ = false;
      j_pos_command_  = *ctrl_.commands_buffer_.readFromNonRT();
    }
      
    // Read the new MQTT message and send the command to the robot
      
    if (mqtt_drapebot_client_->isNewMessageAvailable() && mqtt_drapebot_client_->isDataValid()) 
    {
      memset(&command_from_mqtt_,0x0,sizeof(cnr::drapebot::drapebot_msg));

      if (!mqtt_drapebot_client_->getLastReceivedMessage(command_from_mqtt_))
      {
        ROS_ERROR("Can't recover the last received message");
        return;
      }

      for (size_t i=0; i<(MSG_LENGTH-1); i++)
        j_pos_command_[i] =  command_from_mqtt_.joints_values_[i]; 

    }
    else
      ROS_DEBUG_THROTTLE(2.0,"No message available");
    
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_0 : "<< j_pos_command_[0]);
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_1 : "<< j_pos_command_[1]);
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_2 : "<< j_pos_command_[2]);
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_3 : "<< j_pos_command_[3]);
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_4 : "<< j_pos_command_[4]);
    ROS_DEBUG_STREAM_THROTTLE(2.0,"joint command_5 : "<< j_pos_command_[5]);
    
    ctrl_.commands_buffer_.writeFromNonRT(j_pos_command_);
    ctrl_.update(time,period);
  }
    

  void MQTTToPositionController::waiting(const ros::Time& time)
  {
    ctrl_.waiting(time);
  }


  void MQTTToPositionController::aborting(const ros::Time& time) 
  {
    ctrl_.aborting(time);
  }

} //end drapebot_controller

PLUGINLIB_EXPORT_CLASS(drapebot_controller::MQTTToPositionController, controller_interface::ControllerBase)


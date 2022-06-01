#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>

#ifndef __MQTT_TO_JOINT_POSITION_CONTROLLER_H__
#define __MQTT_TO_JOINT_POSITION_CONTROLLER_H__

namespace drapebot_controller 
{
    class MQTTToPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
        MQTTToPositionController();
        ~MQTTToPositionController();

        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n) final;
        void update(const ros::Time& time, const ros::Duration& period) final;
        void starting(const ros::Time& time) final;
        void stopping(const ros::Time& time) final;

    private:
        std::vector<hardware_interface::JointHandle> joints_handle_;

        drapebot::MQTTClient* mqtt_client_;

        std::vector<double> j_pos_feedback_;

        std::string mqtt_command_topic_;
        std::string mqtt_feedback_topic_;
        std::string mqtt_out_feedback_topic_;
    };

}//namespace

#endif
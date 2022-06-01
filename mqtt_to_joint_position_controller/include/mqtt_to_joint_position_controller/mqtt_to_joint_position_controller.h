#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <position_controllers/joint_group_position_controller.h>
#include <velocity_controllers/joint_group_velocity_controller.h>
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

        virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n) final;
        virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) final;
        void update(const ros::Time& time, const ros::Duration& period) final;
        void starting(const ros::Time& time) final;
        void stopping(const ros::Time& time) final;
        void waiting(const ros::Time& time) final;
        void aborting(const ros::Time& time) final;

    private:
        std::vector<hardware_interface::JointHandle> joints_handle_;

        drapebot::MQTTClient* mqtt_client_;

        std::vector<double> j_pos_feedback_;

        std::string mqtt_command_topic_;
        std::string mqtt_feedback_topic_;
        
        position_controllers::JointGroupPositionController ctrl_;
    };

}//namespace

#endif

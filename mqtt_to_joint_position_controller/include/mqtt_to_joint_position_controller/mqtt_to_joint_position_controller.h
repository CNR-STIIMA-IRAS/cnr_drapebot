#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#ifndef __MQTT_TO_JOINT_POSITION_CONTROLLER_H__
#define __MQTT_TO_JOINT_POSITION_CONTROLLER_H__

namespace controller_ns 
{
    class MQTTToPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;

    private:
        std::vector<hardware_interface::JointHandle> joints_handle_;
        //static const double gain_ = 1.25;
        //static const double setpoint_ = 3.00;
    };

}//namespace

#endif
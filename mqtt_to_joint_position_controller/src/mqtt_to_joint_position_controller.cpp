
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <mqtt_to_joint_position_controller/mqtt_to_joint_position_controller.h>



namespace controller_ns
{

    bool MQTTToPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n)
    {
        // get joint name from the parameter server
        std::string my_joint;

				std::string jnt_namespace = "/egm/joint_group_mqtt_to_position_controller/joints";
				std::vector<std::string> joints_names; 
				if (!n.getParam(jnt_namespace, joints_names))
				{
					ROS_ERROR("Could not find the namespace.");
					return false;
				}

        for (const std::string& jnt_name : joints_names)
        	// get the joint object to use in the realtime loop
					joints_handle_.push_back( hw->getHandle(jnt_name) );  // throws on failure
        
        return true;
    }

    void MQTTToPositionController::update(const ros::Time& time, const ros::Duration& period)
    {
        //double error = setpoint_ - joint_.getPosition();
        //joint_.setCommand(error * gain_);

				for(const hardware_interface::JointHandle& jnt_handle : joints_handle_ )
				{
					jnt_handle.setCommand();
					//do something	
				} 

    }

    void MQTTToPositionController::starting(const ros::Time& time)
    { 

    }

    void MQTTToPositionController::stopping(const ros::Time& time) 
    {

    }
}

PLUGINLIB_EXPORT_CLASS(controller_ns::MQTTToPositionController, controller_interface::ControllerBase)


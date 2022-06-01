#pragma once

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The DeformationCtrl class
 */
class DeformationCtrl: public cnr::control::JointCommandController<
        hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
//         hardware_interface::JointHandle, hardware_interface::PositionJointInterface>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DeformationCtrl();
  bool doInit();
  bool doUpdate  (const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  std::mutex m_mtx;


  ect::FilteredVectorXd vel_fitler_sp_;
  ect::FilteredVectorXd wrench_fitler_;

  rosdyn::VectorXd dq_sp_;
  rosdyn::VectorXd q_sp_;
  rosdyn::VectorXd ddq_;
  rosdyn::VectorXd dq_;
  rosdyn::VectorXd q_;

  geometry_msgs::PoseStamped pose_sp_;
  Eigen::Affine3d T_base_targetpose_;

  bool use_cartesian_reference_;

  bool first_cycle_;

  rosdyn::ChainPtr chain_bs_;
  rosdyn::ChainPtr chain_bt_;

  void setTargetPoseCallback   (const geometry_msgs::PoseStampedConstPtr&    msg );
  void setTargetJointsCallback (const sensor_msgs::JointStateConstPtr&       msg );
  
};


}
}

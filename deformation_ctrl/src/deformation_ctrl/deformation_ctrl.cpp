#include <deformation_ctrl/deformation_ctrl.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <rosdyn_core/primitives.h>
#include <name_sorting/name_sorting.h>
#include <tf2_eigen/tf2_eigen.h>
#include <deformation_ctrl/utils.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::DeformationCtrl  , controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief DeformationCtrl::DeformationCtrl
 */
DeformationCtrl::DeformationCtrl()
{
}

/**
 * @brief DeformationCtrl::doInit
 * @return
 */
bool DeformationCtrl::doInit()
{
  
  GET_AND_DEFAULT( this->getControllerNh(), "use_cartesian_reference", use_cartesian_reference_, false);
  if(use_cartesian_reference_)
  {
    std::string pose_target;
    GET_AND_RETURN( this->getControllerNh(), "pose_target"  , pose_target);
    this->template add_subscriber<geometry_msgs::PoseStamped>(pose_target,5,boost::bind(&DeformationCtrl::setTargetPoseCallback,this,_1), false);
  }
  else
  {
    std::string joint_target;
    GET_AND_RETURN( this->getControllerNh(), "joint_target_topic"  , joint_target);
    this->template add_subscriber<sensor_msgs::JointState>(joint_target,5,boost::bind(&DeformationCtrl::setTargetJointsCallback,this,_1), false);
  }
  
  this->setPriority(this->Q_PRIORITY);
  {
      ect::FilteredVectorXd::Value dead_band;
      ect::FilteredVectorXd::Value saturation;
      ect::FilteredVectorXd::Value init_value;

      dead_band = 0.0 * this->chain().getDQMax();
      saturation = this->chain().getDQMax();
      init_value = dead_band;
      if(!vel_fitler_sp_.activateFilter ( dead_band, saturation, (10.0 / 2.0 / M_PI), this->m_sampling_period, init_value ))
      {
        CNR_RETURN_FALSE(this->logger());
      }
  }
  dq_sp_ = vel_fitler_sp_.getUpdatedValue();

  first_cycle_ = true;  
  
  urdf::Model urdf_model;
  if ( !urdf_model.initParam ( "/robot_description" ) ) {
      ROS_ERROR ( "Urdf robot_description '%s' does not exist", (  this->getControllerNamespace()+"/robot_description" ).c_str() );
      return false;
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  std::string robot_base_frame;
  GET_AND_RETURN( this->getControllerNh(), "robot_base_frame"  , robot_base_frame);
  std::string robot_tip_frame;
  GET_AND_RETURN( this->getControllerNh(), "robot_tip_frame"   , robot_tip_frame);
  
  chain_bt_ = rosdyn::createChain ( urdf_model,robot_base_frame, robot_tip_frame   , gravity );

  q_sp_ .setZero();
  dq_sp_.setZero();
  q_    .setZero();
  dq_   .setZero();
  ddq_  .setZero();

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief DeformationCtrl::doStarting
 * @param time
 */
bool DeformationCtrl::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  
  q_sp_  = this->getPosition();
  dq_sp_ = this->getVelocity();
  q_  = q_sp_;
  dq_ = dq_sp_;
  this->setCommandPosition(q_);
  this->setCommandVelocity(dq_);
  T_base_targetpose_ = chain_bt_->getTransformation(q_sp_);
  pose_sp_.pose = tf2::toMsg (T_base_targetpose_);
  
  dq_sp_ = 0 * this->getVelocity();

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief DeformationCtrl::stopping
 * @param time
 */
bool DeformationCtrl::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief DeformationCtrl::doUpdate
 * @param time
 * @param period
 * @return
 */
bool DeformationCtrl::doUpdate(const ros::Time& time, const ros::Duration& period)
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
    std::stringstream report;
    std::lock_guard<std::mutex> lock(m_mtx);
    
    if (first_cycle_)
    {
      Eigen::Vector3d x = this->chainState().toolPose().translation();
      q_sp_ = this->getPosition();
      dq_sp_ = this->getVelocity();
      ROS_INFO_STREAM("\n\n[DeformationCtrl] : initial cmd j_pos deformation_ctrl: " << q_sp_.transpose());
      T_base_targetpose_ = chain_bt_->getTransformation(q_sp_);
      pose_sp_.pose = tf2::toMsg (T_base_targetpose_);
      first_cycle_ = false;
    }
    
    if(use_cartesian_reference_)
      tf2::fromMsg (pose_sp_.pose, T_base_targetpose_);
    else
      T_base_targetpose_ = chain_bt_->getTransformation(q_sp_);
    
    q_=q_sp_;
    
    CNR_WARN_THROTTLE(this->logger(),2.0,"[DeformationCtrl] : command position] \n"<<q_sp_);
    
    this->setCommandPosition( q_sp_ );
//     this->setCommandVelocity( dq_);
    
    CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
    
  }


  void DeformationCtrl::setTargetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    try
    {
      pose_sp_ = *msg;
//       CNR_FATAL(this->logger(),"new cartesian setpoint recived:"<<pose_sp_);
    }
    catch(...)
    {
      ROS_ERROR("Something wrong in target callback");
    }
  }

void DeformationCtrl::setTargetJointsCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(this->jointNames(),tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      CNR_ERROR(this->logger(),"joints not found");
      return;
    }
    for (unsigned int iAx=0;iAx<q_sp_.rows();iAx++)
    {
      q_sp_(iAx)=tmp_msg.position.at(iAx);
      dq_sp_(iAx)=tmp_msg.velocity.at(iAx);
    }

  }
  catch(...)
  {
    CNR_ERROR(this->logger(),"Something wrong in target callback");
  }
}

  }
  }

#include <ros/ros.h>
#include <ros/time.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_controller_interface/utils/utils.h>
#include <cnr_open_loop_delta_position_controller/cnr_open_loop_delta_position_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::OpenLoopDeltaPositionController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{

//!
//! \brief OpenLoopDeltaPositionController::doInit
//! \return
//!
bool OpenLoopDeltaPositionController::doInit()
{
  CNR_TRACE_START(this->m_logger);
  if(!this->getControllerNh().getParam("setpoint_topic_name", m_setpoint_topic_name))
  {
    CNR_RETURN_FALSE(this->m_logger,"The param '"+this->getControllerNamespace()+"/setpoint_topic_name' does not exist");
  }
  bool setpoint_watchdog=true;
  if(!this->getControllerNh().getParam("enable_setpoint_watchdog", setpoint_watchdog))
  {
    CNR_DEBUG(this->m_logger,"The param '"+this->getControllerNamespace()+"/enable_setpoint_watchdog' does not exist, enable watchdog by default");
    setpoint_watchdog=true;
  }
  this->template add_subscriber<sensor_msgs::JointState>(m_setpoint_topic_name, 1,
      boost::bind(&OpenLoopDeltaPositionController::callback, this, _1),
      setpoint_watchdog);

  this->setPriority(this->Q_PRIORITY);


  CNR_DEBUG(this->m_logger, "Controller ' "+this->getControllerNamespace()+"' controls the following joint: "
                     + cnr::control::to_string(this->jointNames()));
  CNR_DEBUG(this->m_logger, "Controller ' "+this->getControllerNamespace()+"' get the setpoint from the topic: '"
                     + m_setpoint_topic_name + "'");
  
  
  delta_deadband_.resize(this->nAx()) ;
  delta_filt_.resize(this->nAx()) ;
  old_pos_.resize(this->nAx()) ;
  delta_trg_.resize(this->nAx()) ;
  
  delta_deadband_.setZero() ;
  delta_filt_    .setZero() ;
  old_pos_       .setZero() ;
  delta_trg_.     setZero()  ;
  
  std::vector<double> delta_deadband(this->nAx(),0);
  this->getControllerNh().getParam("delta_deadband", delta_deadband );
  delta_deadband_ =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(delta_deadband.data(), delta_deadband.size()) ;

  
  double omega;
  if(!this->getControllerNh().getParam("omega_filt",omega))
    {
       CNR_INFO(this->logger(),"Omega filt not set. default = 100");
       omega = 100.0;
    }
  ect::FilteredVectorXd::Value dead_band;
  ect::FilteredVectorXd::Value saturation;
  ect::FilteredVectorXd::Value init_value;

  dead_band  = delta_deadband_;
  saturation = 1000.0 * dead_band;
  init_value = 0.0 * dead_band;
  if(!delta_fitler_.activateFilter ( dead_band, saturation, (omega / (2 * M_PI)), this->m_sampling_period, init_value ))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  delta_filt_ = delta_fitler_.getUpdatedValue();
  
  
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopDeltaPositionController::doStarting
//! \return
//!
bool OpenLoopDeltaPositionController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopDeltaPositionController::doStopping
//! \return
//!
bool OpenLoopDeltaPositionController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopDeltaPositionController::doUpdate
//! \return
//!
bool OpenLoopDeltaPositionController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

//!
//! \brief OpenLoopDeltaPositionController::extractJoint
//! \param msg
//! \return
//!
bool OpenLoopDeltaPositionController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  auto target = this->getCommandPosition();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < this->jointNames().size(); iAx++)
    {
      if(msg.name.at(iJoint) == this->jointNames().at(iAx))
      {
        if(msg.position.size() > (iJoint))
        {
          eigen_utils::at(target,iAx) = msg.position.at(iJoint);
          cnt++;
        }
        else
        {
          return false;
        }
      }
    }
  }

  
  for(int jj=0;jj<target.size();jj++)
  {
    delta_trg_(jj) = target(jj)-old_pos_(jj);   
    
    old_pos_(jj)=target(jj);
  }
  
  
  
  delta_fitler_.update(target);
  delta_filt_ = delta_fitler_.getUpdatedValue();
  
  bool ok = (cnt == this->nAx());
  if( ok )
  {
    this->setCommandPosition(delta_filt_);
  }
  else
  {
    CNR_FATAL(this->m_logger, this->getControllerNamespace() + " command message dimension is wrong. found "+std::to_string(cnt)+" over "+std::to_string(this->nAx())+" joints");
    for (size_t iAx=0; iAx < this->jointNames().size(); iAx++)
    {
      CNR_FATAL(this->m_logger, this->getControllerNamespace() + " controlled joints: "+this->jointNames().at(iAx));
    }
    CNR_FATAL(this->m_logger, this->getControllerNamespace() + " received message\n"<< msg);
  }
  return ok;
}


//!
void OpenLoopDeltaPositionController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);
  if(extractJoint(*msg))
  {
    m_configured = true;
  }
  
  
  CNR_RETURN_OK_THROTTLE_DEFAULT(this->m_logger, void());
}

}
}

// -*- C++ -*-
/*!
 * @file  MobileRobotKinematics.cpp
 * @brief Mobile robot kinematics component
 * @date $Date$
 *
 * $Id$
 */

#include "MobileRobotKinematics.h"
#include <math.h>
// Module specification
// <rtc-template block="module_spec">
static const char* mobilerobotkinematics_spec[] =
  {
    "implementation_id", "MobileRobotKinematics",
    "type_name",         "MobileRobotKinematics",
    "description",       "Mobile robot kinematics component",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Experimenta",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.axleTrack", "0.331",
    "conf.default.wheelRadius", "0.09525",
    // Widget
    "conf.__widget__.axleTrack", "text",
    "conf.__widget__.wheelRadius", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MobileRobotKinematics::MobileRobotKinematics(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_currentWheelAngleIn("currentWheelAngle", m_currentWheelAngle),
    m_targetVelocityIn("targetVelocity", m_targetVelocity),
    m_updatePoseIn("updatePose", m_updatePose),
    m_targetWheelVelocityOut("targetWheelVelocity", m_targetWheelVelocity),
    m_currentPoseOut("currentPose", m_currentPose)

    // </rtc-template>
{
  x = y = theta = 0;
}

/*!
 * @brief destructor
 */
MobileRobotKinematics::~MobileRobotKinematics()
{
}



RTC::ReturnCode_t MobileRobotKinematics::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("currentWheelAngle", m_currentWheelAngleIn);
  addInPort("targetVelocity", m_targetVelocityIn);
  addInPort("updatePose", m_updatePoseIn);
  
  // Set OutPort buffer
  addOutPort("targetWheelVelocity", m_targetWheelVelocityOut);
  addOutPort("currentPose", m_currentPoseOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("axleTrack", m_axleTrack, "0.331");
  bindParameter("wheelRadius", m_wheelRadius, "0.09525");
  // </rtc-template>

  m_currentWheelAngleIn.addConnectorDataListener(ON_BUFFER_WRITE,
						    new WheelAngleListener(this));
  m_updatePoseIn.addConnectorDataListener(ON_BUFFER_WRITE,
					  new UpdatePoseListener(this));
  m_targetVelocityIn.addConnectorDataListener(ON_BUFFER_WRITE,
					  new TargetVelocityListener(this));

  m_targetWheelVelocity.data.length(2);
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MobileRobotKinematics::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MobileRobotKinematics::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MobileRobotKinematics::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MobileRobotKinematics::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MobileRobotKinematics::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MobileRobotKinematics::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MobileRobotKinematics::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MobileRobotKinematics::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MobileRobotKinematics::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MobileRobotKinematics::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MobileRobotKinematics::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MobileRobotKinematicsInit(RTC::Manager* manager)
  {
    coil::Properties profile(mobilerobotkinematics_spec);
    manager->registerFactory(profile,
                             RTC::Create<MobileRobotKinematics>,
                             RTC::Delete<MobileRobotKinematics>);
  }
  
};



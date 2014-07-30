// -*- C++ -*-
/*!
 * @file  MobileRobotKinematics.h
 * @brief Mobile robot kinematics component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MOBILEROBOTKINEMATICS_H
#define MOBILEROBOTKINEMATICS_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>


using namespace RTC;


/*!
 * @class MobileRobotKinematics
 * @brief Mobile robot kinematics component
 *
 */
class MobileRobotKinematics
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  MobileRobotKinematics(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~MobileRobotKinematics();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  axleTrack
   * - DefaultValue: 0.331
   */
  double m_axleTrack;
  /*!
   * 
   * - Name:  wheelRadius
   * - DefaultValue: 0.09525
   */
  double m_wheelRadius;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedDoubleSeq m_currentWheelAngle;
  /*!
   */
  InPort<RTC::TimedDoubleSeq> m_currentWheelAngleIn;
  RTC::TimedVelocity2D m_targetVelocity;
  /*!
   */
  InPort<RTC::TimedVelocity2D> m_targetVelocityIn;
  RTC::TimedPose2D m_updatePose;
  /*!
   */
  InPort<RTC::TimedPose2D> m_updatePoseIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_targetWheelVelocity;
  /*!
   */
  OutPort<RTC::TimedDoubleSeq> m_targetWheelVelocityOut;
  RTC::TimedPose2D m_currentPose;
  /*!
   */
  OutPort<RTC::TimedPose2D> m_currentPoseOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

  double x, y, theta;

public:
  double getAxleTrack() const {return m_axleTrack;}
  double getWheelRadius() const {return m_wheelRadius;}
  double getX() {return x;}
  double getY() {return y;}
  double getTheta() {return theta;}
  void setX(const double a) {x=a;}
  void setY(const double a) {y=a;}
  void setTheta(const double a) {theta=a;}


  void writeCurrentPose(const RTC::Time &tm) {
    m_currentPose.tm = tm;
    m_currentPose.data.position.x = x;
    m_currentPose.data.position.y = y;
    m_currentPose.data.heading = theta;
    m_currentPoseOut.write();
  }

  void writeWheelVelocity(const RTC::Time &tm, const double wr, const double wl) {
	  m_targetWheelVelocity.tm = tm;
	  m_targetWheelVelocity.data[0] = wr;
	  m_targetWheelVelocity.data[1] = wl;
	  m_targetWheelVelocityOut.write();
  }
};


class WheelAngleListener
  : public ConnectorDataListenerT<RTC::TimedDoubleSeq>
{
private:
  MobileRobotKinematics *m_pRTC;
public:
  WheelAngleListener(MobileRobotKinematics* pRTC) : m_pRTC(pRTC),
							 m_oldRightWheelAngle(0.0),
							 m_oldLeftWheelAngle(0.0) {}
  virtual ~WheelAngleListener() {}

  virtual void operator()(const ConnectorInfo& info,
                          const RTC::TimedDoubleSeq& data) {
    double axleTrack = m_pRTC->getAxleTrack();
    double wheelRadius = m_pRTC->getWheelRadius();
    double rightWheelAngle = data.data[0];
    double leftWheelAngle = data.data[1];
    static const double PI = 3.14159265358979;
    double x = m_pRTC->getX();
    double y = m_pRTC->getY();
    double theta = m_pRTC->getTheta();

    double deltaR = rightWheelAngle - m_oldRightWheelAngle;
    if (deltaR > PI) { deltaR -= 2*PI; }
    else if (deltaR < -PI) { deltaR += 2*PI; }
    double deltaL = leftWheelAngle - m_oldLeftWheelAngle;
    if (deltaL > PI) { deltaL -= 2*PI; }
    else if (deltaL < -PI) { deltaL += 2*PI; }

    double deltaTrans = (deltaR+deltaL) * wheelRadius / 2;
    double deltaTheta = (deltaR-deltaL) * wheelRadius / axleTrack;

    x = x + deltaTrans * cos(theta + deltaTheta/2);
    y = y + deltaTrans * sin(theta + deltaTheta/2);
    theta = theta + deltaTheta;
    
    m_pRTC->setX(x);
    m_pRTC->setY(y);
    m_pRTC->setTheta(theta);

    m_oldRightWheelAngle = rightWheelAngle;
    m_oldLeftWheelAngle  = leftWheelAngle;

    m_pRTC->writeCurrentPose(data.tm);
  };
private:
  double m_oldRightWheelAngle;
  double m_oldLeftWheelAngle;
};

class TargetVelocityListener
  : public ConnectorDataListenerT<RTC::TimedVelocity2D>
{
private:
  MobileRobotKinematics *m_pRTC;
public:
  TargetVelocityListener(MobileRobotKinematics* pRTC) : m_pRTC(pRTC) {}
  virtual ~TargetVelocityListener() {}

  virtual void operator()(const ConnectorInfo& info,
                          const RTC::TimedVelocity2D& data) {
	double axleTrack = m_pRTC->getAxleTrack();
    double wheelRadius = m_pRTC->getWheelRadius();

	double v_buf = data.data.va * axleTrack/2;
	double vr = data.data.vx + v_buf;
	double vl = data.data.vx - v_buf;

	double omegaR = vr / wheelRadius;
	double omegaL = vl / wheelRadius;

	m_pRTC->writeWheelVelocity(data.tm, omegaR, omegaL);
  };
};

class UpdatePoseListener
  : public ConnectorDataListenerT<RTC::TimedPose2D>
{
private:
  MobileRobotKinematics *m_pRTC;
public:
  UpdatePoseListener(MobileRobotKinematics* pRTC) : m_pRTC(pRTC) {}
  virtual ~UpdatePoseListener() {}

  virtual void operator()(const ConnectorInfo& info,
                          const RTC::TimedPose2D& data) {
							  m_pRTC->setX(data.data.position.x);
							  m_pRTC->setY(data.data.position.y);
							  m_pRTC->setTheta(data.data.heading);

  };
};


extern "C"
{
  DLL_EXPORT void MobileRobotKinematicsInit(RTC::Manager* manager);
};

#endif // MOBILEROBOTKINEMATICS_H

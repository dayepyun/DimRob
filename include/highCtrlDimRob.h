/** @file highCtrlDimRob.h
 *  @brief Declaration of highCtrlDimRob class implementation
 *
 *	Subscribe joystick messages, and publish desired actuator values to the low-level controller.\n
 *	Shutdown the subscriber of joystick messages if GUI test mode is enabled.
 *
 *  @author R. Pyun
 *
 */

#ifndef HIGHCTRLDIMROB_H_
#define HIGHCTRLDIMROB_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include "math.h"

#include "xeno_interface/Dimrob.h"
#include "xeno_interface/JoyMode.h"
#include "sensor_msgs/Joy.h"


/********************************************//**
 * @brief The enumeration of axes of joystick
 ***********************************************/
enum AXES
{
	AXES_JOY1_H =0,
	AXES_JOY1_V,
	AXES_LT,
	AXES_JOY2_H,
	AXES_JOY2_V,
	AXES_RT,
	AXES_DPAD_H,
	AXES_DPAD_V
};

/********************************************//**
 * @brief The enumeration of buttons of joystick
 ***********************************************/
enum BUTTONS
{
	BUTTONS_A =0,
	BUTTONS_B,
	BUTTONS_X,
	BUTTONS_Y,
	BUTTONS_LB,
	BUTTONS_RB,
	BUTTONS_BACK,
	BUTTONS_START,
	BUTTONS_LOGITECH,
	BUTTONS_JOY1,
	BUTTONS_JOY2
};

/********************************************//**
 * @brief Class for High-level controller
 ***********************************************/
class HighCtrl
{
public:
	HighCtrl();
	~HighCtrl();

private:
  void resetValveMotor();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sensorCallback(const xeno_interface::Dimrob& msg);
  void guiCallback(const xeno_interface::JoyMode& msg);


  ros::NodeHandle n;

  ros::Publisher publisherHC;
  ros::Subscriber subscriberHC;
  ros::Subscriber subscriberJoyHC;
  ros::Subscriber subscriberGUIHC;

  bool coolerJoy, lightJoy, swEstopJoy, speedJoy;
  double r, theta, motorJoy, leftValveJoy, rightValveJoy;

};

#endif /* HIGHCTRLDIMROB_H_ */

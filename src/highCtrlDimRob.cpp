/** @file highCtrlDimRob.cpp
 *  @brief ROS program for High-level controller for DimRob
 *
 *  The sensor readings from LC are subscribed.\n
 *  A part of subscribed message (e.g. i_val_pres_oil) is stored into a logfile (src/xeno_interface/logfile_HC.txt).\n
 *
 *  Gamepad(joy_node) message is subscribed, and when Gamepad is opereated, some of desired actuator signals are published to LC. \n
 *
 *  1.	Left Joystick (only if RB button is pressed):
 *
 *  		-radius (r): 	proportional to desired motor command (Maximum motor command is defined in MAX_MOTOR_JOY)\n
 *  		-angle (theta): define the direction of Moog valve for left and right tracks\n
 *  		-when the joystick is pushed (click), the desired motor command and the desired Moog valve position are set to 0.\n
 *
 *  2.	Button B & D-PAD(Right):
 *
 *    		-Button B: 		Enable the SW E-stop
 *    		-D-PAD(Right):	Disable the SW E-stop
 *
 *  3.	Button X & D-PAD(Left):
 *
 *    		-Button X: 		Enable the cooler
 *    		-D-PAD(Left):	Disable the cooler
 *
 *  4.	Button Y & D-PAD(Up):
 *
 *    		-Button Y: 		Enable the signal light
 *    		-D-PAD(Up):		Disable the signal light
 *
 *  5.	Button A & D-PAD(Down):
 *
 *    		-Button A: 		Enable the speed valve
 *    		-D-PAD(Down):	Disable the speed valve
 *
 *  When joystick-mode is disabled from GUI, subscriberJoyHC is shut down.\n
 *
 *  @author R. Pyun
 */

#include "highCtrlDimRob.h"

const char * logfile = "src/xeno_interface/logfile_HC.txt"; ///<logfile path
FILE * fp = fopen(logfile,"w"); ///<open a file object

/********************************************************/
#define MAX_MOTOR_JOY 200 		///< maximum motor command with joystick
/********************************************************/

/********************************************//**
 *  @brief Constructor of HighCtrl
 ***********************************************/
HighCtrl::HighCtrl()
{
  publisherHC = n.advertise<xeno_interface::Dimrob>("actuatorDimRob", 1);
  subscriberHC = n.subscribe("sensorDimRob", 1, &HighCtrl::sensorCallback, this);
  subscriberJoyHC = n.subscribe("joy", 1, &HighCtrl::joyCallback, this);
  subscriberGUIHC = n.subscribe("GuiToJoy", 1, &HighCtrl::guiCallback, this);

  motorJoy = 0;
  r = 0;
  theta = 0;
  leftValveJoy = 0;
  rightValveJoy = 0;

  coolerJoy = false;
  lightJoy = false;
  swEstopJoy = false;
  speedJoy = false;

}
HighCtrl::~HighCtrl()
{
	publisherHC.shutdown();
	subscriberHC.shutdown();
	subscriberJoyHC.shutdown();
	subscriberGUIHC.shutdown();
}

/********************************************//**
 *  @brief Set Moog valve and motor commands to zero
 ***********************************************/
void HighCtrl::resetValveMotor()
{
	leftValveJoy = 0;
	rightValveJoy = 0;
	motorJoy =0;
}
/********************************************//**
 *  @brief The sensor messages are subscribed from LC.
 *
 *
 *  @param[in] msg	message published from lowCtrolDimRob.cpp
 *  @return void
 ***********************************************/
void HighCtrl::sensorCallback(const xeno_interface::Dimrob& msg)
{
//
//	ROS_INFO("%f ", msg.i_val_temp_scb);
//	ROS_INFO("%f ", msg.i_val_temp_mot);
//	ROS_INFO("%f ", msg.i_val_volt_bat);
//	ROS_INFO("%f ", msg.i_val_pres_oil_1);
//	ROS_INFO("%f ", msg.i_val_pres_oil_4);
//	ROS_INFO("%f ", msg.i_val_pres_oil_5);
//	ROS_INFO("%f ", msg.i_val_pres_oil);
//	ROS_INFO("%f ", msg.i_val_flow_oil);
//	ROS_INFO("%f ", msg.i_val_temp_oil);
//	ROS_INFO("%f ", msg.i_val_pres_oil_3);
//	ROS_INFO("%f ", msg.i_val_pres_oil_2);
//	ROS_INFO("%d ", msg.i_encoderCount);
//	ROS_INFO("%f ", msg.s_motorRPMCurrent);
//	ROS_INFO("%d ", msg.s_motorCmdCurrent);
//
//	ROS_INFO("%d ", msg.i_chargerEnabled);
//	ROS_INFO("%d ", msg.i_chargerPowered);
//	ROS_INFO("%d ", msg.i_hwEstopOpened);
//	ROS_INFO("%d ", msg.i_manualModeEnabled);
//	ROS_INFO("%d ", msg.i_autoModeEnabled);
//	ROS_INFO("%d ", msg.s_nowState);
//	ROS_INFO("%d ", msg.s_errCondDetected);
//
//	ROS_INFO("%f ", msg.o_motorRPM);
//	ROS_INFO("sw estop %d ", msg.o_swEstopEnabled);


	fprintf (fp, "%f \n", msg.i_val_pres_oil);

}

/********************************************//**
 *  @brief The subscribed message from Joy node for joystick inputs.
 *
 *  1.	Left Joystick:
 *
 *  		-radius (r): 	proportional to desired motor command (Maximum motor command is defined in MAX_MOTOR_JOY)\n
 *  		-angle (theta): define the direction of Moog valve for left and right tracks\n
 *  		-when the joystick is pushed (click), the desired motor command and the desired Moog valve position are set to 0.\n
 *
 *  2.	Button B & D-PAD(Right):
 *
 *    		-Button B: 		Enable the SW E-stop
 *    		-D-PAD(Right):	Disable the SW E-stop
 *
 *  3.	Button X & D-PAD(Left):
 *
 *    		-Button X: 		Enable the cooler
 *    		-D-PAD(Left):	Disable the cooler
 *
 *  4.	Button Y & D-PAD(Up):
 *
 *    		-Button Y: 		Enable the signal light
 *    		-D-PAD(Up):		Disable the signal light
 *
 *  5.	Button A & D-PAD(Down):
 *
 *    		-Button A: 		Enable the speed valve
 *    		-D-PAD(Down):	Disable the speed valve
 *
 *
 *  Definition of "sensor_msgs::Joy"\n
 *
 *	# Reports the state of a joysticks axes and buttons.\n
 *	Header header           # timestamp in the header is the time the data is received from the joystick\n
 *	float32[] axes          # the axes measurements from a joystick\n
 *	int32[] buttons         # the buttons measurements from a joystick\n
 *
 *
 *  @param[in] msg	message published from Joy package.
 *  @return void
 ***********************************************/
void HighCtrl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	xeno_interface::Dimrob msgHC;

	//if RB button is pressed, allow joystick to manipulate Moog valves and motor command
	if(joy->buttons[BUTTONS_RB] == 1)
	{
		//rosrun joy joy_node _deadzone:=0.9 (refer to http://docs.ros.org/api/joy/html/)
		if (joy->buttons[BUTTONS_JOY1] == 1) //if joystick is pressed -> set flow valves and motor command to 0
		{
			resetValveMotor();
		}
		else if(((joy->axes[AXES_JOY1_V]) == 0) && ((joy->axes[AXES_JOY1_H]) == 0)) //joystick in deadzone
		{
			resetValveMotor();
		}
		else
		{

			//track drive conversion for circular boundary joystick
			r = pow((pow(joy->axes[AXES_JOY1_V],2)+pow(joy->axes[AXES_JOY1_H],2)),0.5);
			theta = atan2((joy->axes[AXES_JOY1_V]),(joy->axes[AXES_JOY1_H]));

			//joystick is not perfectly calibrated -> limit the maximum and minimum value of radius for joystick to 1 or -1
			if(r > 1 )
				r = 1;
			else if(r < -1)
				r = -1;

			if(theta >0 && theta <= M_PI/2)
			{
				leftValveJoy = sin(2*theta-(M_PI/2));
				rightValveJoy= 1;
			}
			else if (theta > M_PI/2 && theta <= M_PI)
			{
				leftValveJoy = 1;
				rightValveJoy= sin(2*theta-(M_PI/2));
			}
			else if (theta >= -M_PI && theta <= -M_PI/2)
			{
				leftValveJoy = sin(2*theta+(M_PI/2));
				rightValveJoy= -1;
			}
			else if (theta > -M_PI/2 && theta <= 0)
			{
				leftValveJoy = -1;
				rightValveJoy= sin(2*theta+(M_PI/2));
			}

			//motor command is proportional to the radius of joystick command
			motorJoy = fabs(r)*MAX_MOTOR_JOY;

		}
	}
	else
	{
		resetValveMotor();
	}


	//buttons A,B,X,Y & DPAD (use it as a toggle buttons)
	if(joy->buttons[BUTTONS_B] == 1)
		swEstopJoy = true;
	else if(joy->axes[AXES_DPAD_H] < 0)
		swEstopJoy = false;

	if(joy->buttons[BUTTONS_X] == 1)
		coolerJoy = true;
	else if(joy->axes[AXES_DPAD_H] > 0)
		coolerJoy = false;

	if(joy->buttons[BUTTONS_Y] == 1)
		lightJoy = true;
	else if(joy->axes[AXES_DPAD_V] > 0)
		lightJoy = false;

	if (joy->buttons[BUTTONS_A] == 1)
		speedJoy = true;
	else if(joy->axes[AXES_DPAD_V] < 0)
		speedJoy = false;


	msgHC.o_swEstopEnabled = swEstopJoy;
	msgHC.o_coolerEnabled = coolerJoy;
	msgHC.o_lightEnabled = lightJoy;
	msgHC.o_highSpeedValEnabled = speedJoy;

	msgHC.o_flowValveLeft = leftValveJoy*10; //in V (+-10V)
	msgHC.o_flowValveRight = rightValveJoy*10; //in V (+-10V)

	msgHC.o_motorCmd = motorJoy; //or msgHC.o_motorRPM

	ROS_INFO("motorCmd: %g , ............Left: %f , Right: %f", msgHC.o_motorCmd, leftValveJoy*10, rightValveJoy*10);

	publisherHC.publish(msgHC);
}


/********************************************//**
 *  @brief The subscribed message on enable/disable joystick-mode in GUI.
 *
 *  When joystick-mode is disabled, subscriberJoyHC shuts down.
 *  Otherwise, subscriberJoyHC is re-start "joyCallback".
 *
 *  @param[in] msg	message published from GUI package
 *  @return void
 ***********************************************/
void HighCtrl::guiCallback(const xeno_interface::JoyMode& msg)
{

	if(msg.guiTestModeEnabled == true)
	{
		subscriberJoyHC.shutdown();
	}
	else
	{
		subscriberJoyHC = n.subscribe("joy", 1000, &HighCtrl::joyCallback, this);
	}

}
/********************************************//**
 *  @brief Main function for HC.
 *
 *  @return 0
 ***********************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "highCtrlDimRob");
  HighCtrl highCtrl;

  ros::spin();

  return 0;
}

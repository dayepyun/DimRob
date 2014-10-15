/** @file lowCtrlDimRob.cpp
 *  @brief DimRob LC application code
 *
 *  !!make sure to have the following path, ini_name[] = "src/xeno_interface/src/config/conf.ini", in configure_board() function.\n
 *	!!also "src/xeno_interface/src/config/pci6229conf.ini" as a filename inside the conf.ini file.\n
 *
 *  This is the application code for the low-level control of hydraulic system for DimRob.\n
 *	Xenomai and ROS publisher are integrated in one file.\n
 *
 *
 *	Two real-time tasks with different priorities are initiated.\n
 *	The task with high priority is a serving as a real-time task for our system.\n
 *	The non-real task is implemented with a real-time task with low priority in order to use mutex function in native API.\n
 *	We simply ignore mode switches in the real-time task with low priority, a non-real time task for our system.\n
 *
 *	Mutex system is implemented with time flag to prevent accessing the global variables by two tasks at the same time.\n
 *  If the real-time thread with high priority cannot acquire the mutex in 1/10 of its frequency, the timeout message is printed.\n
 *
 *
 *  1. task_rt (high priority):\n
 *  -Read sensor inputs
 *  -Set to corresponding state machine status.\n
 *  -Send actuator outputs\n
 *  -Desired motor command is passed to non-real time loop.\n
 *
 *	2. task_nrt (low priority):\n
 *	-Acquire the sensors inputs and the actuators outputs from RT thread and publish the data\n
 *	-Send the desired motor command to the motor controller\n
 *
 *
 *  Four state machine status
 *
 *  1.	Idle:
 *
 *  		-Initial loop when computer starts up
 *  		-Wait for toggle switch input for Manual or Auto mode -> Manual mode loop or Auto mode loop
 *  		-Check error condition and E-stop status -> Error State
 *  			1. Mode Valve:	  	Center
 *  			2. Speed Valve:		Off
 *  			3. MOOGs:			Center
 *  			4. Motor Command:	Off
 *				5. Other outputs:	Off - Cooler and Signal light
 *
 *  2.	Error:\n
 *
 *  		-Enter the loop when error is detected OR E-stop is pressed
 *  		-Exit the loop when error is removed AND E-stop is not pressed
 *  			1. Mode Valve:	  	Center
 *  			2. Speed Valve:		Off
 *  			3. MOOGs:			Center
 *  			4. Motor Command:	0
 *				5. Other outputs:	Off - Cooler and Signal light
 *
 *  3.	Manual mode:\n
 *
 *			-Enter the loop from Idle loop when toggle switch is set to Manual mode
 *			-Exit to Idle loop if toggle switch status is not Manual mode
 *			-Exit to Error loop in case of error is detected or E-stop is pressed
 *   			1. Mode Valve:	  	Passive (Center command)
 *  			2. Speed Valve:		Off
 *  			3. MOOGs:			Center
 *  			4. Motor Command:	MicroScript - constant motor command (disable the motor command from serial communication)
 *
 *  4.	Auto mode:\n
 *
 *			-Enter the loop from Idle loop when toggle switch is set to Auto mode
 *			-Exit to Idle loop if toggle switch status is not RC mode
 *			-Exit to Error loop in case of error is detected or E-stop is pressed
 *   			1. Mode Valve:	  	Left
 *  			2. Speed Valve:		Modulated vs. Speed command
 *  			3. MOOGs:			Selectable
 *  			4. Motor Command:	Serial communication - Modulated vs. Supply Pressure
 *
 *  5.	Test mode:\n
 *
 *			- Enter the loop from Idle loop when toggle switch is set to Auto mode and Test mode is enabled
 *			- Exit to Idle loop if toggle switch status is not Auto mode OR Test mode is disabled
 *			- Exit to Error loop in case of error is detected or E-stop is pressed
 *			   	1. Mode Valve:	  	Left
 *  			2. Speed Valve:		selectable
 *  			3. MOOGs:			Selectable
 *  			4. Motor Command:	Open-loop control
 *
 *
 *  @todo  speed valve control algorithm in auto mode
 *  @todo  motor RPM algorithm (e.g. PID control) for auto mode
 *  @todo  real-time ROS tool (RT ROS publisher)
 *
 *	@date	18.06.2014
 *  @author R. Pyun
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <memory>

#include "iniparser.h"
#include "controlUtils.h"

#include <signal.h>
#include <fcntl.h>
#include <termios.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

/*Real-time related headers*/
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <sys/mman.h>
#include <rtdk.h> //Required for rt_printf (Real-time console print-out)
#include <execinfo.h> //for backtrace

#include "stateMachine.h"
#include "xeno_interface/Dimrob.h"
#include "realtime_subscriber/RealtimeSubscriber.hpp"

/********************************************************/
//MODIFY TO TEST IN DESIRED FREQUENCY
#define FREQ_HZ_RT  100 	///< the real-time task frequency in HzRQ_ERR_OPEN_PORT
#define FREQ_HZ_NRT 10 		///< the non real-time task frequency in Hz
/********************************************************/
#define IDLE_TIME_MS 3000	///< fixed minimum duration for idle state in ms
/********************************************************/
#define MAX_MOTOR 250 		///< maximum motor command
#define MOTOR_CMD_RATE	500 ///< motor command increment or decrement rate (MOTOR_CMD_RATE per second)
/********************************************************/
#define MAX_DAQ_BOARD 1 	///< maximum number of DAQ board (use 1 DAQ board for the current system)

RT_TASK demo_task_rt;		///<declaration of a real-time task (for high priority task)
RT_TASK demo_task_nrt;		///<declaration of a real-time task (for low priority task)
RT_MUTEX mutexPub;			///<declaration of a mutex for ROS publisher

pciBase     * pciboards[MAX_DAQ_BOARD]; ///< pciBase class initialized
int pci_board_num = 0; 					///< number of DAQ board, the value is changed during the configuration

const int oneSec_ns = 1000000000; 	///< 1 second in nanosecond
const int oneSec_ms = 1000000; 		///< 1 second in millisecond

static volatile int modeSwitchesCount = 0; 		///< mode switch counter: to count the switch from primary to secondary mode in a real-time thread
const char * backtrace_file = "backtrace.txt";	///< the switch from primary to secondary mode in a real-time thread are logged into the file
FILE * backtrace_fd; 							///< a file descriptor for backtrace_file


const char * logfile = "log_LC.txt"; ///<logfile path
FILE * fp = fopen(logfile,"w"); ///<open a file object

/********************************************************/
//GLOBAL VARIABLES
RoboteqDevice device;				///<New instance of the Roboteq-device class.
unsigned timeoutPub =0;				///<timeout counter: incremented when mutex timeout occurs
/********************************************************/
int gp_motorErr = 0;		///<Return error value for motor controller related functions (0 if successful)  [motorErrNumber, motorErr]
int gp_motorCurrErr = 0;
int gp_motorBatCurrErr = 0;
int gp_motorVoltErr = 0;
int gp_motorBatVoltErr = 0;
double gp_motorCurr = 0;		///<
double gp_motorBatCurr = 0;		///<
double gp_motorVolt = 0;		///<
double gp_motorBatVolt = 0;		///<
int gp_motorCmdNow = 0;
int gp_motorFaultFlagErr = 0;
int gp_motorCmdActualErr = 0;

int motorControllerStatus = RQ_ERR_NOT_CONNECTED;					///<motor controller initialization status
xeno_interface::Dimrob buffer;	///<Gloabla variable for [local variable in RT thread] <> [local variable in non-RT thread (or ROS msg in Dimrob.msg)]
/********************************************************/



/*******************************
 *  @brief the real-time task
 *
 *  1. task_rt (high priority):\n
 *  -Read sensor inputs
 *  -Set to corresponding state machine status.\n
 *  -Send actuator outputs
 *  -Desired motor command and other variables for ROS publisher are passed to non-real time loop.\n
 *
 *  @param[in] arg
 *  @return void
 ***********************************************/
void task_rt(void *arg)
{

	long time_stamp_ms, time_stamp_us, idleTimeOut_ms = 0;
	double errorRPM = 0, dtRPM = 0, val_control = 0;
	int timeoutUs = 500, mutexErr = 0;

	RTIME now,previous;

	stateMachine FSM;
	PID pidRPM;

	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle()); //two node handle in 1 program???
	RealtimeSubscriber<xeno_interface::Dimrob> subscriberLC;
	subscriberLC.subscribe(nh, "actuatorDimRob", 1, 1);


	rt_task_set_mode(0, T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, oneSec_ns/FREQ_HZ_RT);

	//set all outputs and motor command to 0 and start in Idle mode
	FSM.daqOutErr = FSM.InitFSM(pciboards[0]);

	FSM.freqLoop = FREQ_HZ_RT;
	dtRPM = oneSec_ns/FREQ_HZ_RT;
	pidRPM.pid_initialize(0.8,0,50,2); //todo tune PID parameter

	rt_fprintf (fp, "%s	%s	%s	%s	%s	%s	%s	%s\n", "TimeStamp (ms)", "motorCmdSent","motorRPM (RPM)","motorAmp (A)","battAmp (A)","internalVolt (V)","battVolt (V)","supplyPress (bar)");

	previous = rt_timer_read();//the timer start here.

	while (1) {

		rt_task_wait_period(NULL);

		now = rt_timer_read();
		time_stamp_ms = (now-previous)/oneSec_ms;
		time_stamp_us = (now-previous)%oneSec_ms;

		FSM.daqErr = FSM.ReadSensors(pciboards[0]);
		if(FSM.daqErr < 0)
		{
			rt_printf("read sensor value failed (err=%d)\n",FSM.daqErr);
		}
//		else
//			rt_printf("Time: %ld.%06ld ms\n"
//							"State: %d\n"
//							"Battery: %g V\n"
//							"Power electronics box temp: %g °C\n"
//							"Motor Temperature in V: %g V\n"
//							"Flow sensor: %g l/min\n"
//							"Pressure sensor: %g bar\n"
//							"Temperature sensor: %g °C\n"
//							"Pressure 1 (R, A port): %g bar\n"
//							"Pressure 2 (R, B port): %g bar\n"
//							"Pressure 4 (L, A port): %g bar\n"
//							"Pressure 5 (L, B port): %g bar\n"
//							"Pressure 3 (Supply): %g bar\n"
//							"Mode Valve: %s\n"
//							"Speed Valve: %s\n"
//							"Charger enabled: %s\n"
//							"CHarger powered: %s\n"
//							"HW estop pressed: %s\n"
//							"SW estop pressed: %s\n"
//							"Cooler: %s\n"
//							"Signal light: %s\n"
//							"Auto mode: %s \n"
//							"Manual mode: %s \n"
//							"Flow valve Left: %g V\n"
//							"Flow valve Left: %g V\n"
//							"MotorCommand(serial): %g\n"
//							"Counter %d.................%d......%g rpm\n"
//							,  time_stamp_ms, time_stamp_us, FSM.nowState,
//							FSM.val_volt_bat, FSM.val_temp_scb,
//							FSM.val_temp_mot,FSM.val_flow_oil,
//							FSM.val_pres_oil,FSM.val_temp_oil,
//							FSM.val_pres_oil_1,
//							FSM.val_pres_oil_2,
//							FSM.val_pres_oil_4,
//							FSM.val_pres_oil_5,
//							FSM.val_pres_oil_3,
//							FSM.autoModeValEnabled == 1 ? "YES" : "NO",
//							FSM.highSpeedValEnabled == 1 ? "YES" : "NO",
//							FSM.chargerEnabled == 1 ? "YES" : "NO",
//							FSM.chargerPowered == 1 ? "YES" : "NO",
//							FSM.hwEstopOpened == 0 ? "YES" : "NO",
//							FSM.swEstopEnabled == 1 ? "YES" : "NO",
//							FSM.coolerEnabled == 1 ? "YES" : "NO",
//							FSM.lightEnabled == 1 ? "YES" : "NO",
//							FSM.autoModeEnabled == 1 ? "YES" : "NO",
//							FSM.manualModeEnabled == 1 ? "YES" : "NO",
//							FSM.flowValveLeft, FSM.flowValveRight,FSM.motorCmd,
//							SUBD_CNT-11, FSM.encoderCount, FSM.motorRPMnow);

		xeno_interface::Dimrob dimrobMsg;
		int ret = subscriberLC.retrieveMessage(dimrobMsg,timeoutUs);

		switch (ret)
		{
			case RealtimeSubscriber<xeno_interface::Dimrob>::SUCCESS:
			{
//				std::cout<<"Successfully read message"<<std::endl;
				FSM.swEstopEnabled = dimrobMsg.o_swEstopEnabled;
				FSM.guiTestModeEnabled = dimrobMsg.o_guiTestModeEnabled;

				FSM.motorCmd = dimrobMsg.o_motorCmd; //desired motor command
				FSM.desiredSupplyPres = dimrobMsg.s_desiredSupplyPres; //desired supply pressure

				FSM.flowValveLeft = dimrobMsg.o_flowValveLeft;
				FSM.flowValveRight = dimrobMsg.o_flowValveRight;

				FSM.coolerEnabled = dimrobMsg.o_coolerEnabled;
				FSM.lightEnabled = dimrobMsg.o_lightEnabled;

				FSM.highSpeedValEnabled = dimrobMsg.o_highSpeedValEnabled;

				FSM.autoModeValEnabled = dimrobMsg.o_autoModeValEnabled;

				FSM.openLoopModeEnabled = dimrobMsg.o_openLoopModeEnabled;
			}
			break;
			case RealtimeSubscriber<xeno_interface::Dimrob>::QUEUE_BUSY:
//				rt_printf("Retrieving data timed out (queue is busy).\n");
			break;
			case RealtimeSubscriber<xeno_interface::Dimrob>::BUFFER_EMPTY:
//				rt_printf("No data to retrieve.\n");
			break;
		}

		FSM.daqOutErr = pciboards[0]->WriteDO(SUBD_DIO,DO_ESTOP_SW, FSM.swEstopEnabled);

		FSM.CheckErrCondition();

//rt_printf("state: %d, idleTimeOut_ms %ld and FSM.idleSetTime %d\n", FSM.nowState, idleTimeOut_ms, FSM.idleSetTime);

		switch(FSM.GetNowState())
		{
		case STATE_IDLE :
			{

				if((FSM.hwEstopOpened == false) || (FSM.swEstopEnabled == true) || (FSM.errCondDetected == true))
				{
					FSM.daqOutErr = FSM.SetErrorState(pciboards[0]);
				}
				else
				{

					if(FSM.manualModeEnabled | FSM.autoModeEnabled)
					{
						idleTimeOut_ms = (now-FSM.idleSetTime)/oneSec_ms;
					}
					else
					{
						idleTimeOut_ms = 0;
					}


					if((FSM.manualModeEnabled == true) && (idleTimeOut_ms > IDLE_TIME_MS))
					{


//						//todo need to change to accept if motor controller is connected and turned off again
//						if(motorControllerStatus != RQ_SUCCESS) //if the motor controller driver is not initialized
//						{
//
//							Set_Up_Driver();
//
//						}


						if(FSM.guiTestModeEnabled == false)
						{
							FSM.daqOutErr = FSM.SetManualState(pciboards[0]);
						}
						else
						{
							FSM.daqOutErr = FSM.SetTestState(pciboards[0]);
						}
					}
					else if((FSM.autoModeEnabled == true) && (idleTimeOut_ms > IDLE_TIME_MS))
					{

//						//todo need to change to accept if motor controller is connected and turned off again
//						if(motorControllerStatus != RQ_SUCCESS) //if the motor controller driver is not initialized
//						{
//							Set_Up_Driver();
//						}


						if(FSM.guiTestModeEnabled == false)
						{
							FSM.daqOutErr = FSM.SetAutoState(pciboards[0]);
						}
						else
						{
							FSM.daqOutErr = FSM.SetTestState(pciboards[0]);
						}

					}
					else
					{
						//Idle State: the desired motor command is 0 and no other command for actuator is required
						//rt_printf("%ld ms\n", idleTimeOut_ms);

						FSM.motorCmd = 0;

						FSM.autoModeValEnabled = false;
						FSM.highSpeedValEnabled = false;
						FSM.flowValveLeft = 0;
						FSM.flowValveRight = 0;
						FSM.coolerEnabled = false;
						FSM.lightEnabled = false;

					}

				}
			}
			break;

		case STATE_ERROR :
			{
				if( (FSM.hwEstopOpened == true) &&  (FSM.swEstopEnabled == false)  && (FSM.errCondDetected == false))
				{
					FSM.daqOutErr = FSM.SetIdleState(pciboards[0]);
				}
				else
				{
					//Error State: the desired motor command is 0 and no other command for actuator is required
					FSM.motorCmd = 0;

					//todo disable other actuators as well like in setErrorState ?!
					FSM.autoModeValEnabled = false;
					FSM.highSpeedValEnabled = false;
					FSM.flowValveLeft = 0;
					FSM.flowValveRight = 0;
					FSM.coolerEnabled = false;
					FSM.lightEnabled = false;


					rt_printf("HW estop pressed: %s\n"
							"SW estop pressed: %s\n"
							"Error condition detected: %s\n"
							"DAQ read error %d\n"
							"DAQ write error %d\n",
							FSM.hwEstopOpened == 0 ? "YES" : "NO",
							FSM.swEstopEnabled == 1 ? "YES" : "NO",
							FSM.errCondDetected == 1 ? "YES" : "NO",
							FSM.daqErr,
							FSM.daqOutErr);
				}
			}
			break;
		case STATE_MANUAL :
			{
				if( (FSM.hwEstopOpened == false) ||  (FSM.swEstopEnabled == true) || (FSM.errCondDetected == true))
				{
					FSM.daqOutErr = FSM.SetErrorState(pciboards[0]);
				}
				else if(FSM.manualModeEnabled == false)
				{
					FSM.daqOutErr = FSM.SetIdleState(pciboards[0]);
				}
				else if(FSM.guiTestModeEnabled == true)
				{
					FSM.daqOutErr = FSM.SetTestState(pciboards[0]);
				}
				else
				{
					//Manual state: the desired motor command is 0
					//send motor command via mScript: hard-wired to motor controller

					FSM.daqOutErr = pciboards[0]->WriteDO(SUBD_DIO,DO_COOLER, FSM.coolerEnabled);
					FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_LIGHT, FSM.lightEnabled);

					FSM.motorCmd = 0;

					FSM.autoModeValEnabled = false;
					FSM.highSpeedValEnabled = false;
					FSM.flowValveLeft = 0;
					FSM.flowValveRight = 0;

				}
			}
			break;
		case STATE_AUTO:
			{
				if( (FSM.hwEstopOpened == false) ||  (FSM.swEstopEnabled == true)  || (FSM.errCondDetected == true))
				{
					FSM.daqOutErr = FSM.SetErrorState(pciboards[0]);
				}
				else if(FSM.autoModeEnabled == false)
				{
					FSM.daqOutErr = FSM.SetIdleState(pciboards[0]);
				}
				else if(FSM.guiTestModeEnabled == true)
				{
					FSM.daqOutErr = FSM.SetTestState(pciboards[0]);
				}
				else
				{

					//Auto state: Closed-loop control
					FSM.daqOutErr =  pciboards[0]->WriteDA(AO_FLOW_VAL_L,FSM.flowValveLeft);
					FSM.daqOutErr =  pciboards[0]->WriteDA(AO_FLOW_VAL_R,FSM.flowValveRight);

					FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_COOLER, FSM.coolerEnabled);
					FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_LIGHT, FSM.lightEnabled);

					FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_SPEED_VAL, FSM.highSpeedValEnabled);

					//FSM.motorCmd defined using joystick D-pad ... for temporary solution

						/*
						//PID control - desired motor RPM
						errorRPM = FSM.motorRPM - FSM.motorRPMCurrent; //desiredMotorRPM - currentMotorRPM
						pidRPM.pid_update(errorRPM, dtRPM, val_control);

						FSM.motorCmd = val_control; //todo test and see if pidRPM.pid_update(double, double, int) is possible instead of double, double, double

						rt_printf("desired RPM: %g and RPM %g and desired motor command in [int %d, double %g] \n", FSM.motorRPM, FSM.motorRPMCurrent, FSM.motorCmd, val_control);

						*/
							//joystick message for motor command is already copied to FSM.motorCmd above
						FSM.autoModeValEnabled = true;

				}

			}
			break;
		case STATE_TEST:
				{

					if( (FSM.hwEstopOpened == false) ||  (FSM.swEstopEnabled == true)  || (FSM.errCondDetected == true))
					{
						FSM.daqOutErr = FSM.SetErrorState(pciboards[0]);
					}
					else if((FSM.autoModeEnabled == false) && (FSM.manualModeEnabled == false))
					{
						FSM.daqOutErr = FSM.SetIdleState(pciboards[0]);
					}
					else if(FSM.guiTestModeEnabled == false)
					{
						FSM.daqOutErr = FSM.SetIdleState(pciboards[0]);
					}
					else
					{
						//Auto state: Closed-loop control vs. open-loop control(i.e. test mode)
						FSM.daqOutErr =  pciboards[0]->WriteDA(AO_FLOW_VAL_L,FSM.flowValveLeft);
						FSM.daqOutErr =  pciboards[0]->WriteDA(AO_FLOW_VAL_R,FSM.flowValveRight);

						FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_COOLER, FSM.coolerEnabled);
						FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_LIGHT, FSM.lightEnabled);

						FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_SPEED_VAL, FSM.highSpeedValEnabled);

						if(FSM.openLoopModeEnabled == true)
						{

							//open-loop control of motor command
							//from GUI or HC: desired motor command stored into FSM.motorCmd variable

							FSM.daqOutErr =  pciboards[0]->WriteDO(SUBD_DIO,DO_MODE_VAL, FSM.autoModeValEnabled);
							//no change in FSM.motorCmd from gs_motorCmd

						}


					}

				}
				break;
		}


		mutexErr = rt_mutex_acquire(&mutexPub,(oneSec_ns/FREQ_HZ_RT)/10);
		if(mutexErr != 0)
		{
			timeoutPub++;
			rt_printf("mutex err : %d and counter %d\n",mutexErr, timeoutPub);
		}
		else
		{

			//store the desired motor command, and other variables to the global variables
			buffer.s_nowState = FSM.nowState;
			buffer.s_errCondDetected = FSM.errCondDetected;

			buffer.o_motorCmd = FSM.motorCmd;
			buffer.s_motorRPMnow = FSM.motorRPMnow;

			buffer.i_val_temp_scb = FSM.val_temp_scb;
			buffer.i_val_temp_mot = FSM.val_temp_mot;
			buffer.i_val_volt_bat = FSM.val_volt_bat;
			buffer.i_val_pres_oil_1 = FSM.val_pres_oil_1;
			buffer.i_val_pres_oil_4 = FSM.val_pres_oil_4;
			buffer.i_val_pres_oil_5 = FSM.val_pres_oil_5;
			buffer.i_val_pres_oil = FSM.val_pres_oil;
			buffer.i_val_flow_oil = FSM.val_flow_oil;
			buffer.i_val_temp_oil = FSM.val_temp_oil;
			buffer.i_val_pres_oil_3 = FSM.val_pres_oil_3;
			buffer.i_val_pres_oil_2 = FSM.val_pres_oil_2;

			buffer.i_chargerEnabled = FSM.chargerEnabled;
			buffer.i_chargerPowered = FSM.chargerPowered;
			buffer.i_hwEstopOpened = FSM.hwEstopOpened;
			buffer.i_manualModeEnabled = FSM.manualModeEnabled;
			buffer.i_autoModeEnabled = FSM.autoModeEnabled;
			buffer.i_encoderCount = FSM.encoderCount;


			buffer.o_swEstopEnabled = FSM.swEstopEnabled;
			buffer.o_guiTestModeEnabled = FSM.guiTestModeEnabled;
			buffer.o_openLoopModeEnabled = FSM.openLoopModeEnabled;

			buffer.s_desiredSupplyPres = FSM.desiredSupplyPres;

			buffer.o_flowValveLeft = FSM.flowValveLeft;
			buffer.o_flowValveRight = FSM.flowValveRight;

			buffer.o_coolerEnabled = FSM.coolerEnabled;
			buffer.o_lightEnabled = FSM.lightEnabled;

			buffer.o_highSpeedValEnabled = FSM.highSpeedValEnabled;

			buffer.o_autoModeValEnabled = FSM.autoModeValEnabled;


			FSM.motorErrNumber = gp_motorErr;
			FSM.motorCurrErrNumber = gp_motorCurrErr;
			FSM.motorBatCurrErrNumber = gp_motorBatCurrErr;
			FSM.motorVoltErrNumber = gp_motorVoltErr;
			FSM.motorFaultFlagErrNumber = gp_motorFaultFlagErr;
			FSM.motorCmdActualErrNumber = gp_motorCmdActualErr;


			FSM.motorCurr = gp_motorCurr;
			FSM.motorBatCurr = gp_motorBatCurr;
			FSM.motorVolt = gp_motorVolt;
			FSM.motorBatVolt = gp_motorBatVolt;
			FSM.motorCmdNow = gp_motorCmdNow;

			//release mutex
			rt_mutex_release(&mutexPub);

		}

        rt_fprintf (fp, "%ld.%06ld	%d	%g	%g	%g	%g	%g	%g\n", time_stamp_ms, time_stamp_us, FSM.motorCmdNow, FSM.motorRPMnow, FSM.motorCurr, FSM.motorBatCurr, FSM.motorVolt, FSM.motorBatVolt, FSM.val_pres_oil_3);

	}
}
/********************************************//**
 *  @brief set up the motor controller driver
 *
 *  @return void
 ***********************************************/
void Set_Up_Driver()
{

	motorControllerStatus = device.Connect("/dev/ttyS0"); /* Change this port if required. Compare:'ls cd /dev'	*/

	if(motorControllerStatus != RQ_SUCCESS)
	{
		printf("Error connecting to device: %d\n",motorControllerStatus);
//		exit(EXIT_FAILURE);
	}
	else
	{
		printf("Successfully connected to Roboteq device.\n");
	}
}
/********************************************//**
 *  @brief the non-real time task
 *
 *  1. task_nrt (low priority):\n
 *	-Acquire the sensors inputs and the actuators outputs from RT thread and publish the data\n
 *	-Send the desired motor command to the motor controller (Roboteq API for more information)\n
 *  -acquire the timestamp, the desired motor command, and sensor values from the real-time task\n
 *
 *  @param[in] arg
 *  @return void
 ***********************************************/
void task_nrt(void *arg)
{

		int err = 0, motorErr = 0, motorCurrErr = 0, motorBatCurrErr = 0, motorVoltErr = 0, motorBatVoltErr = 0, motorFaultFlagErr = 0,motorCmdActualErr = 0;
		int motorCurr = 0, motorBatCurr = 0, motorVolt = 0, motorBatVolt = 0, motorFaultFlag = 0, motorCmdActual = 0;
		double motorCmdCal = 0;

		ros::NodeHandle n;
		ros::Publisher publisherLC = n.advertise<xeno_interface::Dimrob>("sensorDimRob", 1);


		/*
		 * Arguments: &task (NULL=self),
		 *            start time,
		 *            period (here: 1 s)
		 */
		rt_task_set_periodic(NULL, TM_NOW,  oneSec_ns/FREQ_HZ_NRT);

		while (1) {
			rt_task_wait_period(NULL);

			//acquire mutex to get the desired motor command and the timestamp
			xeno_interface::Dimrob msg_pub;


			err = rt_mutex_acquire(&mutexPub,TM_INFINITE);
			if(err != 0)
			{
				rt_printf("RT err timeout: %d\n",err);
				timeoutPub++;
			}
			else
			{
				//copy the data in global variables into local variables
				//sensor value was converted to corresponding unit
				msg_pub.s_nowState = buffer.s_nowState;
				msg_pub.s_errCondDetected = buffer.s_errCondDetected;

				msg_pub.o_motorCmd = buffer.o_motorCmd;
				msg_pub.s_motorRPMnow = buffer.s_motorRPMnow;

				msg_pub.i_val_temp_scb = buffer.i_val_temp_scb;
				msg_pub.i_val_temp_mot = buffer.i_val_temp_mot;
				msg_pub.i_val_volt_bat = buffer.i_val_volt_bat;
				msg_pub.i_val_pres_oil_1 = buffer.i_val_pres_oil_1;
				msg_pub.i_val_pres_oil_4 = buffer.i_val_pres_oil_4;
				msg_pub.i_val_pres_oil_5 = buffer.i_val_pres_oil_5;
				msg_pub.i_val_pres_oil = buffer.i_val_pres_oil;
				msg_pub.i_val_flow_oil = buffer.i_val_flow_oil;
				msg_pub.i_val_temp_oil = buffer.i_val_temp_oil;
				msg_pub.i_val_pres_oil_3 = buffer.i_val_pres_oil_3;
				msg_pub.i_val_pres_oil_2 = buffer.i_val_pres_oil_2;

				msg_pub.i_chargerEnabled = buffer.i_chargerEnabled;
				msg_pub.i_chargerPowered = buffer.i_chargerPowered;
				msg_pub.i_hwEstopOpened = buffer.i_hwEstopOpened;
				msg_pub.i_manualModeEnabled = buffer.i_manualModeEnabled;
				msg_pub.i_autoModeEnabled = buffer.i_autoModeEnabled;
				msg_pub.i_encoderCount = buffer.i_encoderCount;

				gp_motorErr = motorErr;
				gp_motorCurrErr = motorCurrErr;
				gp_motorBatCurrErr = motorBatCurrErr;
				gp_motorVoltErr = motorVoltErr;
				gp_motorBatVoltErr = motorBatVoltErr;
				
				//todo change this
				gp_motorCurr = motorCurr*0.1;
				gp_motorBatCurr = motorBatCurr*0.1;
				gp_motorVolt = motorVolt*0.1;
				gp_motorBatVolt = motorBatVolt*0.1;
				gp_motorCmdNow = motorCmdActual;


				//actuator status
				msg_pub.o_swEstopEnabled = buffer.o_swEstopEnabled;
				msg_pub.o_guiTestModeEnabled = buffer.o_guiTestModeEnabled;
				msg_pub.o_openLoopModeEnabled = buffer.o_openLoopModeEnabled;

				msg_pub.s_desiredSupplyPres = buffer.s_desiredSupplyPres;


				msg_pub.o_flowValveLeft = buffer.o_flowValveLeft;
				msg_pub.o_flowValveRight = buffer.o_flowValveRight;

				msg_pub.o_coolerEnabled = buffer.o_coolerEnabled;
				msg_pub.o_lightEnabled = buffer.o_lightEnabled;

				msg_pub.o_highSpeedValEnabled = buffer.o_highSpeedValEnabled;

				msg_pub.o_autoModeValEnabled = buffer.o_autoModeValEnabled;

				//release mutex
				err =rt_mutex_release(&mutexPub);
			}

			//increase or decrease actual motor command gradually with a rate of MOTOR_CMD_RATE
            //msg_pub.o_motorCmd: 	[double] - copy of the desired motor command from RT thread
            //motorCmdCal:			[double]  - calculated motor command based on MOTOR_CMD_RATE
            //motorCmdActual: 		[integer] - actual value commanded to motor (= rounded value of motorCmdCal)
			if(motorCmdCal < msg_pub.o_motorCmd)
			{
				motorCmdCal += MOTOR_CMD_RATE/static_cast<double>(FREQ_HZ_NRT);
				motorCmdCal = min(motorCmdCal, msg_pub.o_motorCmd);
			}
			else if (motorCmdCal > msg_pub.o_motorCmd)
			{
				motorCmdCal -= MOTOR_CMD_RATE/static_cast<double>(FREQ_HZ_NRT);
				motorCmdCal = max(motorCmdCal, msg_pub.o_motorCmd);
			}
			else
			{
				motorCmdCal = msg_pub.o_motorCmd;
			}

			msg_pub.s_motorCmdNow = static_cast<int>(motorCmdCal+0.5); //convert double to int (does not work for negative numbers)
			//add 0.5 because compiler always truncate.


			if((msg_pub.s_nowState  != STATE_IDLE) && (msg_pub.i_manualModeEnabled | msg_pub.i_autoModeEnabled)) //if it is not hardware IDLE mode
			{
				if(motorControllerStatus == RQ_ERR_NOT_CONNECTED) //if the motor controller driver is not initialized
				{
					Set_Up_Driver();
				}

				if(motorControllerStatus == RQ_SUCCESS)
				{
					//limit the maximum motor command to MAX_MOTOR for safety
					if((msg_pub.s_nowState  == STATE_AUTO) || (msg_pub.s_nowState  == STATE_TEST)) //use of serial communication for motor command (only TRUE for Auto mode loop == 3)
					{

						if((msg_pub.s_motorCmdNow) < MAX_MOTOR)
						{
							rt_printf("SetCommand (_GO, 1, %d)", msg_pub.s_motorCmdNow);

							if((motorErr = device.SetCommand(_GO, 1, msg_pub.s_motorCmdNow)) != RQ_SUCCESS)
							{
								rt_printf("...failed --> %d \n", motorErr);
							}
							else
							{
								rt_printf("...succeeded.\n");
							}
						}
						else
						{
							rt_printf("Max limit of motor command exceeded. \n");
							rt_printf("SetCommand (_GO, 1, %d)...", MAX_MOTOR);
							msg_pub.s_motorCmdNow = MAX_MOTOR;

							if((motorErr = device.SetCommand(_GO, 1, msg_pub.s_motorCmdNow)) != RQ_SUCCESS)
							{
								rt_printf("...failed --> %d \n", motorErr);
							}
							else
							{
								rt_printf("...succeeded.\n");
							}
						}
					}
						//read sensor values
						if((motorFaultFlagErr = device.GetValue(_FLTFLAG, motorFaultFlag)) != RQ_SUCCESS)
						rt_printf("GetValue (_FLTFLAG) failed --- Error %d \n", motorFaultFlagErr);
						else
						rt_printf("GetValue (_FLTFLAG, %d) \n", motorFaultFlag);

						if((motorCurrErr = device.GetValue(_MOTAMPS, 1, motorCurr)) != RQ_SUCCESS)
						rt_printf("GetValue (_MOTAMPS, 1) failed --- Error %d \n", motorCurrErr);
						else
						rt_printf("GetValue (_MOTAMPS, 1, %d) \n", motorCurr);

						if((motorBatCurrErr = device.GetValue(_BATAMPS, 1, motorBatCurr)) != RQ_SUCCESS)
						rt_printf("GetValue (_BATAMPS, 1) failed --- Error %d \n", motorBatCurrErr);
						else
						rt_printf("GetValue (_BATAMPS, 1, %d) \n", motorBatCurr);

						if((motorVoltErr = device.GetValue(_VOLTS, 1, motorVolt)) != RQ_SUCCESS)
						rt_printf("GetValue (_VOLTS, 1) failed --- Error %d \n", motorVoltErr);
						else
						rt_printf("GetValue (_VOLTS, 1, %d) \n", motorVolt);

						if((motorBatVoltErr = device.GetValue(_VOLTS, 2, motorBatVolt)) != RQ_SUCCESS)
						rt_printf("GetValue (_VOLTS, 2) failed --- Error %d \n", motorBatVoltErr);
						else
						rt_printf("GetValue (_VOLTS, 2, %d) \n", motorBatVolt);

						if((motorCmdActualErr = device.GetValue(_MOTCMD, 1, motorCmdActual)) != RQ_SUCCESS)
						rt_printf("GetValue (_MOTCMD , 1) failed --- Error %d \n", motorCmdActualErr);
						else
						rt_printf("GetValue (_MOTCMD , 1, %d) \n", motorCmdActual);
				}
				else
				{
					motorFaultFlagErr = RQ_ERR_NOT_CONNECTED;
				}



			}

			msg_pub.s_motorCmdActual = motorCmdActual;
			msg_pub.s_motorFaultFlag = motorFaultFlag;
			
			msg_pub.s_motorCurr = motorCurr*0.1; //convert to Amps
			msg_pub.s_motorBatCurr = motorBatCurr*0.1;//convert to Amps
			msg_pub.s_motorVolt = motorVolt*0.1; //convert to Volt
			msg_pub.s_motorBatVolt = motorBatVolt*0.1;//convert to Volt
			
			msg_pub.s_nrtFreq = FREQ_HZ_NRT;
			
			publisherLC.publish(msg_pub);

		}


}



/********************************************//**
 *  @brief Configuring the board
 *
 *	This function reads config/conf.ini file and open DAQ device.
 *
 *  @param	void
 *  @return void
 ***********************************************/
void configure_board(void)
{
    char ini_name[] = "/home/adrl/catkin_ws/src/xeno_interface/src/config/conf.ini";//"src/xeno_interface/src/config/conf.ini";
    char ini_key_tmpl[] = "pci_%d:filename";
    char ini_key[64];
    char * pci_conf_filename;

    dictionary * ini = NULL;

    // load config file
    ini = iniparser_new(ini_name);
    if (ini==NULL) {
        fprintf(stderr, "cannot parse file: %s\n", ini_name);
        exit(1);
    }

    iniparser_dump(ini, stderr);

    //writes into string i.e. (ini_key == dimRob:pci_num_board)
    sprintf(ini_key, "dimRob:pci_num_board");

    /*
    This function queries a dictionary for a key. A key as read from an
     ini file is given as "section:key". If the key cannot be found,
     the notfound value is returned.

     Store "pci_board_num" from config file into "pci_board_num" global variable
    */
    pci_board_num = iniparser_getint(ini, ini_key, 0);

    for (int i=0; i<pci_board_num; i++) {
        snprintf(ini_key, sizeof(ini_key), ini_key_tmpl, i);
        fprintf(stderr, "ini_key %s\n", ini_key);
        pci_conf_filename = iniparser_getstring(ini, ini_key, NULL); //search key string from "ini" dictionary and return
        pciboards[i] = new pciAnalogy("NI m series DAQ", pci_conf_filename);
        pciboards[i]->Open();
    }

    iniparser_free(ini);

}

/********************************************//**
 *  @brief Close the board
 *
 *  @param	void
 *  @return void
 ***********************************************/
void close_board(void) {


    // close boards and free resource
    for (int i=0; i<pci_board_num; i++) {
        if (pciboards[i]->boardIsOpen) {
            pciboards[i]->Close();
        }
        delete pciboards[i];
    }

}

/********************************************//**
 *  @brief Warn the switch to secondary mode by logging into file and by printing to terminal
 *
 *	Mode switches are monitored in three methods:\n
 *	1. the backtrace message is logged in build/backtrace.txt\n
 *	2. the backtrace message is printed out to the terminal\n
 *	3. modeSwitchesCount is incremented for every mode switch\n
 *
 *	Note: Xenomai Watchdog is also counted as the mode switch\n
 *
 *  @param	sig __attribute__((unused))
 *  @return void
 ***********************************************/
void warn_upon_switch(int sig __attribute__((unused)))

{
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
//    backtrace_symbols_fd(bt,nentries,fileno(stdout));
    backtrace_symbols_fd(bt,nentries,fileno(backtrace_fd));

    modeSwitchesCount++;
}

/********************************************//**
 *  @brief Empty function used by termination signals
 *
 *	Signal catcher - required to end program with interrupts (e.g. keyboard shortcut Ctrl+C )
 *
 *  @return void
 ***********************************************/
void catch_signal(int sig)
{
}


/********************************************//**
 *  @brief Setup Xenomai related signals
 *
 *  @return void
 ***********************************************/
void setupXenomai()
{
    /*
	 * The SIGTERM signal is a generic signal used to cause program termination.\n
	 * Unlike SIGKILL, this signal can be blocked, handled, and ignored. It is the normal way to politely ask a program to terminate.
	 * The shell command kill generates SIGTERM by default.
	 *
	 * [http://www.gnu.org/software/libc/manual/html_node/Termination-Signals.html]
	 *
	 */
	signal(SIGTERM, catch_signal);
	/*
	 * The SIGINT (“program interrupt”) signal is sent when the user types the INTR character (normally C-c).
	 *
	 * [http://www.gnu.org/software/libc/manual/html_node/Termination-Signals.html]
	 */
	signal(SIGINT, catch_signal);

	signal(SIGXCPU, warn_upon_switch);

	/* no memory-swapping for this programm */
	int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
	if (ret) {
		perror("ERROR : mlockall has failled");
		exit(1);
	}


	/*
	 * This is a real-time compatible printf() package from
	 * Xenomai's RT Development Kit (RTDK), that does NOT cause
	 * any transition to secondary (i.e. non real-time) mode when
	 * writing output.
	 */
	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	rt_print_auto_init(1);
}

/********************************************//**
 *  @brief main function for calling real-time threads
 *
 *  Configure board, and calls real-time threads in order. \n
 *  e.g. if more than one thread is declared, a thread must end by pressing ctr-c in order to start the next thread.
 *	When the last thread is terminated by pressing ctr-c by the user, the DAQ board is automatically closed.
 *  @return 0
 ***********************************************/
int main(int argc, char* argv[])
{
    backtrace_fd = fopen(backtrace_file, "w");

	setupXenomai();

	// read configuration and open a device
    configure_board();

	//initialize ROS loop
	ros::init(argc, argv, "lowCtrlDimRob");



  	rt_mutex_create(&mutexPub,"MutexPub");

 	//Start the real-time thread
	rt_task_create(&demo_task_rt, "dimRob_rt", 0, 99, 0);
	rt_task_start(&demo_task_rt, &task_rt, NULL);


 	//create and start non-real time thread
 	rt_task_create(&demo_task_nrt, "dimRob_nrt", 0, 0, 0);
 	rt_task_start(&demo_task_nrt, &task_nrt, NULL);


	pause();

	//Delete threads


	rt_task_delete(&demo_task_nrt);
	rt_task_delete(&demo_task_rt);

	rt_mutex_delete(&mutexPub);

	device.Disconnect();

	close_board();


	fprintf(stderr, "end of main, Mode Switches = %d, Time out (mutexPub) = %d\n", modeSwitchesCount, timeoutPub);

    fclose(backtrace_fd);


	return 0;
}

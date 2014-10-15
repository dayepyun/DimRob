/** @file stateMachine.h
 *  @brief Declaration of stateMachine class.
 *
 *  All the variables for DAQ inputs and outputs are declared.\n
 *  Also, state machine loop functions (idle, error, manual, auto, and test state) are declared.\n
 *
 *  @date	02.06.2014
 *  @author 	R. Pyun
 */


#ifndef stateMachine_H_
#define stateMachine_H_

#include "pciAnalogy.h"
#include <native/timer.h>
#include "ErrorCodes.h"

/********************************************//**
 * @brief The enumeration of state machine status
 ***********************************************/
enum STATE
{
	STATE_IDLE =0,
	STATE_ERROR,
	STATE_MANUAL,
	STATE_AUTO,
	STATE_TEST
};

/********************************************//**
 * @brief Class for input and output variables and state machine related functions
 ***********************************************/
class stateMachine
{
public:

	RTIME 	idleSetTime;			///< Settling time in Idle state
	int    	motorErrNumber;     		///< Motor controller error
	int	motorCurrErrNumber; 		///< Error number for motor controller - Motor Current
	int	motorBatCurrErrNumber; 		///< Error number for motor controller - Battery current
	int	motorVoltErrNumber; 		///< Error number for motor controller - Internal voltage
	int	motorBatVoltErrNumber; 		///< Error number for motor controller - Battery voltage
	int	motorCmdActualErrNumber; 	///< Error number for motor controller command verify via serial
	int	motorFaultFlagErrNumber; 	///< Error number for motor controller fault flag

	int	daqErr;				///< Read sensor error
	int 	daqOutErr;			///< Write output error

	double 	motorCurr;     			///< Motor controller - Motor Current
	double  motorBatCurr;     		///< Motor controller - Battery current
	double  motorVolt;     			///< Motor controller - Internal voltage
	double 	motorBatVolt;     		///< Motor controller - Battery voltage
	int    	motorCmdActual;     		///< Motor controller command verify via serial
	int    	motorFaultFlag;     		///< Motor controller fault flag
	
	//same list as ROS message in /xeno_inteface/msg/Hyd.msg
	int		nowState; 		///< Current state in state machine
	double	motorCmd; 			///< Desired motor command
	int 	motorCmdNow;			///< Current motor command
	bool	errCondDetected;		///< Error condition detected;
	bool    guiTestModeEnabled; 		///< Check box of test mode in GUI
	bool	openLoopModeEnabled;		///< Check box of open-loop mode in GUI

	//encoder
	signed 	encoderCount;			///< Count from the encoder (DIF_ENC_CLK)
	signed 	encoderPreCount;		///< Count from the encoder in previous loop
	double 	desiredSupplyPres;		///< Desired supply pressure
	double  motorRPMnow;			///< Calculated Motor RPM
	int 	freqLoop;			///< Frequency of the control loop

	//analog input
	double	val_temp_scb;			///< Voltage signal of NI SCB 68A on-board temperature sensor (AI_TEMP_SCB)
	double	val_temp_mot;			///< Voltage signal of motor temperature sensor (AI_TEMP_MOT)
	double	val_volt_bat;			///< Voltage signal of battery voltage sensor (AI_VOLT_BAT)
	double	val_pres_oil_1;			///< Voltage signal of Rexroth oil pressure sensor 1 (AI_PRES_OIL_1)
	double	val_pres_oil_4;			///< Voltage signal of Rexroth oil pressure sensor 4 (AI_PRES_OIL_4)
	double	val_pres_oil_5;			///< Voltage signal of Rexroth oil pressure sensor 5 (AI_PRES_OIL_5)
	double	val_pres_oil;			///< Voltage signal of Parker oil pressure sensor (AI_PRES_OIL)
	double	val_flow_oil;			///< Voltage signal of Parker oil flow sensor (AI_FLOW_OIL)
	double	val_temp_oil;			///< Voltage signal of Parker oil temperature sensor (AI_TEMP_OIL)
	double	val_pres_oil_3;			///< Voltage signal of Rexroth oil pressure sensor 3 (AI_PRES_OIL_3)
	double	val_pres_oil_2;			///< Voltage signal of Rexroth oil pressure sensor 2 (AI_PRES_OIL_2)

	unsigned short 	raw_temp_scb;		///< Rawdata of NI SCB 68A on-board temperature sensor (AI_TEMP_SCB)
	unsigned short 	raw_temp_mot;		///< Rawdata of motor temperature sensor (AI_TEMP_MOT)
	unsigned short 	raw_volt_bat;		///< Rawdata of battery voltage sensor (AI_VOLT_BAT)
	unsigned short 	raw_pres_oil_1;		///< Rawdata of Rexroth oil pressure sensor 1 (AI_PRES_OIL_1)
	unsigned short 	raw_pres_oil_4;		///< Rawdata of Rexroth oil pressure sensor 4 (AI_PRES_OIL_4)
	unsigned short 	raw_pres_oil_5;		///< Rawdata of Rexroth oil pressure sensor 5 (AI_PRES_OIL_5)
	unsigned short 	raw_pres_oil;		///< Rawdata of Parker oil pressure sensor (AI_PRES_OIL)
	unsigned short 	raw_flow_oil;		///< Rawdata of Parker oil flow sensor (AI_FLOW_OIL)
	unsigned short 	raw_temp_oil;		///< Rawdata of Parker oil temperature sensor (AI_TEMP_OIL)
	unsigned short 	raw_pres_oil_3;		///< Rawdata of Rexroth oil pressure sensor 3 (AI_PRES_OIL_3)
	unsigned short 	raw_pres_oil_2;		///< Rawdata of Rexroth oil pressure sensor 2 (AI_PRES_OIL_2)

	//analog output
	double	flowValveLeft;			///< Voltage signal for flow valve control of left track (AO_FLOW_VAL_L)
	double	flowValveRight; 		///< Voltage signal for flow valve control of right track (AO_FLOW_VAL_R)

	//digital input/output
	bool 	swEstopEnabled;			///< Soft E-Stop enabled (DO_ESTOP_SW)
	bool 	coolerEnabled;			///< Cooler enabled (DO_COOLER)
	bool 	lightEnabled;			///< Signal light enabled (DO_LIGHT)

	bool 	chargerEnabled;			///< Charger enabled (DI_CHAR_SIG)
	bool	chargerPowered;			///< Charger powered (DI_CHAR_PWR)
	bool	hwEstopOpened;			///< Hard E-Stop Opened (DI_ESTOP_HW)

	bool	highSpeedValEnabled;		///< High speed mode valve enabled (DO_SPEED_VAL)
	bool	autoModeValEnabled;		///< Auto mode valve enabled (DO_MODE_VAL)

	bool 	manualModeEnabled;		///< Manual mode toggle switch enabled (DIF_MANUAL)
	bool	autoModeEnabled;		///< Auto mode toggle switch enabled (DIF_AUTO)


public:
	stateMachine(void);
	~stateMachine(void);

	int InitFSM(pciBase * pciBoard);
	int ReadSensors(pciBase * pciBoard);
	void CheckErrCondition();

	int SetIdleState(pciBase * pciBoard);
	int SetErrorState(pciBase * pciBoard);
	int SetManualState(pciBase * pciBoard);
	int SetAutoState(pciBase * pciBoard);
	int SetTestState(pciBase * pciBoard);

	inline int GetNowState(){return nowState;} ///< return current state

};


#endif /* stateMachine_H_ */


/** @file stateMachine.cpp
 *  @brief Finite State Machine (FSM) implementations
 *
 *	@date	02.06.2014
 *  @author R. Pyun
 *
 */

#include <iostream>
#include <array>

#include "stateMachine.h"
#include "math.h"

using namespace std;

stateMachine::stateMachine(void)
{
}

stateMachine::~stateMachine(void)
{
}


/********************************************//**
 *  @brief Initialization of the state machine
 *
 *	Set output variables to 0.\n
 *	Except errCondDetected is set to true.\n
 *	Also, encoderX4Enabled is declared as specified in config/pci6229conf.ini (0 for x1 mode, and 1 for x4 mode).\n
 *
 *	The state machine is initiated as Idle state. \n
 *	The counter to process encoder data for the motor is initialized.
 *
 *	@param[in] pb	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::InitFSM(pciBase * pciBoard)
{
	int err = 0;
	bool encoderMode = pciBoard->cnt_conf[0];//defined in config/pci6229conf.ini (0 for x1 mode, and 1 for x4 mode)

	motorCmd = 0;
	motorCmdNow = 0;
	errCondDetected = true;
	guiTestModeEnabled = false;

	encoderCount = 0;
	encoderPreCount = 0;
	desiredSupplyPres = 0;
	motorRPMnow = 0;


	flowValveLeft = 0;
	flowValveRight = 0;

	swEstopEnabled = false;
	coolerEnabled = false;
	lightEnabled = false;

	highSpeedValEnabled = false;
	autoModeValEnabled = false;

	SetIdleState(pciBoard);

	//initialize counter

	err = pciBoard->WriteDO(SUBD_DIO, DO_ENC_MODE , encoderMode);
	err += pciBoard->StartCNT(SUBD_CNT, DIF_ENC_CLK);

	return err;

}

/********************************************//**
 *  @brief Read input values including counter
 *
 *  Read all sensor readings and convert them in prorper unit. (otherwise, in voltage)
 *
 *  @param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 *
 ***********************************************/
int stateMachine::ReadSensors(pciBase * pciBoard)
{
	int err = 0;

//	array<int,11> aiList = {AI_TEMP_SCB, AI_TEMP_MOT, AI_VOLT_BAT, AI_PRES_OIL_1, AI_PRES_OIL_4,AI_PRES_OIL_5,AI_PRES_OIL,AI_FLOW_OIL,AI_TEMP_OIL,AI_PRES_OIL_3,AI_PRES_OIL_2};
//	array<double,11> *aiValueList;
//	aiValueList = {&val_temp_scb, &val_temp_mot, &val_volt_bat, &val_pres_oil_1, &val_pres_oil_4, &val_pres_oil_5, &val_pres_oil, &val_flow_oil, &val_temp_oil, &val_pres_oil_3, &val_pres_oil_2};
//
////	double *aiValueList [1];
////	aiValueList [0]= {&val_temp_scb};
//	array<unsigned short,11> aiRawList = {raw_temp_scb, raw_temp_mot, raw_volt_bat, raw_pres_oil_1, raw_pres_oil_4,raw_pres_oil_5,raw_pres_oil,raw_flow_oil,raw_temp_oil,raw_pres_oil_3,raw_pres_oil_2};
//
//	array<int,2> subd_dioList = {SUBD_DIO, SUBD_DIO_F};
//	array<int,5> diList = {DI_CHAR_SIG, DI_CHAR_PWR, DI_ESTOP_HW, DIF_MANUAL, DIF_AUTO};
//	array<bool,5> diStateList = {chargerEnabled, chargerPowered, hwEstopOpened, manualModeEnabled, autoModeEnabled};
//
//	//read sensor value (AI)
//	for (int i = 0; i < aiList.size(); i++)
//	{
//
//		if ((err = pciBoard->ReadAD(aiList[i], *aiValueList[i], aiRawList[i])) < 0)
//		{
//			return err;
//		}
//	}
//
//	//read sensor value (DI)
//	for (int j = 0; j < diList.size(); j++)
//	{
//		// to get the appropriate index of the subd_dioList
//		int subd_dio = j < 3 ? 0 : 1;
//
//		if ((err = pciBoard->ReadDI(subd_dioList[subd_dio],diList[j], diStateList[j])) < 0)
//		{
//			return err;
//		}
//	}
//
//
//	val_temp_scb = val_temp_scb*100;
//	//val_temp_mot = ??? //todo tempMotor conversion from voltage to °C
//	//val_volt_bat = val_volt_bat*6;//todo wrong conversion
//	val_pres_oil_1 = (val_pres_oil_1-0.5)*62.5;
//	val_pres_oil_4 = (val_pres_oil_4-0.5)*62.5;
//	val_pres_oil_5 = (val_pres_oil_5-0.5)*62.5;
//	val_pres_oil = val_pres_oil*40;
//	val_flow_oil = (val_flow_oil-0.928)*60/3.712;
//	val_temp_oil = (val_temp_oil*175/4.64)-50;
//	val_pres_oil_3 = (val_pres_oil_3-0.5)*62.5;
//	val_pres_oil_2 = (val_pres_oil_2-0.5)*62.5;
//
//	err = pciBoard->ReadCNT(SUBD_CNT, DIF_ENC_CLK, encoderCount, freqLoop, motorRPMnow);
//
//	return err;

		//read sensor value
		err = pciBoard->ReadAD(AI_TEMP_SCB, val_temp_scb, raw_temp_scb);
		err = pciBoard->ReadAD(AI_TEMP_MOT, val_temp_mot, raw_temp_mot);
		err = pciBoard->ReadAD(AI_VOLT_BAT, val_volt_bat, raw_volt_bat);
		err = pciBoard->ReadAD(AI_PRES_OIL_1, val_pres_oil_1, raw_pres_oil_1);
		err = pciBoard->ReadAD(AI_PRES_OIL_4, val_pres_oil_4, raw_pres_oil_4);
		err = pciBoard->ReadAD(AI_PRES_OIL_5, val_pres_oil_5, raw_pres_oil_5);
		err = pciBoard->ReadAD(AI_PRES_OIL, val_pres_oil, raw_pres_oil);
		err = pciBoard->ReadAD(AI_FLOW_OIL, val_flow_oil, raw_flow_oil);
		err = pciBoard->ReadAD(AI_TEMP_OIL, val_temp_oil, raw_temp_oil);
		err = pciBoard->ReadAD(AI_PRES_OIL_3, val_pres_oil_3, raw_pres_oil_3);
		err = pciBoard->ReadAD(AI_PRES_OIL_2, val_pres_oil_2, raw_pres_oil_2);

		val_temp_scb = val_temp_scb*100;
		//val_temp_mot = ??? //todo tempMotor conversion from voltage to °C
		//val_volt_bat = val_volt_bat*6;//todo wrong conversion
		val_pres_oil_1 = (val_pres_oil_1-0.5)*62.5;
		val_pres_oil_4 = (val_pres_oil_4-0.5)*62.5;
		val_pres_oil_5 = (val_pres_oil_5-0.5)*62.5;
		val_pres_oil = val_pres_oil*40;
		val_flow_oil = (val_flow_oil-0.928)*60/3.712;
		val_temp_oil = (val_temp_oil*175/4.64)-50;
		val_pres_oil_3 = (val_pres_oil_3-0.5)*62.5;
		val_pres_oil_2 = (val_pres_oil_2-0.5)*62.5;

		err = pciBoard->ReadDI(SUBD_DIO,DI_CHAR_SIG, chargerEnabled);
		err = pciBoard->ReadDI(SUBD_DIO,DI_CHAR_PWR, chargerPowered);
		err = pciBoard->ReadDI(SUBD_DIO,DI_ESTOP_HW, hwEstopOpened);
		err = pciBoard->ReadDI(SUBD_DIO_F,DIF_MANUAL, manualModeEnabled);
		err = pciBoard->ReadDI(SUBD_DIO_F,DIF_AUTO, autoModeEnabled);
		err = pciBoard->ReadCNT(SUBD_CNT, DIF_ENC_CLK, encoderCount, freqLoop, motorRPMnow);


		return err;
}

/********************************************//**
 *  @brief Check error condition
 *
 *  Set errCondDetected as false only if error condition is not detected.\n
 *  Otherwise, errCondDetected is set to true, which would result in Estop state.
 *
 *  @return 	void
 *
 ***********************************************/
void stateMachine::CheckErrCondition()
{
	//todo implement error condition check algorithm

	errCondDetected = false;

	//check if everything is in good condition (i.e. no error), then set errCondDetected to false.
	if(motorCmd >= 0) //motor command is positive
	{
		if(val_pres_oil_3 < 200) //supply pressure is less than 200 bar
		{
			if(val_temp_scb < 70) // power electronics box temperature is less than 65°C (= 0.65 V)
			{
				if(motorErrNumber == RQ_SUCCESS) //no motor controller error from serial communication
				{
					if(motorCurrErrNumber == RQ_SUCCESS)
					{
						if(motorBatCurrErrNumber == RQ_SUCCESS)
						{
							if(motorBatVoltErrNumber  == RQ_SUCCESS)
							{
								if((motorCmd <= 1000) && (motorCmd >= 0)) //valid range of motor commands
								{
									if((fabs(flowValveLeft) <= 10) && (fabs(flowValveRight) <= 10)) //valid range of Moog Valve command
									{
										if(!((manualModeEnabled == true) && (autoModeEnabled == true))) //manual and auto mode at the same time
										{
											if(motorFaultFlag == 0)
											{
												if((daqErr >= 0) && (daqOutErr >= 0)) //sensor reading error from DAQ
												{
													if(motorFaultFlagErrNumber == RQ_SUCCESS)
													{
														//if(val_temp_mot < 4) // motor temperature sensor is less than 4V
														errCondDetected = false;

													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

/********************************************//**
 *  @brief Transition to Idle loop
 *
 *  Idle:\n
 *
 *  - Initial loop when computer starts up
 *
 *
 *  		a. Mode Valve:	  	Center
 *  		b. Speed Valve:		Off
 *  		c. MOOGs:			Center - 0 for CH_FVAL_L and CH_FVAL_R
 *  		d. Motor Command:	Off - done in hardware - disable MicroScript
 *  		e. Other outputs:	Off - Cooler and Signal light
 *
 *  - Wait for toggle switch input for Manual or Auto mode -> Manual mode loop or Auto mode loop (or Test loop)
 *  - Check error condition and E-stop status -> Error loop
 *
 *  @param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::SetIdleState(pciBase * pciBoard)
{
	int err = 0;

	//a. Mode valve at center -> low on CH_MVAL
	autoModeValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_MODE_VAL,autoModeValEnabled);
	if(err < 0)
	{
		return err;
	}

	//b. Speed valve off
	highSpeedValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_SPEED_VAL, highSpeedValEnabled);
	if(err < 0)
	{
		return err;
	}

	//c. MOOGs flow valve at center
	flowValveLeft = 0;
	flowValveRight = 0;
	err = pciBoard->WriteDA(AO_FLOW_VAL_L,flowValveLeft);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDA(AO_FLOW_VAL_R,flowValveRight);
	if(err < 0)
	{
		return err;
	}

	//d.  Motor Command: Off - done in hardware
	motorCmd = 0; //set to 0 for safety


	//e.  Other outputs: Off - Cooler and Signal light
	coolerEnabled = false;
	lightEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_COOLER, coolerEnabled);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDO(SUBD_DIO,DO_LIGHT, lightEnabled);
	if(err < 0)
	{
		return err;
	}

	nowState = STATE_IDLE;

	idleSetTime = rt_timer_read();

	return err;

}

/********************************************//**
 *  @brief Transition to Error loop

 *  Error:\n
 *  - Enter the loop when error is detected OR E-stop is pressed
 *  - Exit the loop when error is removed AND E-stop is not pressed
 *
 *
 *   	a. Mode Valve:	  	Center
 *  	b. Speed Valve:		Off
 *  	c. MOOGs:			Center
 *  	d. Motor Command:	0
 *		e. Other outputs: 	Off - Cooler and Signal light
 *
 *  @param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::SetErrorState(pciBase * pciBoard)
{
	int err = 0;

	//a. Mode valve at center -> low on CH_MVAL
	autoModeValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_MODE_VAL, autoModeValEnabled);
	if(err < 0)
	{
		return err;
	}

	//b. Speed valve off
	highSpeedValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_SPEED_VAL, highSpeedValEnabled);
	if(err < 0)
	{
		return err;
	}

	//c. MOOGs flow valve at center
	flowValveLeft = 0;
	flowValveRight = 0;
	err = pciBoard->WriteDA(AO_FLOW_VAL_L,flowValveLeft);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDA(AO_FLOW_VAL_R,flowValveRight);
	if(err < 0)
	{
		return err;
	}

	//d.  Motor Command:	0
	motorCmd = 0;


	//e.  Other outputs: Off - Cooler and Signal light
	coolerEnabled = false;
	lightEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_COOLER, coolerEnabled);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDO(SUBD_DIO,DO_LIGHT, lightEnabled);
	if(err < 0)
	{
		return err;
	}

	nowState = STATE_ERROR;

	return err;

}

/********************************************//**
 *  @brief Transition to Manual mode loop
 *
 *  Manual mode:\n
 *	- Enter the loop from Idle loop when toggle switch is set to Manual mode
 *	- Exit to Idle loop if toggle switch status is not Manual mode
 *	- Exit to Error loop in case of error is detected or E-stop is pressed
 *
 *
 *   	a. Mode Valve:	  	Passive (Center command)
 *  	b. Speed Valve:		Off
 *  	c. MOOGs:			Center
 *  	d. Motor Command:	0 -MicroScript - constant motor command (disable the motor command from serial communication)
 *
 *	@param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::SetManualState(pciBase * pciBoard)
{
	int err = 0;

	//a. Mode valve at Passive (Center command) -> low on CH_MVAL
	autoModeValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_MODE_VAL,autoModeValEnabled);
	if(err < 0)
	{
		return err;
	}

	//b. Speed valve off
	highSpeedValEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_SPEED_VAL, highSpeedValEnabled);
	if(err < 0)
	{
		return err;
	}

	//c. MOOGs flow valve at center
	flowValveLeft = 0;
	flowValveRight = 0;
	err = pciBoard->WriteDA(AO_FLOW_VAL_L,flowValveLeft);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDA(AO_FLOW_VAL_R,flowValveRight);
	if(err < 0)
	{
		return err;
	}

	//d. Motor Command:	MicroScript - constant motor command (disable the motor command from serial communication)
	motorCmd = 0; // not used in manual mode

	nowState = STATE_MANUAL;

	return err;

}

/********************************************//**
 *  @brief Transition to Auto mode loop
 *
 *  Auto mode:\n
 *	- Enter the loop from Idle loop when toggle switch is set to Auto mode AND Test mode is disabled
 *	- Exit to Idle loop if toggle switch status is not Auto mode
 *	- Exit to Error loop in case of error is detected or E-stop is pressed
 *	- Exit to Test loop if Test mode is enabled in GUI
 *
 *
 *   	a. Mode Valve:	  	Left
 *  	b. Speed Valve:		Modulated vs. Speed command (but, for SetAutoState, the valve is set to 0)
 *  	c. MOOGs:			Selectable (but, for SetAutoState, the valves are set to 0)
 *  	d. Motor Command:	Serial communication - Modulated vs. Supply Pressure (but, for SetAutoState, the motor command is set to 0)
 *
 *  @param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::SetAutoState(pciBase * pciBoard)
{
	int err = 0;

	//turn light on when auto mode
	lightEnabled = false;
	err = pciBoard->WriteDO(SUBD_DIO,DO_LIGHT, lightEnabled);
	if(err < 0)
	{
		return err;
	}


	//a. Mode Valve:	  	Left
	autoModeValEnabled = true;
	err = pciBoard->WriteDO(SUBD_DIO,DO_MODE_VAL,autoModeValEnabled);
	if(err < 0)
	{
		return err;
	}

	//b. Speed Valve:		Modulated vs. Speed command
	highSpeedValEnabled = false; //set it to low speed at the transition to Auto state.
	err = pciBoard->WriteDO(SUBD_DIO,DO_SPEED_VAL, highSpeedValEnabled);
	if(err < 0)
	{
		return err;
	}

	//c. MOOGs:			Selectable
	flowValveLeft = 0;
	flowValveRight = 0;
	err = pciBoard->WriteDA(AO_FLOW_VAL_L,flowValveLeft);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDA(AO_FLOW_VAL_R,flowValveRight);
	if(err < 0)
	{
		return err;
	}
	//d. Motor Command:	Serial communication - Modulated vs. Supply Pressure
	motorCmd = 0; // desired motor command

	nowState = STATE_AUTO;

	return err;

}

/********************************************//**
 *  @brief Transition to Test mode loop
 *
 *  Test mode:\n
 *	- Enter the loop from Idle loop when toggle switch is set to Auto mode and Test mode is enabled
 *	- Exit to Idle loop if toggle switch status is not Auto mode OR Test mode is disabled
 *	- Exit to Error loop in case of error is detected or E-stop is pressed
 *
 *
 *   	a. Mode Valve:	  	Left
 *  	b. Speed Valve:		Selectable
 *  	c. MOOGs:			Selectable
 *  	d. Motor Command:	Open-loop control
 *
 *  @param[in] pciBoard	pciBase class
 *  @return 		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int stateMachine::SetTestState(pciBase * pciBoard)
{
	int err = 0;


	//a. Mode Valve:	  	Left
	autoModeValEnabled = true;
	err = pciBoard->WriteDO(SUBD_DIO,DO_MODE_VAL,autoModeValEnabled);
	if(err < 0)
	{
		return err;
	}

	//b. Speed Valve:		Selectable
	highSpeedValEnabled = false; //set it to low speed at the transition to Test state.
	err = pciBoard->WriteDO(SUBD_DIO,DO_SPEED_VAL, highSpeedValEnabled);
	if(err < 0)
	{
		return err;
	}

	//c. MOOGs:				Selectable
	flowValveLeft = 0;
	flowValveRight = 0;
	err = pciBoard->WriteDA(AO_FLOW_VAL_L,flowValveLeft);
	if(err < 0)
	{
		return err;
	}
	err = pciBoard->WriteDA(AO_FLOW_VAL_R,flowValveRight);
	if(err < 0)
	{
		return err;
	}

	//d. Motor Command:	Serial communication
	motorCmd = 0; // desired motor command

	nowState = STATE_TEST;

	return err;

}

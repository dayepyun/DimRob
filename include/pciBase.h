/** @file pciBase.h
 *  @brief Declaration of the device specific configuration macros and pciBase class, a parent class of pciAnalogy.
 *
 *  1. Configuration macros that are specific to the DAQ device is defined here (e.g. number of analog input channel, number of digital input, etc).\n
 *  If the DAQ device is changed to another model, the configuration macros should be modified.\n
 *  For our implementation, the device configuration macros for NI PCI 6221 and NI PCI 6229 devices are defined.\n
 *  Therefore, either PCI 6221 or PCI 6229 DAQ card can be used by simply defining correct device number on the macro (i.e. DEVICE) in the file.\n
 *
 *  2. The channels for all sensors and actuators are defined as specified in dropbox/DIMROB/DAQ/pinLayout.ods.\n
 *
 *  3. Structures to store data for logging or debugging are defined here. \n
 *
 *  4. pciBase class, a parent class of pciAnalogy class is declared.\n
 *
 *
 *  @author R. Pyun
 *
 */


#ifndef PCIBASE_H_
#define PCIBASE_H_



#include <sys/time.h>
#include <pthread.h>

#define DEBUG 0 ///< set it to 1 for debugging (to see printf values)

/********************************************//**
 *  This changes the device specific information including the calibration coefficient (defined in pciAnalogy.cpp) for analog signal.
 *  (i.e. ADC_NUM, DAC_NUM, CNT_NUM....DCH_NUM)
 ***********************************************/
//#define DEVICE 6221				///<Define DAQ device model number (i.e. 6221 or 6229)
#define DEVICE 6229				///<Define DAQ device model number (i.e. 6221 or 6229)

//Device specific information
#if DEVICE == 6229
//PCI 6229
#define ADC_NUM             32	///<number of analog input channel
#define DAC_NUM             4	///<number of analog output channel
#define CNT_NUM             2	///<number of counter channel
#define PWM_NUM             2	///<number of PWM channel
#define ENC_NUM             2	///<number of encoder channel
#define DIO_NUM             48	///<number of all digital channel (i.e. P0.x, P1.x and P2.x pins)
#define PFI_NUM             16	///<number of PFI channel (i.e. P1.x and P2.x pins)
#define DGR_NUM             12	///<number of digital input output group: defined in "config" folder
#define DCH_NUM             4	///<number of digital channel in each group

/********************************************************/
//List of Channels
//AI
#define AI_TEMP_SCB		0 	///< AI 0: NI SCB 68A on-board Temperature Sensor
#define AI_TEMP_MOT		1 	///< AI 1: Motor Temperature Sensor
#define AI_VOLT_BAT		2 	///< AI 2: Battery Voltage Sensor

#define AI_PRES_OIL_1	16 	///< AI 16: Rexroth Oil Pressure Sensor 1
#define AI_PRES_OIL_4	17 	///< AI 17: Rexroth Oil Pressure Sensor 4
#define AI_PRES_OIL_5	18 	///< AI 18: Rexroth Oil Pressure Sensor 5
#define AI_PRES_OIL		19 	///< AI 19: Parker Oil Pressure Sensor
#define AI_FLOW_OIL		20 	///< AI 20: Parker Oil Flow Sensor
#define AI_TEMP_OIL		21 	///< AI 21: Parker Oil Temperature Sensor
#define AI_PRES_OIL_3	22 	///< AI 22: Rexroth Oil Pressure Sensor 3
#define AI_PRES_OIL_2	23 	///< AI 23: Rexroth Oil Pressure Sensor 2

//AO
#define AO_FLOW_VAL_L	2 	///< AO 2: MOOGs Flow Control Valve (Left track)
#define AO_FLOW_VAL_R	3 	///< AO 3: MOOGs Flow Control Valve (Left track)

//DIO
#define DO_ESTOP_SW		0	///< DO 0.0: Enable Soft E-Stop / Disable "
#define DO_COOLER		1	///< DO 0.1: Enable Cooler / Disable "
#define DO_ENC_MODE		2 	///< DO 0.2: Enable Encoder Mode x4 / Enable x1
#define DO_LIGHT		3 	///< DO 0.3: Enable Signal Light / Disable "

#define DI_CHAR_SIG		4	///< DI P0.4: Charger Enabled / Charger Disabled (Charger fully charged)
#define DI_CHAR_PWR 	5	///< DI P0.5: Charger Powered / Charger Off
#define DI_ENC_DIR		6 	///< DI P0.6: Encoder Up / Encoder Down **Channel cannot be changed.
#define DI_ESTOP_HW		7 	///< DI P0.7: E-Stop Not Pressed / E-Stop Pressed

#define DO_SPEED_VAL	18	///< DO P0.18: Enable High Speed Mode / Enable Low Speed Mode
#define DO_MODE_VAL		19	///< DO P0.19: Enable Auto Mode Valve / Disable Auto Mode Valve (Neutral Status)

//DIO (PFI)
#define DIF_MANUAL		4 	///< DI PFI 4: Manual Mode Enabled (Toggle switch) / Manual Mode Disabled
#define DIF_ENC_IND		5 	///< DI PFI 5: Encoder Index Pulse (1 per revolution)
#define DIF_ENC_CLK		6 	///< DI PFI 6: Encoder Pulses
#define DIF_AUTO		7 	///< DI PFI 7: Auto Mode Enabled (Toggle switch) / Auto Mode Disabled



#elif DEVICE == 6221
//PCI 6221
#define ADC_NUM             16	///<number of analog input channel
#define DAC_NUM             2	///<number of analog output channel
#define CNT_NUM             2	///<number of counter channel
#define PWM_NUM             2	///<number of PWM channel
#define ENC_NUM             2	///<number of encoder channel
#define DIO_NUM             24	///<number of all digital channel (i.e. P0.x, P1.x and P2.x pins)
#define PFI_NUM             16	///<number of PFI channel (i.e. P1.x and P2.x pins)
#define DGR_NUM             6	///<number of digital input output group: defined in "config" folder
#define DCH_NUM             4	///<number of digital channel in each group

/********************************************************/
//List of Channels (for testing with PCI6221)
//AI
#define AI_TEMP_SCB		0 	///< AI 0: NI SCB 68A on-board Temperature Sensor
#define AI_TEMP_MOT		1 	///< AI 1: Motor Temperature Sensor
#define AI_VOLT_BAT		2 	///< AI 2: Battery Voltage Sensor

#define AI_PRES_OIL_1	3 	///< AI 16: Rexroth Oil Pressure Sensor 1
#define AI_PRES_OIL_4	4 	///< AI 17: Rexroth Oil Pressure Sensor 4
#define AI_PRES_OIL_5	5 	///< AI 18: Rexroth Oil Pressure Sensor 5
#define AI_PRES_OIL		6 	///< AI 19: Parker Oil Pressure Sensor
#define AI_FLOW_OIL		7 	///< AI 20: Parker Oil Flow Sensor
#define AI_TEMP_OIL		8 	///< AI 21: Parker Oil Temperature Sensor
#define AI_PRES_OIL_3	9 	///< AI 22: Rexroth Oil Pressure Sensor 3
#define AI_PRES_OIL_2	10 	///< AI 23: Rexroth Oil Pressure Sensor 2

//AO
#define AO_FLOW_VAL_L	0 	///< AO 0: MOOGs Flow Control Valve (Left track)
#define AO_FLOW_VAL_R	1 	///< AO 1: MOOGs Flow Control Valve (Left track)

//DIO
#define DO_ESTOP_SW		0	///< DO 0.0: Enable Soft E-Stop / Disable "
#define DO_COOLER		1	///< DO 0.1: Enable Cooler / Disable "
#define DO_ENC_MODE		2 	///< DO 0.2: Enable Encoder Mode x4 / Enable x1
#define DO_LIGHT		3 	///< DO 0.3: Enable Signal Light / Disable "

#define DI_CHAR_SIG		4	///< DI P0.4: Charger Enabled / Charger Disabled (Charger fully charged)
#define DI_CHAR_PWR 	5	///< DI P0.5: Charger Powered / Charger Off
#define DI_ENC_DIR		6 	///< DI P0.6: Encoder Up / Encoder Down **Channel cannot be changed.
#define DI_ESTOP_HW		7 	///< DI P0.7: E-Stop Not Pressed / E-Stop Pressed

//DIO (PFI)
#define DO_SPEED_VAL	0	///< DO P0.18: Enable High Speed Mode / Enable Low Speed Mode
#define DO_MODE_VAL		1	///< DO P0.19: Enable Auto Mode Valve / Disable Auto Mode Valve (Neutral Status)


#define DIF_MANUAL		4 	///< DI PFI 4: Manual Mode Enabled (Toggle switch) / Manual Mode Disabled
#define DIF_ENC_IND		5 	///< DI PFI 5: Encoder Index Pulse (1 per revolution)
#define DIF_ENC_CLK		6 	///< DI PFI 6: Encoder Pulses
#define DIF_AUTO		7 	///< DI PFI 7: Auto Mode Enabled (Toggle switch) / Auto Mode Disabled
#endif






/********************************************************/

/*
NI-DAQ Analogy Subdevice ID
| idx | type
|  00 | Analog input subdevice
|  01 | Analog output subdevice
|  02 | Digital input/output subdevice //P0
|  03 | Unused subdevice
|  04 | Unused subdevice
|  05 | Calibration subdevice
|  06 | Memory subdevice
|  07 | Digital input/output subdevice //PF
|  08 | Unused subdevice
|  09 | Serial subdevice
|  10 | Unused subdevice
|  11 | Counter subdevice
|  12 | Counter subdevice
|  13 | Counter subdevice
*/

#define SUBD_AI		0	///< Analog input subdevice
#define SUBD_AO		1	///< Analog output subdevice
#define SUBD_DIO	2  	///< Digital input/output subdevice for P0.x pins
#define SUBD_CAL	5	///< Calibration subdevice
#define SUBD_MEM	6	///< Memory subdevice
#define SUBD_DIO_F	7  	///< Digital input/output subdevice for P1.x and P2.x pins
#define SUBD_SER	9	///< Serial subdevice
#define SUBD_CNT	11	///< Counter subdevice
#define SUBD_CNT_1	12	///< Counter subdevice
#define SUBD_CNT_2	13	///< Counter subdevice


/********************************************//**
 * @brief The enumeration of counter type
 ***********************************************/
typedef enum {
    ENCx1,
    ENCx4,
} cnt_t;


/********************************************//**
 * @brief The enumeration of digital pins (i.e. input and output)
 ***********************************************/
typedef enum {
    DIN,
    DOUT,
} dio_t;

/********************************************//**
 * @brief The structure to store the desired analog input range and reference defined in the "config" file.
 ***********************************************/
typedef struct {
    int range;
    int	reference;
} ai_t;

/********************************************//**
 * @brief The structure to store digital signal data
 ***********************************************/
typedef struct {
    unsigned	dio_conf; ///<input/output configuration mask
} digital_t;

/********************************************//**
 * @brief The structure to store encoder signal data
 ***********************************************/
typedef struct {
    signed	counter_now; ///< counter value of the buffer
} encoder_t;


/********************************************//**
 * @brief The structure to store all signal data
 ***********************************************/
typedef struct {
    digital_t	dio;
    digital_t	dio_f;
    encoder_t 	enc[ENC_NUM];
} io_board_t;

/********************************************//**
 * @brief Class for NI DAQ configuration functions and protected variables
 *
 * This class is a parent(i.e. base) class of pciAnalogy class in pciAnalogy.h \n
 * All pciBase class functions are declared with virtual keyword.\n
 * Therefore, the program go look and see if there are any more-derived versions of the function available.\n
 * The functions are re-defined in pciAnalogy class, and the function defined in pciAnalogy can be called from pciBase->function.\n
 *
 ***********************************************/
class pciBase {
public:

    //Constructor & destructor
    pciBase(const char * device_name, const char * config_file = 0);
    virtual ~pciBase();

    //Open & close device
    virtual int     Open() = 0;
    virtual void    Close() = 0;

    // Analog Input:
    virtual int     ReadAD(int ch, double &phys, unsigned short &raw) = 0;

    // Analog Output:
    virtual int     WriteDA(int ch, double phys) = 0;

    // Digital I/O:
    virtual int     SetDIO(int ch_group, unsigned mode) = 0;
    virtual int     ReadDI(int subdevice, int channel, bool &value) = 0;
    virtual int     WriteDO(int subdevice, int channel, bool &value) = 0;

    // Counters:
    virtual int 	StartCNT(int subdevice, int channel_pfi) = 0;
    virtual int 	ReadCNT(int subdevice, int channel_pulse, signed &counter, int freq, double &rpm) = 0;

    // etc...
    void Conf_cnt_dio(void);
    void Reset_IO();

    bool boardIsOpen;			///<status of board: set to TRUE when the loading procedure of the device is completed

    ai_t    ai_conf[ADC_NUM];	///<an array to store the configuration data for AI configuration
    dio_t   dio_grp_conf[DGR_NUM];	///<an array to store the configuration data for DIO groups
    cnt_t   cnt_conf[CNT_NUM];		///<an array to store the configuration data for counters


protected:
    const char      * dev_name;		///<protected device name
    io_board_t      io_board;		///<structure to store data for logging and debugging

    void    read_config(const char * config_file);

};

#endif /* PCIBASE_H_ */

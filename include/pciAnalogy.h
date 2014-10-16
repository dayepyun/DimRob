/** @file pciAnalogy.h
 *  @brief Declaration of pciAnalogy class implementation
 *
 *	This file declares pciAnalogy class which includes the virtual functions defined in pciBase class.\n
 *	The functions are for commanding the DAQ features via analogy driver.\n
 *
 *  @author R. Pyun
 *
 *
 */

#ifndef PCIANALOGY_H_
#define PCIANALOGY_H_

#include <analogy/analogy.h>
#include "pciBase.h"

/********************************************//**
 * @brief Class for NI DAQ related functions and variables
 *
 *
 * This class is a child(i.e. derived) class of pciBase class. \n
 * All pciBase class members are private members of pciAnalogy class. \n
 * Functions defined with virtual are accessible by pciBase class. \n
 * Therefore, in the main file, the user can construct a pciBase class and call the data acquisition functions from the pciAnalogy class.\n
 *
 ***********************************************/
class pciAnalogy : public pciBase 
{
public:

	//Constructor & destructor
    pciAnalogy(const char * device_name, const char * config_file = 0);
    virtual ~pciAnalogy();

    //Open & close device
    virtual int     Open();
    virtual void    Close();

    // Analog Input:
    virtual int     ReadAD(int ch, double &phys, unsigned short &raw);

    // Analog Output:
    virtual int     WriteDA(int ch, double phys);

    // Digital I/O:
    virtual int     SetDIO(int ch_group, unsigned mode);

    virtual int     ReadDI(int subdevice, int channel, bool &value);
    virtual int     WriteDO(int subdevice, int channel, bool &value);


    // Counters:
    virtual int     StartCNT(int subdevice, int channel_pfi);
    virtual int     ReadCNT(int subdevice, int channel_pulse, signed &counter, int freq, double &rpm);

    // etc...
    void    printDeviceInfo(a4l_desc_t *dsc);
    int     rawToDouble(int ch, a4l_chinfo_t *chan, a4l_rnginfo_t *rng, double *dst, void *src);

private:

    a4l_desc_t      dsc;        ///<Structure containing device-information useful to users
    a4l_sbinfo_t    *info; 	///<Structure containing sub-device information

    // used by ADC
    a4l_chinfo_t    *ai_chinfo[ADC_NUM];	///<Structure containing analog input sub-device channel information
    a4l_rnginfo_t   *ai_rnginfo[ADC_NUM];	///<Structure containing analog input sub-device range information

    // used by DAC
    a4l_chinfo_t    *ao_chinfo[DAC_NUM];	///<Structure containing analog output sub-device channel information
    a4l_rnginfo_t   *ao_rnginfo[DAC_NUM];	///<Structure containing analog output sub-device range information

    double CALIB[4][4]; ///< 2D array storing the ADC calibration coefficient (DAQ specific)
    double CALIB_AO[2]; ///< an array storing the DAC calibration coefficient (DAQ specific)

};

#endif /* PCIANALOGY_H_ */

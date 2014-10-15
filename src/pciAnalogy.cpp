/** @file pciAnalogy.cpp
 *  @brief Perform application features on DAQ card (e.g. acquire or generate signals)
 *
 *	All the functions to perform data acquisition features on the DAQ card are implemented in this file (e.g. open device, close device, read analog input, write analog output, read digital input, write digital output, etc). \n
 *	These functions can be called in main file to perform the data acquisition.\n
 *
 *
 *  @author HyQ
 *  @author R. Pyun
 *
 */

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <posix/sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <rtdm/rtdm.h>

#include "pciAnalogy.h"
#include "utils.h"
#include "ksrc.h"
#include "math.h"

/********************************************//**
 *  @brief Constructor of pciAnalogy
 *
 *
 *  @param[in] device_name	device name
 *	@param[in] config_file	configuration file path
 ***********************************************/
pciAnalogy::pciAnalogy(const char * device_name, const char * config_file)
:pciBase(device_name,config_file)
{
}

/********************************************//**
 *  @brief Destructor of pciAnalogy class.
 ***********************************************/
pciAnalogy::~pciAnalogy()
{
    free(dsc.sbdata);
}

/********************************************//**
 *  @brief Open an Analogy device and perform configuration setup.
 *
 *  1. Open an Analogy device.\n
 *  2. Fill the descriptor.\n
 *	3. Get channel and range information.\n
 *	4. Configure digital input/output pins.\n
 *	5. Reset all analog output and digital output to 0.\n
 *
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::Open()
{
    int ret = 0;
    a4l_insn_t insn;
    unsigned int data_f[10];

    boardIsOpen = false;

    //1. Open an Analogy device
    ret = a4l_open(&dsc, dev_name);
    if (ret < 0) {
        printf( "pciAnalogy::Open : a4l_open %s failed (ret=%d)\n", dev_name, ret);
        return boardIsOpen;
    }

    //dynamically allocate memory and sets the first "dsc.sbsize" bytes of the area starting at "dsc.sbdata" to zero
    dsc.sbdata = malloc(dsc.sbsize);
    bzero(dsc.sbdata, dsc.sbsize);
    if (dsc.sbdata == NULL) {
        ret = -ENOMEM;
        printf( "pciAnalogy::Open : info buffer allocation failed\n");
        return boardIsOpen;
    }

    //2. Fill the descriptor
    ret = a4l_fill_desc(&dsc);
    if (ret < 0) {
    	printf( "pciAnalogy::Open : a4l_fill_desc failed (ret=%d)\n", ret);
        return boardIsOpen;
    }

    boardIsOpen = true;

#if DEBUG == 1
    // print device, subdevices, and channels information
    printDeviceInfo(&dsc);
    printf("Open NI DAQ PCI%d on %s\n", DEVICE, dev_name);
#endif


    // 3. get Analog Input channels information
    for (int ch=0; ch<ADC_NUM; ch++) {

        ret = a4l_get_chinfo(&dsc, SUBD_AI, ch, &ai_chinfo[ch]);
        if (ret < 0) {
        	printf(
                    "pciAnalogy::Open : info for channel %d on subdevice %d not available (ret=%d)\n",
                    ch, SUBD_AI, ret);
        }

        ret = a4l_get_rnginfo(&dsc, SUBD_AI, ch, ai_conf[ch].range, &ai_rnginfo[ch]); //AIN_RNG
        if (ret < 0) {
        	printf( "pciAnalogy::Open : failed to recover range descriptor\n");
        }

		#if DEBUG == 1
        	printf( "pciAnalogy::Open : ain ch info %d %d\n", ai_chinfo[ch]->nb_bits, ai_chinfo[ch]->nb_rng);
        	printf( "pciAnalogy::Open : ain ch range [%ld:%ld]\n", ai_rnginfo[ch]->min, ai_rnginfo[ch]->max);
		#endif

    }

    // 3. get Analog Output channels information
    for (int ch=0; ch<DAC_NUM; ch++) {

        ret = a4l_get_chinfo(&dsc, SUBD_AO, ch, &ao_chinfo[ch]);
        if (ret < 0) {
            printf(
                    "pciAnalogy:: info for channel %d on subdevice %d not available (ret=%d)\n",
                    ch, SUBD_AO, ret);
        }

        ret = a4l_get_rnginfo(&dsc, SUBD_AO, ch, 0, &ao_rnginfo[ch]);
        if (ret < 0) {
            printf( "pciAnalogy:: failed to recover range descriptor\n");
        }

		#if DEBUG == 1
        	printf( "pciAnalogy::Open : aout ch info %d %d\n", ao_chinfo[ch]->nb_bits, ao_chinfo[ch]->nb_rng);
        	printf( "pciAnalogy::Open : aout ch range [%ld:%ld]\n", ao_rnginfo[ch]->min, ao_rnginfo[ch]->max);
		#endif

    }

    /********************************************
     *  ADC calibration equation (i.e. equation for analog input)
     *  f(x) = a0 + a1*x + a2*x² + a3*x³
     *
     *
     *  CALIB[i][k]: ADC calibration coefficient (i.e. a0, a1, a2, and a3 in the above equation)
     *  i : range number (AIN_RANGE) (0 == +-10V, 1 == +-5V, 2 == +-1V, 3 == +- 0.2V)
     *  k : coefficient number (a0, a1, a2, a3)
     *
     *  DAC calibration equation (i.e. equation for analog output)
     *	f(x) = b0 + b1*x
     *
     *	CALIB_AO[j]: DAC calibration coefficient (i.e. b0 and b1 in the above equation)
     *	j : coefficient number (b0, b1)
     *
     *  Coefficient was acquired using LabView with DAQmx on Windows
     ***********************************************/
    #if DEVICE == 6229
        //coefficient for PCI 6229
        CALIB[0][0] = -1.00109356131773E-02;
        CALIB[0][1] = 3.24470802928744E-04;
        CALIB[0][2] = 4.39948019641884E-14;
        CALIB[0][3] = -1.70094672442487E-18;
        CALIB[1][0] = -5.13731776347320E-03;
        CALIB[1][1] = 1.61940404194871E-04;
        CALIB[1][2] = 2.19574024788867E-14;
        CALIB[1][3] = -8.48926922179182E-19;
        CALIB[2][0] = -1.25913057438975E-03;
        CALIB[2][1] = 3.24574922180090E-05;
        CALIB[2][2] = 4.40089194311601E-15;
        CALIB[2][3] = -1.70149254025153E-19;
        CALIB[3][0] = -4.89075616930470E-04;
        CALIB[3][1] = 6.49178029681627E-06;
        CALIB[3][2] = 8.80216604931854E-16;
        CALIB[3][3] = -3.40313283410603E-20;

        CALIB_AO[0] = 4.74546575546264;
        CALIB_AO[1] = 3235.10693359375;

    #elif DEVICE == 6221
        //coefficient for PCI 6221
        CALIB[0][0] =  3.33979097137085E-02;
        CALIB[0][1] =  3.23987235245867E-04;
        CALIB[0][2] =  3.52197609256684E-15;
        CALIB[0][3] =  3.46760270517253E-18;
        CALIB[1][0] =  1.61985579530617E-02;
        CALIB[1][1] =  1.61720785341747E-04;
        CALIB[1][2] =  1.75802216162167E-15;
        CALIB[1][3] =  1.73088125619548E-18;
        CALIB[2][0] =  2.47594639171580E-03;
        CALIB[2][1] =  3.24149931993417E-05;
        CALIB[2][2] =  3.52374472414510E-16;
        CALIB[2][3] =  3.46934403205381E-19;
        CALIB[3][0] =  -2.84013547464310E-04;
        CALIB[3][1] =  6.48141922050094E-06;
        CALIB[3][2] =  7.04577250495203E-17;
        CALIB[3][3] =  6.93699762748694E-20;

        CALIB_AO[0] = -2.37451148033142;
        CALIB_AO[1] = 3.23575708007812E+03;

    #endif

	// route PFI channels to static digital input/output
	for(int i = 0; i < PFI_NUM; i++)
	{
	  insn.type      = A4L_INSN_CONFIG;
	  insn.idx_subd  = SUBD_DIO_F;
	  insn.chan_desc = i;
	  insn.data_size = sizeof(data_f[0])*2;
	  data_f[0] = A4L_INSN_CONFIG_SET_ROUTING ;
	  data_f[1] = 16;// NI_PFI_OUTPUT_PFI_DO = 16 in ni_mio.h;
	  insn.data      = data_f;

	  ret = a4l_snd_insn(&dsc, &insn);
	  if (ret < 0)
		  printf("re-route PFI channels: routing of channel %d failed (ret=%d)\n",i,ret);
	}

    // 4. Configuration of digital channel to input or output
	// (based on the user-defined setting in config/pci6229conf.ini DIO)
    Conf_cnt_dio();

    // 5. Reset analog output to 0V and digital output 0
    Reset_IO();

    return boardIsOpen;
}

/********************************************//**
 *  @brief Reset analog and digital output to 0 and Close the Analogy device.
 *
 *  @return void
 ***********************************************/
void pciAnalogy::Close()
{
    Reset_IO();
    a4l_close(&dsc);
    boardIsOpen = false;

#if DEBUG == 1
    printf("Close %s\n", dev_name);
#endif

}

/********************************************//**
 *  @brief Read analog input.
 *
 *  1. Read analog input of channel with range and reference defined by AIN_RNG and AIN_CONFIG.\n
 *  2. Convert rawdata into voltage value using the calibration coefficient.\n
 *
 *
 *	@param[in] channel 			channel to be read
 *	@param[out] physical_value	input voltage value converted from raw value
 *	@param[out] raw 			rawdata in a 16-bit integer: range [0,(2¹⁶-1)]
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::ReadAD(int channel, double &physical_value, unsigned short &raw)
{
    int ret = 0;
    int tmp = a4l_sizeof_chan(ai_chinfo[channel]);
    signed long int raw_s;

    //read analog input
    //PACK(a, b, c) ==  (CHAN(a) | RNG(b) | AREF(c))
 	//Channel + range + reference
    ret = a4l_sync_read(&dsc, SUBD_AI, PACK(channel,ai_conf[channel].range,ai_conf[channel].reference), 0, &raw, tmp);

    if (ret < 0) {
    	printf( "pciAnalogy::ReadDA : a4l_sync_read failed (ret=%d)\n", ret);
        return ret;
    }

    raw_s = raw-32768; //signed type of rawdata: range [-32768,32767]

    //convert rawdata into voltage data (the calibration coefficient is used here)
    ret = rawToDouble(channel, ai_chinfo[channel], ai_rnginfo[channel], &physical_value, &raw_s);

    if ( ret < 0 ) {
    	printf( "pciAnalogy::ReadDA : data conversion failed (ret=%d)\n", ret);
        return ret;
    }

#if DEBUG == 1
    printf("AI channel %d : range [%ld,%ld] ### : raw %d -> phys %g\n",
           channel,
           ai_rnginfo[channel]->min, ai_rnginfo[channel]->max, raw, physical_value);
#endif

    return ret;
}

/********************************************//**
 *  @brief Write analog output.
 *
 *  1. Write analog output of channel\n
 *  2. Convert rawdata into voltage value using the calibration coefficient.\n
 *
 *	@param[in] channel 			channel to be read
 *	@param[in] physical_value	output voltage value
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::WriteDA(int channel, double physical_value)
{
    int ret = 0;
    unsigned short raw_convert;

	//calibrated value with NI coefficient
    raw_convert=round((physical_value*CALIB_AO[1])+CALIB_AO[0]+32768);

    ret = a4l_sync_write(&dsc, SUBD_AO, CHAN(channel), 0, &raw_convert, sizeof(raw_convert));

    if ( ret < 0) {
    	printf( "pciAnalogy::WriteDA : a4l_sync_write failed (ret=%d)\n", ret);
        return ret;
    }


#if DEBUG == 1
	printf("AO channel %d : range [%ld,%ld] ### : raw %d-> phys %g \n",
           channel,
           ao_rnginfo[channel]->min, ao_rnginfo[channel]->max, raw_convert, physical_value);
#endif

    return ret;
}


/********************************************//**
 *  @brief Configure digital channel group to digital input or digital output by pre-defined configuration.
 *
 * 	@param[in] ch_group 	a group of channel to be configured
 *	@param[in] direction	0 for digital input and 1 for digital output
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::SetDIO(int ch_group, unsigned direction)
{
    int ret = 0;
    unsigned mask = 0, mask_f =0;

    digital_t * dio = &io_board.dio;
    digital_t * dio_f = &io_board.dio_f;

    if(ch_group < (DGR_NUM-(PFI_NUM/DCH_NUM))) //for P0 channels
    {
    	//for every channel in a group
		for (int j=(ch_group*DCH_NUM); j<((ch_group*DCH_NUM)+DCH_NUM); j++)
		{
    		  ret = a4l_config_subd(&dsc, SUBD_DIO, direction,j);
    		  mask = mask | (direction <<j);
    	}
	    dio->dio_conf = (dio->dio_conf) | mask; //store configuration bit mask(0:input 1:output)
    }
    else if(ch_group < DGR_NUM) //for P1 and P2 channels (== PFI channels)
    {
    	//for every channel in a group for P1 and P2 pins
    	//subdevice == SUBD_DIO_F, and channel starts from 0 again
		for (int j=((ch_group-(DGR_NUM-(PFI_NUM/DCH_NUM)))*DCH_NUM); j<(((ch_group-(DGR_NUM-4))*DCH_NUM)+DCH_NUM); j++)
		{
  		  ret = a4l_config_subd(&dsc, SUBD_DIO_F, direction,j);
		  mask_f = mask_f | (direction <<j);
    	}
	    dio_f->dio_conf = (dio_f->dio_conf ) | mask_f; //store configuration bit mask(0:input 1:output)
    }
    else
    	ret = -EINVAL;

    if ( ret < 0) {
    	printf( "DIO configuration of group %d failed (ret=%d)\n", ch_group, ret);
        return ret;
    }


#if DEBUG == 1
    printf( "Set DIO channel group %d : %s\n", ch_group, direction == 1 ? "OUT" : "IN");
#endif

    return ret;
}


/********************************************//**
 *  @brief Read digital input.
 *
 *	The configuration of each channel is performed when the device is opened. \n
 *	If a channel is configured to a digital input, the value indicates the input status of the channel. \n
 *	If a channel is configured to a digital output, the error is returned.\n
 *
 *	@param[in] subdevice 		subdevice (either SUBD_DIO or SUBD_DIO_F)
 * 	@param[in] channel 			channel to read
 *	@param[in/out] value	the value read (range: [0,1]=[OFF,ON])
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::ReadDI(int subdevice, int channel, bool &value)
{
    int		ret = 0;
    unsigned mask = 0, buffer = 0, config = 0;
    a4l_chinfo_t *chinfo;
	digital_t * dio;

    if(subdevice == SUBD_DIO)
	{
    	dio =  &io_board.dio;
	}
    else
    {
    	dio =  &io_board.dio_f;
    }

	ret= a4l_get_chinfo(&dsc, subdevice, channel, &chinfo);
	if(ret < 0)
	{
		printf("DIO: info for channel %d on subdevice %d not available (err=%d)\n", channel, subdevice, ret);
	    return ret;
	}
	else
	{
		mask = 0x0001 << channel;

		if(((dio->dio_conf) & mask) != 0)//if the channel is not digital input
		{
			printf("Channel %d on subdevice %d is not assigned as input channel\n", channel, subdevice);
		    return ret;
		}
		else
		{
			ret = a4l_sync_dio(&dsc, subdevice, &mask, &buffer);

			if (ret < 0) {
				printf("insn_bits: a4l_sync_dio() failed (err=%d)\n", ret);
				return ret;
			}

			value = buffer & mask;
		}

	}


#if DEBUG == 1
		printf( "DIO subdevice %d: channel %d is [Digital %s] %s\n",  subdevice, channel,  ((dio->dio_conf) & mask) == 0 ? "Input" : "Output", value == 1 ? "ON" : "OFF");
#endif
    return ret;
}


/********************************************//**
 *  @brief Write digital output.
 *
 *	The configuration of each channel is performed when the device is opened. \n
 *	If a channel is configured to a digital input, the error is returned. \n
 *	If a channel is configured to a digital output, the value indicates the desired output value.\n
 *
 *	@param[in] subdevice 		subdevice (either SUBD_DIO or SUBD_DIO_F)
 * 	@param[in] channel 			channel to write
 *	@param[in/out] value	the value written (range: [0,1]=[OFF,ON])
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::WriteDO(int subdevice, int channel, bool &value)
{
    int		ret = 0;
    unsigned mask = 0, buffer = 0, config = 0;
    a4l_chinfo_t *chinfo;
	digital_t * dio;

    if(subdevice == SUBD_DIO)
	{
    	dio =  &io_board.dio;
	}
    else
    {
    	dio =  &io_board.dio_f;
    }

	ret= a4l_get_chinfo(&dsc, subdevice, channel, &chinfo);
	if(ret < 0)
	{
		printf("DIO: info for channel %d on subdevice %d not available (err=%d)\n", channel, subdevice, ret);
	    return ret;
	}
	else
	{

		mask = 0x0001 << channel;
		buffer = value << channel;

		if(((dio->dio_conf) & mask ) == 0)//if the channel is not digital output
		{
			printf("Channel %d on subdevice %d is not assigned as output channel\n", channel, subdevice);
		    return ret;
		}
		else
		{
			ret = a4l_sync_dio(&dsc, subdevice, &mask, &buffer);


			if (ret < 0) {
				printf("insn_bits: a4l_sync_dio() failed (err=%d)\n", ret);
				return ret;
			}

			value = buffer & mask;

		}

	}


#if DEBUG == 1
		printf( "DIO subdevice %d: channel %d is [Digital %s] %s\n",  subdevice, channel,  ((dio->dio_conf) & mask) == 0 ? "Input" : "Output",value == 1 ? "ON" : "OFF");
#endif
    return ret;
}
/********************************************//**
 *  @brief Set-up counter
 *
 *
 *	@param[in]	subdevice		subdevice of counter (only SUBD_CNT or SUBD_CNT_1 for NI PCI 6221 and 6229 card)
 *	@param[in]	channel_pfi	PFI pin which reads the pulse signal, use PFI6 for counter 0 and PFI7 for counter 1 (e.g. CH A from the encoder or CLK from the converter chip)
 *  @return		0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::StartCNT(int subdevice, int channel_pfi)
{
		int ret;
		lsampl_t counter_mode, gate_setup;	// counter setup bits
		unsigned int data[3];	// config data
		a4l_insn_t insn;	// instruction instance
		unsigned initial_count = 0;
	    encoder_t * enc = &io_board.enc[subdevice-11];

	    if((subdevice != SUBD_CNT) && (subdevice != SUBD_CNT_1))
	    {
			printf("Not a valid counter subdevice for counter function. \n");
			return -EINVAL;
	    }

		// reset counter
		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0]);
		data[0]      = A4L_INSN_CONFIG_RESET;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup :  reset counter failed (ret=%d)\n",ret);
			return ret;
		}


		//set source pulse input
		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0])*3;
		data[0]      = A4L_INSN_CONFIG_SET_CLOCK_SRC;
		data[1]      = NI_GPCT_PFI_CLOCK_SRC_BITS(channel_pfi);//NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS//e.g. defined in ni_tio.h Gi_Source_Bit in 4-63 in DAQ-STC manual 8(?) works for 0,1,2
		data[2]		 = 0;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup :  set other source failed (ret=%d)\n",ret);
			return ret;
		}

		//disable gate 1 and 2
		gate_setup = NI_GPCT_DISABLED_GATE_SELECT;//NI_GPCT_PFI_GATE_SELECT(12)//CR_EDGE for rising edge

		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0])*3;
		data[0]      = A4L_INSN_CONFIG_SET_GATE_SRC;
		data[1]      = 0; //gate index (0 or 1)
		data[2]      = gate_setup;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup :  set other source failed (ret=%d)\n",ret);
			return ret;
		}

		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0])*3;
		data[0]      = A4L_INSN_CONFIG_SET_GATE_SRC;
		data[1]      = 1; //gate index (0 or 1)
		data[2]      = gate_setup;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup :  set other source failed (ret=%d)\n",ret);
			return ret;
		}

		//set counter mode
		counter_mode = NI_GPCT_COUNTING_MODE_NORMAL_BITS;
		// count up and down determined by Up/Down signal
		counter_mode |= NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS; //P0.6 to determine up down

		//various counter mode options available in ksrc.h file
		//below are some of the examples.
		//			//Don't alternate the reload source between the load a and load b registers.
		//	 		counter_mode |= NI_GPCT_RELOAD_SOURCE_FIXED_BITS;
		//			// start and stop on gate bit == only count when gate is 1
		//	 	  	counter_mode |= NI_GPCT_EDGE_GATE_STARTS_STOPS_BITS;
		//	 	  	// don't disarm on terminal count or gate signal
		//	 	  	counter_mode |= NI_GPCT_NO_HARDWARE_DISARM_BITS;
		//	 	  	//reload at gate input
		//	 	  	counter_mode |= NI_GPCT_LOADING_ON_GATE_BIT;


		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0])*2;
		data[0]      = A4L_INSN_CONFIG_SET_COUNTER_MODE;
		data[1]      = counter_mode;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup: set counter mode failed (ret=%d)\n",ret);
			return ret;
		}

		//set initial counter to 0
		ret = a4l_sync_write(&dsc, subdevice, 0, 0, &initial_count, sizeof(initial_count));
		if(ret <0){
			fprintf(stderr,"counter setup: write initial counter value as 0 (ret=%d)\n",ret);
			return ret;
		}

		// arm the counter
		insn.type      = A4L_INSN_CONFIG;
		insn.idx_subd  = subdevice;
		insn.chan_desc = 0;
		insn.data_size = sizeof(data[0])*2;
		data[0]      = A4L_INSN_CONFIG_ARM;
		data[1]      = NI_GPCT_ARM_IMMEDIATE;
		insn.data      = data;

		if((ret = a4l_snd_insn(&dsc, &insn))<0){
			fprintf(stderr,"counter setup : arm counter failed (ret=%d)\n",ret);
			return ret;
		}

		//initialize counter
		enc->counter_now = 0;

		return ret;
}

/********************************************//**
 *  @brief Read the velocity of the motor using encoder with quadrature clock converter chip
 *
 *  The mode of encoder is defined in conf/pci6229conf.ini file.
 *
 *	@param[in]	subdevice		subdevice of counter (only SUBD_CNT or SUBD_CNT_1 for NI PCI 6221 and 6229 card)
 *	@param[in]	channel_pulse	PFI pin which reads the pulse signal, use PFI6 for counter 0 and PFI7 for counter 1 (e.g. CH A from the encoder or CLK from the converter chip)
 *	@param[out]	counter			the counter value - size of int16
 *	@param[in]	freq			the frequency of thread
 *	@param[out]	rpm				the calculated RPM value
 *
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::ReadCNT(int subdevice, int channel_pulse, signed &counter, int freq, double &rpm)
{
    int ret = 0;
    int counter_num;
    double count_rev = 0; //number of count in 1 revolution (encoder specific), 200 for our encoder
    encoder_t * enc = &io_board.enc[subdevice-11];
    signed precounter = enc->counter_now;
    int mode = cnt_conf[subdevice-11];

    if((subdevice == SUBD_CNT) || (subdevice == SUBD_CNT_1))
    {
        ret = a4l_sync_read(&dsc, subdevice, 0, 0, &counter, sizeof(counter));
        counter_num = subdevice-11;

    	if ( ret < 0) {
            return ret;
        }

    }
    else
    {
		printf("Not a valid counter subdevice for counter function. \n");
		return -EINVAL;
    }

	enc->counter_now = counter;

	if((precounter > 0) && (counter < 0))
	{
		counter += 4294967296;
	}
	else if((precounter < 0) && (counter > 0))
	{
		counter -= 4294967296;
	}

	if(mode == 1)
	{
		count_rev = 800.0;
	}
	else if(mode == 0)
	{
		count_rev = 200.0;
	}
	rpm = (counter-precounter)*60*freq/count_rev;


	#if DEBUG == 1
    	printf("Read counter %d: steps=%d, rpm = %g RPM\n", counter_num, counter, rpm);
	#endif


    return ret;

}

/********************************************//**
 *  @brief A function to print all device information
 *
 *	Provide more detailed information about all subdevices and channels of a device.
 *
 *  @param[in] dsc Device descriptor
 *  @return void
 ***********************************************/
void pciAnalogy::printDeviceInfo(a4l_desc_t *dsc)
{

  int i,j;
  int ret;

  // print out information about this device
  printf("   Board Name             : %s\n",dsc->board_name);
  printf("   #Subdevices            : %d\n",dsc->nb_subd);
  printf("   Input Subdevice Index  : %d\n",dsc->idx_read_subd);
  printf("   Output Subdevice Index : %d\n",dsc->idx_write_subd);
  printf("   Data Buffer Size       : %d\n",dsc->sbsize);


  // get information about each of the subdevices
  for (i=1; i<=dsc->nb_subd; ++i) {

	  ret = a4l_get_subdinfo(dsc,i-1,&info);

    if (ret < 0) {
      printf("ni_test: a4l_get_subdinfo (ID=%d on %s) failed (ret=%d)\n",i-1,dev_name, ret);
    }

    printf("           Subdevice ID = %d\n",i-1);

    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_UNUSED) {
      printf("             Subdevice is unused\n");
      printf("\n");
      continue;
    }
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_AI)
      printf("             Subdevice is analog input\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_AO)
      printf("             Subdevice is analog output\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_DI)
      printf("             Subdevice is digital input\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_DO)
      printf("             Subdevice is digital output\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_DIO)
      printf("             Subdevice is digital input/output\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_COUNTER)
      printf("             Subdevice is counter\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_TIMER)
      printf("             Subdevice is timer\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_MEMORY)
      printf("             Subdevice is memory, EEPROM, or DPRAM\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_CALIB)
      printf("             Subdevice is calibration DAC\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_PROC)
      printf("             Subdevice is processor or DSP\n");
    if ((info->flags&A4L_SUBD_TYPES) == A4L_SUBD_SERIAL)
      printf("             Subdevice is serial I/O\n");
    if (info->flags & A4L_SUBD_CMD)
      printf("             Subdevice can handle command (asynchronous acquisition)\n");
    if (info->flags & A4L_SUBD_MMAP)
      printf("             Subdevice can do mmap operations\n");

    printf("             Status               %ld\n",info->status);
    printf("             Number of Channels   %d\n",info->nb_chan);


    // get channel information
    for (j=1; j<=info->nb_chan; ++j) {
      a4l_chinfo_t *chan_info;

      ret = a4l_get_chinfo(dsc,i-1,j-1,&chan_info);
      if (ret < 0) {
        printf("a4l_get_chinfo failed (ret=%d)\n", ret);
      }

      printf("                 %2d.Channel: #Bits = %d  #Ranges = %d  Flags = 0x%lx\n",
	     j-1,chan_info->nb_bits,
	     chan_info->nb_rng,chan_info->chan_flags);
    }

    printf("\n");

  }

}

/********************************************//**
 *  @brief Convert rawdata acquired from ADC to voltage value (in double)
 *
 *	The function is almost same as a4l_rawtod function from Analogy.\n
 *	Only the conversion part of the code has been modified to convert \n
 *	rawdata into voltage value using the calibration coefficient in the third order polynomial equation.
 *
 *	@param[in] 	ch		Channel
 *  @param[in] 	chan 	Channel descript
 *  @param[in] 	rng		Range descriptor
 *  @param[out] dst 	Voltage value
 *  @param[in] 	src 	Rawdata acquired by ADC
 *  @return 	0 on success. Otherwise: error code in errno.h\n
 ***********************************************/
int pciAnalogy::rawToDouble(int ch, a4l_chinfo_t * chan, a4l_rnginfo_t * rng, double *dst, void *src)
{
	//copied from a4l_rawtod function
	int size;
	lsampl_t_s tmp;

	/* Temporary data accessor */
	lsampl_t_s(*datax_get) (void *);

	/* Basic checking */
	if (rng == NULL || chan == NULL)
		return -EINVAL;

	/* Find out the size in memory */
	size = a4l_sizeof_chan(chan);

	/* Get the suitable accessor */
	switch (size) {
	case 4:
		datax_get = data32_get_s;
		break;
	case 2:
		datax_get = data16_get_s;
		break;
	case 1:
		datax_get = data8_get_s;
		break;
	default:
		return -EINVAL;
	};

	/* Properly retrieve the data */
	tmp = datax_get(src);

	/* Perform the conversion */
	*dst =  CALIB[ai_conf[ch].range][0]+CALIB[ai_conf[ch].range][1]*tmp+CALIB[ai_conf[ch].range][2]*(tmp^2)+CALIB[ai_conf[ch].range][3]*(tmp^3);

      return 0;
}

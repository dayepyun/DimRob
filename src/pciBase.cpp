/** @file pciBase.cpp
 *  @brief Perform basic features on DAQ card (e.g. configure or reset DAQ card)
 *
 *	The auxiliary functions to handle the following functionalities.\n
 *	-Reading the configuration file.\n
 *	-Configuring the digital channels to input or output.\n
 *	-Resetting analog output and digital output to 0 are implemented.\n
 *
 *  @author R. Pyun
 *
 */


#include <math.h>
#include <assert.h>


#include "iniparser.h"
#include "pciBase.h"


/********************************************//**
 *  @brief Constructor of pciBase
 *
 *
 *	@param[in] device_name	device name
 *	@param[in] config_file	configuration file path
 ***********************************************/
pciBase::pciBase(const char * device_name, const char * config_file)
{
    dev_name = NULL;

    if (device_name) {
        dev_name = strdup(device_name);
    }

    if (config_file) {
        // overwrite dev_name
        read_config(config_file);
    }

    assert(dev_name);

}


/********************************************//**
 *  @brief Destructor of pciBase class.
 ***********************************************/
pciBase::~pciBase()
{
    free((void*)dev_name);
}

/********************************************//**
 *  @brief Read the configuration in config/pci6229conf.ini
 *
 *
 *	@param[in]	configuration file path
 *  @return 	void
 ***********************************************/
void pciBase::read_config(const char * conf)
{
    // digital IO are configurable In Out group by 4
    int i = 0;
    char * str_value = 0;
    char str_key[128];

    // load configuration file
    dictionary * ini = iniparser_new((char*)conf);
    if (ini==NULL) {
    	printf( "cannot parse file: %s\n", conf);
        return;
    }
    else {
        printf( "parse file: %s\n", conf);
    }
    iniparser_dump(ini, stderr);

    str_value = iniparser_getstring(ini, (char*)"pci_conf:device", NULL);
    if (str_value) {
        dev_name = strdup(str_value);
    }

    sprintf(str_key, "pci_conf:daq_model");

    // store configuration for DIO groups into dio_grp_conf[] array
    for (i = 0; i<DGR_NUM; i++) {
        snprintf(str_key, sizeof(str_key), "pci_conf:dio_grp_%d", i);
        str_value = iniparser_getstring(ini, str_key, (char*)"IN");
        if (!strcmp(str_value,"IN")) {
            dio_grp_conf[i] = DIN;
        } else {
            dio_grp_conf[i] = DOUT;
        }
    }

    // store range configuration for analog input channels
    for (i = 0; i<ADC_NUM; i++) {
        snprintf(str_key, sizeof(str_key), "pci_conf:ai_rng_%d", i);
        str_value = iniparser_getstring(ini, str_key, (char*)"0");
        if (!strcmp(str_value,"0")) {
        	ai_conf[i].range = 0;
        } else if (!strcmp(str_value,"1")) {
        	ai_conf[i].range = 1;
        } else if (!strcmp(str_value,"2")) {
        	ai_conf[i].range = 2;
        } else if (!strcmp(str_value,"3")) {
        	ai_conf[i].range = 3;
        }
    }

    // store ground configuration for analog input channels
    for (i = 0; i<ADC_NUM; i++) {
        snprintf(str_key, sizeof(str_key), "pci_conf:ai_ref_%d", i);
        str_value = iniparser_getstring(ini, str_key, (char*)"0");
        if (!strcmp(str_value,"0")) {
        	ai_conf[i].reference = 0;
        } else if (!strcmp(str_value,"1")) {
        	ai_conf[i].reference = 1;
        } else if (!strcmp(str_value,"2")) {
        	ai_conf[i].reference = 2;
        }
    }

    // configure counters
    for (i = 0; i<CNT_NUM; i++) {
        snprintf(str_key, sizeof(str_key), "pci_conf:cnt_%d", i);
        str_value = iniparser_getstring(ini, str_key, (char*)"ENCx1");
        if (!strcmp(str_value,"ENCx1")) {
            cnt_conf[i] = ENCx1;
        } else if (!strcmp(str_value,"ENCx4")) {
            cnt_conf[i] = ENCx4;
        }
    }

    iniparser_free(ini);
}

/********************************************//**
 *  @brief Configure counters and digital pins
 *
 *	Configure the digital pins to input or output based on the user-defined configuration in the configuration file, pci6229conf.ini.
 *
 *  @param	void
 *  @return 	void
 ***********************************************/
void pciBase::Conf_cnt_dio(void)
{
    int clk_src;

    assert(boardIsOpen);

    for (int i=0; i<DGR_NUM; i++) {
		SetDIO(i, dio_grp_conf[i]);
    }
}

/********************************************//**
 *  @brief Set analog output and digital output to 0
 *
 *  @return 	void
 ***********************************************/
void pciBase::Reset_IO()
{
	bool value = false;
    unsigned mask = 0;
	digital_t * dio;

    assert(boardIsOpen);

    // Reset DAC Outputs
    for (int c=0; c<DAC_NUM; c++){
        WriteDA(c, 0);
    }

    // Reset digital outputs (pin: P0.x)
    for(int c = 0; c<(DIO_NUM-PFI_NUM); c++)
    {
    	dio =  &io_board.dio;
	mask = 0x0001 << c;
	if(((dio->dio_conf) & mask ) != 0)//if the channel is digital output
	{
		WriteDO(SUBD_DIO,c,value);
	}
    }

    // Reset digital outputs (pin: P1.x and P2.x)
    for(int c = 0; c<PFI_NUM; c++)
    {
	dio =  &io_board.dio_f;
	mask = 0x0001 << c;
	if(((dio->dio_conf) & mask ) != 0)//if the channel is digital output
	{
		WriteDO(SUBD_DIO_F,c,value);
	}

    }

}

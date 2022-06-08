

#ifndef AHT10_H
#define AHT10_H

#include <stdio.h>

typedef unsigned char bool;

#ifndef true
#define true 									(1)
#endif

#ifndef false
#define false 									(0)
#endif

enum
{
	read_Humidity , 
	read_Temperature ,

};

typedef enum _AHT10_I2C_MESSAGE{
	AHT10_i2c_success = 0,
	AHT10_i2c_error,
}AHT10_I2C_MESSAGE;

typedef enum
{
	AHT1x_SENSOR = 0x00,
	AHT2x_SENSOR = 0x01,
}AHTXX_I2C_SENSOR;

/* list of I2C addresses */
#define AHTXX_ADDRESS_X38                 	0x38  //AHT15/AHT20/AHT21/AHT25 I2C address, AHT10 I2C address if address pin to GND
#define AHT10_ADDRESS_X39                 	0x39  //AHT10 I2C address, if address pin to Vcc

#define AHT10_DEVADDR_7BIT					(AHTXX_ADDRESS_X38)		// 7 bit
#define AHT10_DEVADDR_8BIT					(AHT10_DEVADDR_7BIT << 1)

	
/* list of command registers */	
#define AHT1X_INIT_REG                    	0xE1  //initialization register, for AHT1x only
#define AHT2X_INIT_REG                    	0xBE  //initialization register, for AHT2x only
#define AHTXX_STATUS_REG                  	0x71  //read status byte register
#define AHTXX_START_MEASUREMENT_REG       	0xAC  //start measurement register
#define AHTXX_SOFT_RESET_REG              	0xBA  //soft reset register
	
/* calibration register controls */	
#define AHT1X_INIT_CTRL_NORMAL_MODE       	0x00  //normal mode on/off       bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CYCLE_MODE        	0x20  //cycle mode on/off        bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CMD_MODE          	0x40  //command mode  on/off     bit[6:5], for AHT1x only
#define AHTXX_INIT_CTRL_CAL_ON            	0x08  //calibration coeff on/off bit[3]
#define AHTXX_INIT_CTRL_NOP               	0x00  //NOP control, send after any "AHT1X_INIT_CTRL..."
	
/* status byte register controls */	
#define AHTXX_STATUS_CTRL_BUSY            	0x80  //busy                      bit[7]
#define AHT1X_STATUS_CTRL_NORMAL_MODE     	0x00  //normal mode status        bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CYCLE_MODE      	0x20  //cycle mode status         bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CMD_MODE        	0x40  //command mode status       bit[6:5], for AHT1x only
#define AHTXX_STATUS_CTRL_CRC             	0x10  //CRC8 status               bit[4], no info in datasheet
#define AHTXX_STATUS_CTRL_CAL_ON          	0x08  //calibration coeff status  bit[3]
#define AHTXX_STATUS_CTRL_FIFO_ON         	0x04  //FIFO on status            bit[2], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_FULL       	0x02  //FIFO full status          bit[1], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_EMPTY      	0x02  //FIFO empty status         bit[1], no info in datasheet
	
/* measurement register controls */	
#define AHTXX_START_MEASUREMENT_CTRL      	0x33  //measurement controls, suspect this is temperature & humidity DAC resolution
#define AHTXX_START_MEASUREMENT_CTRL_NOP  	0x00  //NOP control, send after any "AHTXX_START_MEASUREMENT_CTRL..."

/* sensor delays */
#define AHTXX_CMD_DELAY          			10      //delay between commands, in milliseconds
#define AHTXX_MEASUREMENT_DELAY  			80      //wait for measurement to complete, in milliseconds
#define AHT1X_POWER_ON_DELAY     			40      //wait for AHT1x to initialize after power-on, in milliseconds
#define AHT2X_POWER_ON_DELAY     			100     //wait for AHT2x to initialize after power-on, in milliseconds
#define AHTXX_SOFT_RESET_DELAY   			20      //less than 20 milliseconds
			
/* misc */			
#define AHTXX_I2C_SPEED_100KHZ   			100000  //sensor I2C speed 100KHz..400KHz, in Hz
#define AHTXX_I2C_STRETCH_USEC   			1000    //I2C stretch time, in usec
#define AHTXX_FORCE_READ_DATA    			true    //force to read data via I2C
#define AHTXX_USE_READ_DATA      			false   //force to use data from previous read
			
#define AHTXX_NO_ERROR           			0x00    //success, no errors
#define AHTXX_BUSY_ERROR         			0x01    //sensor is busy
#define AHTXX_ACK_ERROR          			0x02    //sensor didn't return ACK (not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))
#define AHTXX_DATA_ERROR         			0x03    //received data smaller than expected
#define AHTXX_CRC8_ERROR         			0x04    //computed CRC8 not match received CRC8, for AHT2x only
#define AHTXX_ERROR              			0xFF    //other errors

void AHT10_WriteReg(unsigned char RegAddr, unsigned char* txData , unsigned char length);
int AHT10_ReadReg(unsigned char RegAddr , unsigned char* rxData , unsigned char length);

float    readHumidity(bool readAHT);
float    readTemperature(bool readAHT);
bool     setNormalMode(void);
bool     setCycleMode(void);
bool     setComandMode(void);
bool     softReset(void);
unsigned char  getStatus(void);

void     _readMeasurement(void);
bool     _setInitializationRegister(unsigned char value); 
unsigned char  _readStatusRegister(void);
unsigned char  _getCalibration(void);
unsigned char  _getBusy(bool readAHT);
bool     _checkCRC8(void);

void AHT10_Init(void);

#endif /* AHT10_H */


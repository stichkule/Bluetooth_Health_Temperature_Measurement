//***********************************************************************************
// Include files
//***********************************************************************************
#include "i2c.h"
#include "em_i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************
void I2C_initialize(void){
	I2C_Init_TypeDef I2C_Def;
	I2C_Def.clhr=i2cClockHLRStandard; // Set to use 4:4 low/high duty cycle
	I2C_Def.enable= true; // Enable when initialization done
	I2C_Def.freq= I2C_FREQ_STANDARD_MAX; // Set to standard rate assuring being within I2C specifications
	I2C_Def.master=true; // Set to master mode
	I2C_Def.refFreq=0; // Use currently configured reference clock

	/* Initializing I2C0 */
	/* Output value must be set to 1 to not drive lines low */
	/* Set SCL first, to ensure it is high before changing SDA. */
	GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeWiredAnd, 1);

	/* Toggle I2C SCL 9 times to reset any I2C slave that may require it */
	for(int i=0;i<9;i++)
	{
	GPIO_PinOutClear(I2C0_SCL_Port, I2C0_SCL_Pin);
	GPIO_PinOutSet(I2C0_SCL_Port, I2C0_SCL_Pin);
	}

	/* Route SDA and SCL functionality to GPIO pins */
	I2C0->ROUTEPEN= I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;
	I2C0->ROUTELOC0 = I2C_ROUTELOC0_SCLLOC_LOC14 | I2C_ROUTELOC0_SDALOC_LOC16;

	I2C_Init(I2C0,&I2C_Def); // Initialize I2C module

	if (I2C0->STATE & I2C_STATE_BUSY) // abort I2C initialization if busy
		{
		        I2C0->CMD = I2C_CMD_ABORT;
		}
}

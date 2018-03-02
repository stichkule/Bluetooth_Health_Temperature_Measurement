//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void cmu_init(void){

	/* enable HFPERCLK for I2C0 */
	CMU_ClockEnable(cmuClock_HFPER, true);

	/*enable low frequency clock for low frequency peripherals*/
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	CMU_ClockEnable(cmuClock_CORE, true);
	if (LETIMER0_Energy_Mode == EM3)
	{
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	//CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
	}
	else CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	/*enabling clocks to the desired peripherals*/

	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
}


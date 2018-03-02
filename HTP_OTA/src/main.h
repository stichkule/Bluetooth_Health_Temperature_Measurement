//***********************************************************************************
// Include files
//***********************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "em_letimer.h"
#include "em_core.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

/*sleep mode levels*/

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

/*sleep mode count state*/

unsigned int sleep_mode_counter[5];

/*LED and LETIMER parameters*/

#define LETIMER0_period                   4.0    /* Temperature measurement period */
#define LETIMER0_LFXO_count               32768
#define LETIMER0_ULFRCO_count             1000
#define LETIMER0_Energy_Mode              EM3    /* Lowest energy mode for LETIMER */

int external_event_status; /* global variable for signaling I2C transfer event */
int temp_measured; /* temperature count of Si7021 */
int temp_MSB; /* MSB of Si7021 temperature measurement */
int temp_LSB; /* LSB of Si7021 temperature measurement */
float temperature; /* converted temperature in deg C */

#define read (1) /* I2C read signal */
#define write (0) /* I2C write signal */
#define slave_address (0x40) /* Si7021 device address */
#define temp_set (15) /* Reference temperature */


//***********************************************************************************
// function prototypes
//***********************************************************************************

void sleep(void);
void blockSleepMode(unsigned int min_energy_mode);
void unblockSleepMode(unsigned int min_energy_mode);
void LETIMER0_Setup(void);
void LED_ON(void);
void LED_OFF(void);
void I2C_read(void);
void Si7021_enable(void);
void Si7021_disable(void);
void measure_temperature();

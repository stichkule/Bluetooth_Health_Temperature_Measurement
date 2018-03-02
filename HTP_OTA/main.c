/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* App headers */
#include "init_app.h"

/* Libraries containing default Gecko configuration values */
#include "em_system.h"
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "gpio.h"
#include "cmu.h"
#include "ustimer.h"
#include "i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************
int external_event_status = 0;
int temp_MSB = 0;
int temp_LSB = 0;
int temp_measured = 0;
float temperature = 0.0;
int connection;

//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/*******************************************************************************
 * # License
 * <b>Copyright 2016 Silicon Laboratories, Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
/* The above Silicon Laboratories IP has been applied, and adhered to by the
 * developer for the following three functions:
 * blockSleepMode()
 * unblockSleepMode()
 * sleep()
 */

void blockSleepMode(unsigned int min_energy_mode)
{
CORE_ATOMIC_IRQ_DISABLE();
sleep_mode_counter[min_energy_mode]++;
CORE_ATOMIC_IRQ_ENABLE();
}

void unblockSleepMode(unsigned int min_energy_mode)
{
CORE_ATOMIC_IRQ_DISABLE();
if (sleep_mode_counter[min_energy_mode] >0)
	sleep_mode_counter[min_energy_mode]--;
else sleep_mode_counter[min_energy_mode] = 0;
CORE_ATOMIC_IRQ_ENABLE();
}

void sleep(void) /* comment sleep function since library functions will be used */
{
/*
if (sleep_mode_counter[EM0] > 0)
	return;
else if (sleep_mode_counter[EM1] > 0)
EMU_EnterEM1();
else if (sleep_mode_counter[EM2] > 0)
EMU_EnterEM2(true);
else if (sleep_mode_counter[EM3] > 0)
EMU_EnterEM3(true);
*/
}

/* Configure LETIMER0 -- UF interrupt for temperature measurement period */

void LETIMER0_Setup(void)
{
LETIMER_Init_TypeDef  LETIMER0_init;
int intFlags;
int COMP0_value;
int LETIMER0_prescaler;
int LFA_freq_div = 0x2; // LFA frequency divider of 2^2 = 4

LETIMER0_init.bufTop = false;  /* COMP1 not loaded with COMP0 */
LETIMER0_init.comp0Top = true; /* COMP0 loaded into CNT when UF */
LETIMER0_init.debugRun = false;
LETIMER0_init.enable = false;
LETIMER0_init.out0Pol = 0;
LETIMER0_init.out1Pol = 1;
LETIMER0_init.repMode = letimerRepeatFree;
LETIMER0_init.ufoa0 = letimerUFOANone;
LETIMER0_init.ufoa1 = letimerUFOANone;

LETIMER_Init(LETIMER0, &LETIMER0_init);

/* Temperature measurement period */

if (LETIMER0_Energy_Mode == EM3)
COMP0_value = LETIMER0_period * LETIMER0_ULFRCO_count; // counter value dependent on UFLRCO in EM3
else
{
LETIMER0_prescaler = LETIMER0_period;
CMU->LFAPRESC0 = CMU->LFAPRESC0 & 0xfffffff0;  //clear first 4 bits of LFAPRESC0 register
CMU->LFAPRESC0 = CMU->LFAPRESC0 | LFA_freq_div; // divide LFA clock by 4
COMP0_value = (LETIMER0_prescaler/(float)(1 << LFA_freq_div)) * LETIMER0_period * (LETIMER0_LFXO_count / (LETIMER0_prescaler));
}
LETIMER0->CNT = COMP0_value; // load COMP0 value
LETIMER_CompareSet(LETIMER0,0,COMP0_value); // set LETIMER in compare-set mode
while((LETIMER0->SYNCBUSY) !=0); // wait until timer is synced

/* Clear LETIMER0 interrupts, if any */
intFlags = LETIMER0->IF;
LETIMER0->IFC = intFlags;

/* enable interrupts on UF */
LETIMER0->IEN = LETIMER_IEN_UF;

/* configure lowest allowable sleep mode for LETIMER0 */
blockSleepMode(LETIMER0_Energy_Mode);

/* Enable interrupts at the NVIC */
NVIC_EnableIRQ(LETIMER0_IRQn);
}

/* IRQ handler for LETIMER0 -- interrupt occurs on UF */

void LETIMER0_IRQHandler(void)
{
int intFlags;
CORE_ATOMIC_IRQ_DISABLE();
intFlags = LETIMER0->IF;
LETIMER0->IFC = intFlags;
external_event_status |= 0x01; // set global variable to signal temperature measurement needs to be taken via I2C
gecko_external_signal(external_event_status);
CORE_ATOMIC_IRQ_ENABLE();
}

/* functions to turn ON/OFF LEDs*/

 void LED_ON(void){
   GPIO_PinOutSet(LED1_port,LED1_pin);
}
 void LED_OFF(void){
    GPIO_PinOutClear(LED1_port,LED1_pin);
 }

 void I2C_read(void){
	 CMU_ClockEnable(cmuClock_I2C0, true); // enable clock to the I2C0 peripheral
	 I2C0->TXDATA = (slave_address << 1) | write; // prepare I2C Tx buffer for writing slave address
	 I2C0->CMD = I2C_CMD_START; // issue write command
	 while ((I2C0->IF & I2C_IF_ACK) ==  0){;} // wait until ACK is received
	 I2C0->IFC = I2C_IFC_ACK; // clear ACK
	 I2C0->TXDATA=0xE3; // command to read temperature from Si7021
	 while ((I2C0->IF & I2C_IF_ACK) ==  0){;} // wait until ACK is received
	 I2C0->IFC = I2C_IFC_ACK; // clear ACK
	 I2C0->CMD = I2C_CMD_START; // issue write command
	 I2C0->TXDATA=(slave_address << 1) | read; // prepare I2C Tx buffer for reading slave register
	 I2C0->IFC=I2C_IFC_START;
	 while ((I2C0->IF & I2C_IF_ACK) ==  0){;} // wait until ACK is received
	 I2C0->IFC = I2C_IFC_ACK; // clear ACK
	 while ((I2C0->IF & I2C_IF_RXDATAV) ==  0){;} // wait until data from slave received
	 temp_MSB= I2C0->RXDATA; //  Si7021 MSB value
	 I2C0->CMD =I2C_CMD_ACK; // send ACK command
	 while ((I2C0->IF & I2C_IF_RXDATAV) ==  0){;} // wait until data from slave received
	 temp_LSB= I2C0->RXDATA; // Si7021  LSB value
	 I2C0->CMD =I2C_CMD_NACK; // send NACK to signal end of transfer
	 I2C0->CMD = I2C_CMD_STOP; // issue stop command
	 while ((I2C0->IF & I2C_IF_MSTOP) ==  0){;} // wait until I2C0 stops
	 I2C0->IFC=I2C_IFC_MSTOP; // clear stop flag for I2C0, so that it is ready for the next transaction
	 CMU_ClockEnable(cmuClock_I2C0, false); // disable I2C0 clock
 }

 void Si7021_enable(void){
	 GPIO_PinModeSet(Sensor_Enable_Port, Sensor_Enable_Pin, gpioModePushPull, 1); // set sensor enable pin
	 USTIMER_Delay(80000); // 80 ms delay for POR
	 GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeWiredAnd, 1); // activate SCL and SDA lines
	 GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeWiredAnd, 1);

	 /* Toggle I2C SCL 9 times to reset any I2C slave that may require it */
	 for(int i=0;i<9;i++)
	 {
	   	GPIO_PinOutClear(I2C0_SCL_Port, I2C0_SCL_Pin);
	 	GPIO_PinOutSet(I2C0_SCL_Port, I2C0_SCL_Pin);
	 }

	 if (I2C0->STATE & I2C_STATE_BUSY) // if I2C0 busy, abort
	 	{
	       I2C0->CMD = I2C_CMD_ABORT;
	 	}
 }

 void Si7021_disable(void){
 	 GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeDisabled, 0); // disable SCL and SDA lines
 	 GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeDisabled, 0);
     GPIO_PinModeSet(Sensor_Enable_Port, Sensor_Enable_Pin, gpioModeDisabled, 0); // disable sensor enable pin
  }

 void measure_temperature()
 {
   uint8_t HTM_Temperature_Buffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
   uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
   int32_t temperature_data; /* Stores the Temperature data read from the RHT sensor. */
   uint32_t temperature_corrected;   /* Stores the temperature data read from the sensor in the correct format */
   uint8_t *p = HTM_Temperature_Buffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */

   /* Convert flags to bitstream and append them in the HTM temperature data buffer (HTM_Temperature_Buffer) */
   UINT8_TO_BITSTREAM(p, flags);

   /* Si7021 temperature measurement */
   Si7021_enable(); // Enable temperature sensor
   I2C_read(); // Fetch temperature
   Si7021_disable(); // disable temperature sensor
   temperature_data = (temp_MSB << 8) + (temp_LSB & 0xFC); // combine LSB and MSB to get 14-bit temperature
   temperature = (175.72*(float)temperature_data)/65536.0 - 46.85; // convert to deg C (p. 22 of Si7021-A20.pdf)

   /* Convert sensor data to correct temperature format */
   temperature_corrected = FLT_TO_UINT32(temperature_data, -3);

   /* Convert temperature to bitstream and place it in the HTM temperature data buffer (HTM_Temperature_Buffer) */
   UINT32_TO_BITSTREAM(p, temperature_corrected);

   /* Send indication of the temperature in HTM_Temperature_Buffer to all "listening" clients.
    * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
    *  0xFF as connection ID will send indications to all connections. */
   gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_temp_measurement, 5, HTM_Temperature_Buffer);
 }

//***********************************************************************************
// main
//***********************************************************************************

/**
 * @brief  Main function
 */
int main(void)
{
  /* Initialize device */
  initMcu();

  /* Initialize board */
  initBoard();

  /* Initialize application */
  initApp();

  /* Initialize GPIO */
  gpio_init();

  /* Initialize clocks */
  cmu_init();

  /* Initialize stack */
  gecko_init(&config);

  /* Initialize us-timer for POR delay of Si7021 */
  USTIMER_Init();

  /* Initialize I2C peripheral */
  I2C_initialize();

  /* Setup and enable LETIMER */
  LETIMER0_Setup();

  while (1){
	  /* Event pointer for handling events */
	  struct gecko_cmd_packet* evt;

      /* Check for stack event. */
	  evt = gecko_wait_event();

	  /* Handle events */
	  switch (BGLIB_MSG_ID(evt->header)) {
	  case gecko_evt_le_connection_opened_id:
	      // Connection interval of 60 (0x3C) * 1.25 = 75 ms
          // Slave Latency of 75 ms * (5 + 1) = 450 ms
	      // Supervision time out of 500 ms * 10 = 5 sec
	      connection = evt->data.evt_le_connection_opened.connection;
	      gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, 0x003C, 0x003C, 0x0005, 0x1F4);
	  break;

	  /* This boot event is generated when the system boots up after reset.
	   * Do not call any stack commands before receiving the boot event.
	   * Here the system is set to start advertising immediately after boot procedure. */
	  case gecko_evt_system_boot_id:
	      /* Set advertising parameters. 337 ms advertisement interval. All channels used.
	       * The first two parameters are minimum and maximum advertising interval, both in
	       * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
	       gecko_cmd_le_gap_set_adv_parameters(539, 539, 7);

	  	   /* Start general advertising and enable connections. */
	  	   gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
	  break;

	  /* Set TX power adaptively based on rssi values. Function sets TX power in units of 0.1 dbm. */
	  case gecko_evt_le_connection_rssi_id:
	      gecko_cmd_system_halt(1); // halt radio before setting new TX power value
	  	  if (evt->data.evt_le_connection_rssi.rssi > -35 ){
	  	      gecko_cmd_system_set_tx_power(-260);	//setting TX power to min (-26 dbm) when rssi more than -35 dB
	  	  }else if (evt->data.evt_le_connection_rssi.rssi > -45 && evt->data.evt_le_connection_rssi.rssi <-35){
	  	      gecko_cmd_system_set_tx_power(-200);
	  	  }else if(evt->data.evt_le_connection_rssi.rssi > -55 && evt->data.evt_le_connection_rssi.rssi <-45){
	  	      gecko_cmd_system_set_tx_power(-150);
	  	  }else if (evt->data.evt_le_connection_rssi.rssi >-65 && evt->data.evt_le_connection_rssi.rssi< -55){
	  	      gecko_cmd_system_set_tx_power(-50);
	  	  }else if (evt->data.evt_le_connection_rssi.rssi >-75 && evt->data.evt_le_connection_rssi.rssi <-65){
	  	      gecko_cmd_system_set_tx_power(0);
	  	  }else if (evt->data.evt_le_connection_rssi.rssi >-85 && evt->data.evt_le_connection_rssi.rssi < -75){
	  	      gecko_cmd_system_set_tx_power(50);
	  	  }else{
	  	      gecko_cmd_system_set_tx_power(80); //setting TX power to max (8 dbm) when rssi less than -85 dB
	  	  }
	  	  gecko_cmd_system_halt(0); // resume radio operation after setting TX power
	  break;

	  /* This event is generated when a connected client has either
	   * 1) changed a Characteristic Client Configuration, meaning that they have enabled
	   * or disabled Notifications or Indications, or
	   * 2) sent a confirmation upon a successful reception of the indication. */
	  case gecko_evt_gatt_server_characteristic_status_id:
	      /* Check that the characteristic in question is temperature - its ID is defined
	  	   * in gatt.xml as "temp_measurement". Also check that status_flags = 1, meaning that
	  	   * the characteristic client configuration was changed (notifications or indications
	  	   * enabled or disabled). */
	  	   if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temp_measurement)
	  	              && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
	  	         if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
	  	         /* Indications have been turned ON - enable LETIMER to start making temperature measurements */
	  	            	LETIMER_Enable(LETIMER0, true);
	  	         }
	  	         gecko_cmd_le_connection_get_rssi(connection); // get rssi values to perform auto TX power adjustment
	  	      }
	  break;

      /* This event is generated by the external signal corresponding to the LETIMER0 interrupt */
	  case gecko_evt_system_external_signal_id:
	      if(external_event_status & 0x01){ // signal to take temperature reading is active
	  	       measure_temperature();
	  	       if(temperature < temp_set){LED_ON();} // if measured temperature less than reference, turn on LED1
	  	      	    else LED_OFF(); // else turn off LED1
	  	    }
	  	  CORE_ATOMIC_IRQ_DISABLE();
	  	  external_event_status &= ~(0x01); // clear event signal
	  	  CORE_ATOMIC_IRQ_ENABLE();
	  break;

	  case gecko_evt_le_connection_closed_id:
 	      LETIMER_Enable(LETIMER0, false); // connection closed, disable timer and temperature measurement
	  	  gecko_cmd_system_halt(1);
	      gecko_cmd_system_set_tx_power(0); // set TX power to 0 when connection closed
	  	  gecko_cmd_system_halt(0);

	  	  /* Check if need to boot to dfu mode */
	  	  if (boot_to_dfu) {
	  	      /* Enter to DFU OTA mode */
	  	      gecko_cmd_system_reset(2);
	  	    } else {

	  	    /* Restart advertising after client has disconnected */
	  	    gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
	  	    }
	  break;

	  /* Events related to OTA upgrading
	  /* ----------------------------------------------------------------------------- */

	  /* Checks if the user-type OTA Control Characteristic was written.
	  * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
	  case gecko_evt_gatt_server_user_write_request_id:
	  	   if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	  	      /* Set flag to enter to OTA mode */
	  	      boot_to_dfu = 1;
	  	      /* Send response to Write Request */
	  	      gecko_cmd_gatt_server_send_user_write_response(
	  	      evt->data.evt_gatt_server_user_write_request.connection,
	  	      gattdb_ota_control,bg_err_success);

	  	      /* Close connection to enter to DFU OTA mode */
	  	      gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
	  	      }
	  break;

	  default:
	  break;
	  }
   }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

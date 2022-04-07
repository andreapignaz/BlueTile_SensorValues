//Interrupt definitions: BlueNRG1_it.h
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "SensorDemo_config.h"
#include "hw_config.h"
#include "osal.h"
#include "clock.h"
// Pressure Sensor
#include "LPS22HH.h"				
#include "LPS22HH_hal.h"
// Temperature Sensor
#include "HTS221.h"
#include "HTS221_hal.h"
// Acceleration Sensor + Gyroscope
#include "lsm6dso.h"
#include "lsm6dso_hal.h"

/*

SUGGESTIONS
1. Read the ST documentation on the BLUENRG2, in particular the Programmming Guidelines.
It has a lot of explaination on Bluetooth and GATT with nice programming examples.

2. Use the nRF connect app for Android to debug Bluetooth data for the device. 
It can list all the devices with their addresses. It also shows all the GATT services and characteristics.

*/

uint16_t led_count, system_count;

//Sensor variables: pressure, temperature, humidity
uint8_t raw_pressure[3];
int16_t temperature = 0;
uint16_t humidity = 0;

uint8_t data_raw_acceleration[6];
uint8_t data_raw_angular_rate[6];

void Platform_Init(void);
void Init_Pressure_Temperature_Sensor(void);
void Init_Humidity_Sensor(void);
void Init_Accelerometer_Gyroscope(void);

int main(void) {
	
	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init the Hardware platform */
	Platform_Init();
	SdkEvalLedInit(LED1); // Init Red LED
	SdkEvalLedInit(LED2); // Init Green LED
	SdkEvalLedInit(LED3); // Init Blue LED
	SdkEvalLedOn(LED3);   // Turn on Blue LED
	SdkEvalI2CInit(400000);  // Bus initialization, otherwise sensors wouldn't work lol

	/* BlueNRG-2 stack Initialization + Public Address Definition. The Stack Init Parameters are defined in SensorDemo_Config.h */
	BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
	aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
	
	/* Variables */
 	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	uint8_t device_name[] = { 'B', 'C', 'N', '-', '0', '0', '2' };
	
	/*Power level configuration */
	aci_hal_set_tx_power_level(1, 0x07);
	
	/*GATT initialization*/
	aci_gatt_init();
	
	/*Sensor Initalization*/
	Init_Pressure_Temperature_Sensor();
	Init_Humidity_Sensor();
	Init_Accelerometer_Gyroscope();
	
	/*GAP initialization: peripheral role + handle(s) configuration*/
	aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	
	/*GAP device name setting*/
	aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);
	
	/*Security setup: PIN is 123456*/
	aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_NOT_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, 0x00);

	/*Service definition: define the UUID, copy it in the required structure, add the service and obtain the handle.
	  The handle is then used by the rest of the program.  */
	static uint16_t ServiceHandle;
	uint8_t string_service_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0xb4,0x9a,0xe1,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};
	Service_UUID_t service_uuid;
	Osal_MemCpy(&service_uuid.Service_UUID_128, string_service_uuid, 16);
	aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 6, &ServiceHandle);
	
	/*Characteristic definition: defined as service*/
	static uint16_t CharHandle;
	uint8_t string_char_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0xE0,0x00};
	Char_UUID_t char_uuid;
	Osal_MemCpy(&char_uuid.Char_UUID_128, string_char_uuid, 16);
	aci_gatt_add_char(ServiceHandle, UUID_TYPE_128, &char_uuid, 25, CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS, 16, 0, &CharHandle);

	/* Set device connectable (advertising) */
	/*Choose the name*/
	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B','T','n','a','m','e' };
	/*Power level*/
	aci_hal_set_tx_power_level(1, 0x04);
	/*Set reponse data to scan*/
	hci_le_set_scan_response_data(0,NULL);
	/*Set discoverable with the public address and the nome defined before*/
	aci_gap_set_discoverable(ADV_IND, 0x160, 0x160, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

	while (1) {
		/* BLE Stack Tick: let the Bluetooth FSM proceed with one tick */
		BTLE_StackTick();
		
		//Define a buffer that will contain the data to be written on the GATT characteristic
		uint8_t output_buffer[25] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
		//A random value is generated for the response MSB. Just to verify if notihing is stuck. 
			
		output_buffer[0] = rand();
			
		//Add a divider to the buffer (just for fun :D)
		output_buffer[1] = 0xFF;	
			
		//Add Pressure (raw value from the sensor) to the buffer
		lps22hh_pressure_raw_get(0, raw_pressure);
		output_buffer[2] = raw_pressure[0];
		output_buffer[3] = raw_pressure[1];
		output_buffer[4] = raw_pressure[2];
		
		//Divider again
		output_buffer[5] = 0xFF;
			
		//Add Humidity and Temperature values to the buffer, correctly shifted 
		HTS221_Get_Measurement(0, &humidity, &temperature);
		output_buffer[6] = humidity;
		output_buffer[7] = humidity >> 8;
		output_buffer[8] = 0xFF;
		output_buffer[9] = temperature;
		output_buffer[10] = temperature >> 8;
		output_buffer[11] = 0xFF;		
		
		//Add accelerometer and gyroscope values to the buffer
		lsm6dso_acceleration_raw_get(0, data_raw_acceleration);
		lsm6dso_angular_rate_raw_get(0, data_raw_angular_rate);
	  for(int k=0; k<6; k++)
			output_buffer[12 + k] = data_raw_acceleration[k];
		output_buffer[18] = 0xFF;
		for(int k=0; k<6; k++)
			output_buffer[19 + k] = data_raw_angular_rate[k];
		
		//Publish the buffer	
		aci_gatt_update_char_value(ServiceHandle, CharHandle, 0, 19, output_buffer);
		
	}
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/* Operations performed when connection with a device completes. */
void hci_le_connection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Role, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint16_t Conn_Interval, uint16_t Conn_Latency, uint16_t Supervision_Timeout, uint8_t Master_Clock_Accuracy) {

	SdkEvalLedOff(LED1);
	SdkEvalLedOff(LED2);
	SdkEvalLedOff(LED3);
	
}

/*Operations performed when DISconnection from a device completes.*/
void hci_disconnection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Reason) {

	SdkEvalLedOff(LED1);
	SdkEvalLedOff(LED2);
	SdkEvalLedOn(LED3);

	/* Set device connectable (copied from main) */
	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B','T','n','a','m','e' };
	aci_hal_set_tx_power_level(1, 0x04);
	hci_le_set_scan_response_data(0,NULL);
	aci_gap_set_discoverable(ADV_IND, 0x160, 0x160, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

}



/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty())
		return SLEEPMODE_RUNNING;
	return sleepMode;
}

/***************************************************************************************/

void Platform_Init(void) {

	/* Configure I/O communication channel */
	SdkEvalComUartInit(UART_BAUDRATE);
	SdkEvalComUartIrqConfig(ENABLE);

	// Init the systick
	Clock_Init();

	/* GPIO Configuration */
	GPIO_InitType GPIO_InitStructure;

	/** GPIO Periph clock enable */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/** Init Structure */
	GPIO_StructInit(&GPIO_InitStructure);
	/** Configure GPIO_Pin_7 for Proximity XSHUT */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);
}

void Init_Pressure_Temperature_Sensor(void) {

	lps22hh_i3c_interface_set(0, LPS22HH_I3C_DISABLE);
	lps22hh_block_data_update_set(0, PROPERTY_ENABLE);
	lps22hh_data_rate_set(0, LPS22HH_50_Hz_LOW_NOISE);
	lps22hh_pin_mode_set(0, LPS22HH_OPEN_DRAIN);
	lps22hh_pin_polarity_set(0, LPS22HH_ACTIVE_LOW);
	lps22hh_int_notification_set(0, LPS22HH_INT_PULSED);
	lps22hh_int_pd_set(0, LPS22HH_PULL_DOWN_DISCONNECT);

}

void Init_Humidity_Sensor(void) {

	HTS221_Set_AvgH(0, HTS221_AVGH_4);
	HTS221_Set_AvgT(0, HTS221_AVGT_2);
	HTS221_Set_BduMode(0, HTS221_ENABLE);
	HTS221_Set_Odr(0, HTS221_ODR_12_5HZ);
	HTS221_Set_HeaterState(0, HTS221_DISABLE);

	HTS221_Set_IrqOutputType(0, HTS221_OPENDRAIN);
	HTS221_Set_IrqActiveLevel(0, HTS221_LOW_LVL);

	HTS221_Set_PowerDownMode(0, HTS221_RESET);
	HTS221_Activate(0);

}

void Init_Accelerometer_Gyroscope(void) {

	uint8_t rst;

	lsm6dso_i3c_disable_set(0, LSM6DSO_I3C_DISABLE);

	rst = lsm6dso_reset_set(0, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(0, &rst);
	} while (rst);

	lsm6dso_pin_mode_set(0, LSM6DSO_PUSH_PULL);
	lsm6dso_pin_polarity_set(0, LSM6DSO_ACTIVE_LOW);
	lsm6dso_all_on_int1_set(0, PROPERTY_ENABLE);
	lsm6dso_int_notification_set(0, LSM6DSO_ALL_INT_LATCHED);

	lsm6dso_block_data_update_set(0, PROPERTY_ENABLE);
	lsm6dso_xl_power_mode_set(0, LSM6DSO_LOW_NORMAL_POWER_MD);
	lsm6dso_gy_power_mode_set(0, LSM6DSO_GY_NORMAL);
	lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
	lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
	lsm6dso_xl_full_scale_set(0, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(0, LSM6DSO_2000dps);

	lsm6dso_auto_increment_set(0, PROPERTY_ENABLE);
}


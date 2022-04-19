#ifndef _SENSORDEMO_CONFIG_H_
#define _SENSORDEMO_CONFIG_H_

#include "bluenrg1_stack.h"
#include "stack_user_cfg.h"
#include "OTA_btl.h"

/* This file contains all the information needed to init the BlueNRG-1,2 BLE stack.
These constants and variables are used from the BlueNRG-1,2 BLE stack to reserve RAM and FLASH according the application requests. */

/* Default number of link */
#define MIN_NUM_LINK	(1)

/* Default number of GAP and GATT services */
#define DEFAULT_NUM_GATT_SERVICES	(2)

/* Default number of GAP and GATT attributes */
#define DEFAULT_NUM_GATT_ATTRIBUTES	(9)

/* Enable/disable Data length extension Max supported ATT_MTU size based on OTA client & server Max ATT_MTU sizes capabilities */
#if (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED == 1) && (OTA_EXTENDED_PACKET_LEN == 1) 
  #define OTA_MAX_ATT_MTU_SIZE	(OTA_ATT_MTU_SIZE)	/*OTA Client & Server supported ATT_MTU*/
#else /* BlueNRG-1 device: no data length extension support */
  #define OTA_MAX_ATT_MTU_SIZE	(DEFAULT_ATT_MTU)	/*DEFAULT_ATT_MTU size = 23 bytes*/
#endif


#if defined (ST_OTA_LOWER_APPLICATION) || defined (ST_OTA_HIGHER_APPLICATION)
/* Number of services requests from the SensorDemo demo plus 1 OTA Service */
#define NUM_APP_GATT_SERVICES	(1+1)

/* Number of attributes requests from the SensorDemo demo plus 9 attributes required for OTA Service Characteristics */
#define NUM_APP_GATT_ATTRIBUTES	(15+9)

/* Set the number of 16-bytes units used on an OTA FW data packet for matching OTA client MAX ATT_MTU */
#define OTA_16_BYTES_BLOCKS_NUMBER	((OTA_MAX_ATT_MTU_SIZE-4)/16)	/*4 bytes for OTA sequence numbers + needs ack + checksum bytes*/

/* OTA characteristics maximum lenght */
#define OTA_MAX_ATT_SIZE	(4 + OTA_16_BYTES_BLOCKS_NUMBER * 16 )

#else
/* NO OTA Service is required */

/* Number of services requests from the SensorDemo demo */
#define NUM_APP_GATT_SERVICES	(1)

/* Number of attributes requests from the SensorDemo demo */
#define NUM_APP_GATT_ATTRIBUTES	(15)

/* OTA characteristics maximum lenght */
#define OTA_MAX_ATT_SIZE	(0)

#endif


#define MAX_CHAR_LEN(a,b) ((a) > (b) )? (a) : (b)

/* Set supported max value for attribute size: it is the biggest attribute size enabled by the application. */
#define SENSORDEMO_MAX_ATT_SIZE	(150)
#define APP_MAX_ATT_SIZE	  MAX_CHAR_LEN(OTA_MAX_ATT_SIZE,  SENSORDEMO_MAX_ATT_SIZE)

/* Number of links needed for the SensorDemo demo: 1 */
#define NUM_LINKS	(MIN_NUM_LINK)

/* Max Number of Attribute Records that can be added for the services */
#define MAX_NUMBER_ATTRIBUTE_SERVICE1	(16)


/* Number of GATT attributes needed for the SensorDemo demo */
#define NUM_GATT_ATTRIBUTES	(DEFAULT_NUM_GATT_ATTRIBUTES+NUM_APP_GATT_ATTRIBUTES)

/* Number of GATT services needed for the SensorDemo demo */
#define NUM_GATT_SERVICES	(DEFAULT_NUM_GATT_SERVICES+NUM_APP_GATT_SERVICES)

/* Array size for the attribute value for OTA service */
#if defined (ST_OTA_LOWER_APPLICATION) || defined (ST_OTA_HIGHER_APPLICATION)
#define OTA_ATT_VALUE_ARRAY_SIZE	(99 + (4 + (OTA_16_BYTES_BLOCKS_NUMBER * 16)))	/*OTA service: 4 characteristics (1 notify property): 99 bytes + Image Content characteristic length = 4  + (OTA_16_BYTES_BLOCKS_NUMBER * 16); 4 for sequence number, checksum and needs acks bytes*/
#else
#define OTA_ATT_VALUE_ARRAY_SIZE	(0)	/*No OTA service is used*/
#endif
/* Array size for the attribute value */
#define ATT_VALUE_ARRAY_SIZE	(748+OTA_ATT_VALUE_ARRAY_SIZE)

/* Set the size of Flash security database */
#define FLASH_SEC_DB_SIZE	(0X400)
/* Set the size of Flash security database */
#define FLASH_SERVER_DB_SIZE	(0X400)

/* Set supported max value for ATT_MTU enabled by the application */
#define APP_MAX_MTU_SIZE	(200)
#define MAX_ATT_MTU  MAX_CHAR_LEN(OTA_MAX_ATT_MTU_SIZE,  APP_MAX_MTU_SIZE) 

#define MAX_ATT_SIZE    (APP_MAX_ATT_SIZE)

/*  Set the minumum number of prepare write requests needed for a long write procedure for a characteristic with len > 20bytes: 
 
 It returns 0 for characteristics with len <= 20bytes
 
 NOTE: If prepare write requests are used for a characteristic (reliable write on multiple characteristics), then 
 this value should be set to the number of prepare write needed by the application.
 
  [New parameter added on BLE stack v2.x] 
 */

#define PREPARE_WRITE_LIST_SIZE	(PREP_WRITE_X_ATT(MAX_ATT_SIZE))

/* Additional number of memory blocks to be added to the minimum */
#define OPT_MBLOCKS	(6)

/* Set the number of memory block for packet allocation */
#define MBLOCKS_COUNT	(MBLOCKS_CALC(PREPARE_WRITE_LIST_SIZE, MAX_ATT_MTU, NUM_LINKS)+ OPT_MBLOCKS)

/* RAM reserved to manage all the data stack according the number of links,
number of services, number of attributes and attribute value length */
NO_INIT(uint32_t dyn_alloc_a[TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_GATT_SERVICES,ATT_VALUE_ARRAY_SIZE,MBLOCKS_COUNT,CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED)>>2]);

/* FLASH reserved to store all the security database information and and the server database information */
NO_INIT_SECTION(uint32_t stacklib_flash_data[TOTAL_FLASH_BUFFER_SIZE(FLASH_SEC_DB_SIZE, FLASH_SERVER_DB_SIZE)>>2], ".noinit.stacklib_flash_data");

/* FLASH reserved to store: security root keys, static random address, public address. */
NO_INIT_SECTION(uint8_t stacklib_stored_device_id_data[56], ".noinit.stacklib_stored_device_id_data");

/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH	(0XFFFFFFFF)


/* Sleep clock accuracy */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)

/* Sleep clock accuracy in Slave mode */
#define SLAVE_SLEEP_CLOCK_ACCURACY	(500)

/* Sleep clock accuracy in Master mode */
#define MASTER_SLEEP_CLOCK_ACCURACY	(MASTER_SCA_500ppm)

#else

/* Sleep clock accuracy in Slave mode */
#define SLAVE_SLEEP_CLOCK_ACCURACY	(100)

/* Sleep clock accuracy in Master mode */
#define MASTER_SLEEP_CLOCK_ACCURACY	(MASTER_SCA_100ppm)
#endif


/* Low Speed Oscillator source */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)/* Internal RO */

#define LOW_SPEED_SOURCE	(1)	/*Internal RO*/

#else
/* External 32 KHz */
#define LOW_SPEED_SOURCE	(0)	/* External 32 KHz*/

#endif

/* High Speed start up time */
#define HS_STARTUP_TIME	(0x148)	/*800 us*/

/* Radio Config Hot Table  */
extern uint8_t hot_table_radio_config[];

/* Low level hardware configuration data for the device */

#define CONFIG_TABLE \
{\
(uint32_t*)hot_table_radio_config,\
MAX_CONN_EVENT_LENGTH,\
SLAVE_SLEEP_CLOCK_ACCURACY,\
MASTER_SLEEP_CLOCK_ACCURACY,\
LOW_SPEED_SOURCE,\
HS_STARTUP_TIME\
}

/* This structure contains memory and low level hardware configuration data for the device */
const BlueNRG_Stack_Initialization_t BlueNRG_Stack_Init_params = {
(uint8_t*)stacklib_flash_data,
FLASH_SEC_DB_SIZE,
FLASH_SERVER_DB_SIZE,
(uint8_t*)stacklib_stored_device_id_data,
(uint8_t*)dyn_alloc_a,
TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_GATT_SERVICES,ATT_VALUE_ARRAY_SIZE,MBLOCKS_COUNT,CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED),
NUM_GATT_ATTRIBUTES,
NUM_GATT_SERVICES,
ATT_VALUE_ARRAY_SIZE,
NUM_LINKS,
0, /* reserved for future use */
PREPARE_WRITE_LIST_SIZE,
MBLOCKS_COUNT,
MAX_ATT_MTU,
CONFIG_TABLE
};


/*
//Device Name and Type
#define NAME_DEVICE	(BlueNRG)
#define TYPE_DEVICE	(BlueNRG-2)
#define FW_STACK_VERSION	(2.1e)
*/

/*
// Service Characteristics Information

// Number of characteristics defined by the user application

// Char property for Service 1
#define NUMBER_CHAR_SERVICE_1	(5)

#define UUID_TYPE_CHAR_1_1	(2)
#define PROPERTY_CHAR_1_1	(128)
#define VALUE_LEN_CHAR_1_1	(100)
#define NUM_DESC_CHAR_1_1	(0)
#define SIZE_DESC_CHAR_1_1	(0)

#define UUID_TYPE_CHAR_1_2	(2)
#define PROPERTY_CHAR_1_2	(128)
#define VALUE_LEN_CHAR_1_2	(75)
#define NUM_DESC_CHAR_1_2	(0)
#define SIZE_DESC_CHAR_1_2	(0)

#define UUID_TYPE_CHAR_1_3	(2)
#define PROPERTY_CHAR_1_3	(128)
#define VALUE_LEN_CHAR_1_3	(75)
#define NUM_DESC_CHAR_1_3	(0)
#define SIZE_DESC_CHAR_1_3	(0)

#define UUID_TYPE_CHAR_1_4	(2)
#define PROPERTY_CHAR_1_4	(128)
#define VALUE_LEN_CHAR_1_4	(175)
#define NUM_DESC_CHAR_1_4	(0)
#define SIZE_DESC_CHAR_1_4	(0)

#define UUID_TYPE_CHAR_1_5	(2)
#define PROPERTY_CHAR_1_5	(128)
#define VALUE_LEN_CHAR_1_5	(175)
#define NUM_DESC_CHAR_1_5	(0)
#define SIZE_DESC_CHAR_1_5	(0)

*/

#endif

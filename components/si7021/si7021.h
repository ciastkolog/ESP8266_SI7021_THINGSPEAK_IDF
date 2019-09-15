/*
 * HTU21D Component
 *
 * esp-idf component to interface with HTU21D humidity and temperature sensor
 * by TE Connectivity (http://www.te.com/usa-en/product-CAT-HSC0004.html)
 *
 * Luca Dentella, www.lucadentella.it
 */

#ifndef __ESP_SI7021_H__
#define __ESP_SI7021_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           4                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           5               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define SI7021_SENSOR_ADDR                  0x40             /*!< slave address for SI7012 sensor */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

// sensor address
#define SI7021_MEASRH_HOLD_CMD           0xE5
#define SI7021_MEASRH_NOHOLD_CMD         0xF5
#define SI7021_MEASTEMP_HOLD_CMD         0xE3   // TRIGGER_TEMP_MEASURE_HOLD
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3
#define SI7021_READPREVTEMP_CMD          0xE0
#define SI7021_RESET_CMD                 0xFE
#define SI7021_WRITERHT_REG_CMD          0xE6
#define SI7021_READRHT_REG_CMD           0xE7
#define SI7021_WRITEHEATER_REG_CMD       0x51
#define SI7021_READHEATER_REG_CMD        0x11
#define SI7021_ID1_CMD                   0xFA0F
#define SI7021_ID2_CMD                   0xFCC9
#define SI7021_FIRMVERS_CMD              0x84B8

#define RES0                             0      //User Register D0: Measurement Resolution for Temp[RES[1:0]]
#define RES1                             7      //User Register D7: Measurement Resolution for RH[RES[1:0]]
#define VDDS                             6
#define HTRE                             2

typedef enum Si7021_commands
{
  Humi_HM        = 0xE5, // Measure Relative Humidity, Hold Master Mode
  Humi_NHM       = 0xF5, // Measure Relative Humidity, No Hold Master Mode
  Temp_HM        = 0xE3, // Measure Temperature, Hold Master Mode
  Temp_NHM       = 0xF3, // Measure Temperature, No Hold Master Mode
  Temp_AH        = 0xE0, // Read Temperature Value from Previous RH Measurement
  Si7021_Reset   = 0xFE, // Reset
  W_RHT_U_reg    = 0xE6, // Write RH/T User Register 1
  R_RHT_U_reg    = 0xE7, // Read RH/T User Register 1
  W_Heater_C_reg = 0x51, // Write Heater Control Register
  R_Heater_C_reg = 0x11, // Read Heater Control Register
  R_ID_Byte11    = 0xFA, // Read Electronic ID 1st Byte, first part
  R_ID_Byte12    = 0x0F, // Read Electronic ID 1st Byte, second part
  R_ID_Byte21    = 0xFC, // Read Electronic ID 2nd Byte, first part
  R_ID_Byte22    = 0xC9, // Read Electronic ID 2nd Byte, second part
  R_Firm_rev1    = 0x84, // Read Firmware Revision, first part
  R_Firm_rev2    = 0xB8  // Read Firmware Revision, second part
}Si7021_commands_t;

// HTU21D commands
#define TRIGGER_TEMP_MEASURE_HOLD  		0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  		0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  	0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  	0xF5
#define WRITE_USER_REG  				0xE6
#define READ_USER_REG  					0xE7
#define SOFT_RESET  					0xFE

// return values
// #define SI7021_ERR_OK				0x00
// #define SI7021_ERR_CONFIG			0x01
// #define SI7021_ERR_INSTALL			0x02
// #define SI7021_ERR_NOTFOUND			0x03
// #define SI7021_ERR_INVALID_ARG		0x04
// #define SI7021_ERR_FAIL		 		0x05
// #define SI7021_ERR_INVALID_STATE	0x06
// #define SI7021_ERR_TIMEOUT	 		0x07

typedef enum Si7021_errs
{
    SI7021_ERR_OK               = 0x00,
    SI7021_ERR_CONFIG	        = 0x01,
    SI7021_ERR_INSTALL	        = 0x02,
    SI7021_ERR_NOTFOUND	        = 0x03,
    SI7021_ERR_INVALID_ARG	    = 0x04,
    SI7021_ERR_FAIL		 	    = 0x05,
    SI7021_ERR_INVALID_STATE	= 0x06,
    SI7021_ERR_TIMEOUT	 	    = 0x07
} Si7021_err_t;
typedef enum Si7021_resolution
{
  H12_T14 = 0x00,   //0011 1010 -> 0x3A
  H8_T12  = 0x01,   //0011 1011 -> 0x3B
  H10_T13 = 0x80,   //1011 1010 -> 0xBA
  H11_T11 = 0x81    //1011 1011 -> 0xBB
}Si7021_resolution_t;
// functions
uint8_t Si7021_Init();
float Si7021_ReadTemperature();
float Si7021_ReadHumidity();
uint8_t Si7021_GetResolution();
uint8_t Si7021_SetResolution(Si7021_resolution_t resolution);
uint8_t Si7021_ReadUserRegister();
uint8_t Si7021_ReadHeaterCRegister();
uint8_t Si7021_WriteUserRegister(uint8_t value);
uint8_t Si7021_WriteHearterCRegister(uint8_t value);
uint16_t ReadValue(Si7021_commands_t command);
bool IsCrcValid(uint16_t value, uint8_t crc);
uint8_t Si7021_SoftReset();
uint8_t Si7021_EnableHeater(uint8_t val);
uint8_t Si7021_ReadHeaterCurrent();
uint8_t Si7021_SetHeaterCurrent(uint8_t current);
uint8_t Si7021_ReadFirmwareRev();

#ifdef __cplusplus
}
#endif

#endif  // __ESP_SI7021_H__

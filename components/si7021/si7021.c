#include "si7021.h"

static const char *TAG = "Si7021";
static const uint8_t  HEATER_CURRENT_OFFSET = 3;      // current value in mA for register value 0
static const uint8_t  HEATER_CURRENT_STEP   = 6;      // mA/LSB

static uint8_t heater_control_register = 0b00000000;
static uint8_t user_register_1 = 0b00111010;	//User Register 1 default value

uint8_t Si7021_Init()
{
	esp_err_t ret;
	int i2c_master_port = I2C_MASTER_NUM;
	
	//setup i2c controller
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
	//install the driver
	ret = i2c_driver_install(i2c_master_port, conf.mode);
	if(ret != ESP_OK)
	{
    ESP_LOGI(TAG, "Si7021 Driver Install Error.\n");
	return SI7021_ERR_INSTALL;
	}
    ret =i2c_param_config(i2c_master_port, &conf);
	if(ret != ESP_OK)
	{
    ESP_LOGI(TAG, "Si7021 Config Error.\n");
	return SI7021_ERR_CONFIG;
	}
	//verify if a sensor is present
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    if(ret != ESP_OK)
	{
    ESP_LOGI(TAG, "Sensor Si7021 is not present.\n");
	return SI7021_ERR_NOTFOUND;
	}

    ESP_LOGI(TAG, "Sensor Si7021 Inited.\n");
    return SI7021_ERR_OK;
}
// verify the CRC, algorithm in the datasheet (see comments below)
bool IsCrcValid(uint16_t value, uint8_t crc) {

	// line the bits representing the input in a row (first data, then crc)
	uint32_t row = (uint32_t)value << 8;
	row |= crc;

	// polynomial = x^8 + x^5 + x^4 + 1
	// padded with zeroes corresponding to the bit length of the CRC
	uint32_t divisor = (uint32_t)0x988000;

	for (int i = 0 ; i < 16 ; i++) {

		// if the input bit above the leftmost divisor bit is 1,
		// the divisor is XORed into the input
		if (row & (uint32_t)1 << (23 - i)) row ^= divisor;

		// the divisor is then shifted one bit to the right
		divisor >>= 1;
	}

	// the remainder should equal zero if there are no detectable errors
	return (row == 0);
}
uint16_t ReadValue(Si7021_commands_t command) 
{
	esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, command, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;

	// wait for the sensor (50ms)
	vTaskDelay(50 / portTICK_RATE_MS);

	// receive the answer
	uint8_t msb, lsb, crc;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &msb, ACK_VAL);
	i2c_master_read_byte(cmd, &lsb, ACK_VAL);
	i2c_master_read_byte(cmd, &crc, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;

	uint16_t raw_value = ((uint16_t) msb << 8) | (uint16_t) lsb;
	if(!IsCrcValid(raw_value, crc)) printf("CRC invalid\r\n");

	ESP_LOGI(TAG, "Command: (Binary)0x%0X, rawValue: (Decimal)%d, Return: (Dec)%d\n",command, raw_value, (raw_value&0xFFFC));
	return raw_value & 0xFFFC;
}
float Si7021_ReadTemperature() 
{

	// get the raw value from the sensor
	uint16_t raw_temperature = ReadValue(Temp_NHM);
	if(raw_temperature == 0) return -999;

	// return the real value, formula in datasheet
	return (raw_temperature * 175.72 / 65536.0) - 46.85;
}

float Si7021_ReadHumidity() {

	// get the raw value from the sensor
	uint16_t raw_humidity = ReadValue(Humi_NHM);
	if(raw_humidity == 0) return -999;

	// return the real value, formula in datasheet
	return (raw_humidity * 125.0 / 65536.0) - 6.0;
}
uint8_t Si7021_ReadUserRegister() 
{
	esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, R_RHT_U_reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;

	// receive the answer
	uint8_t reg_value;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &reg_value, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;
	
	ESP_LOGI(TAG, "UserRegValue: 0x%X\n", reg_value);
	return reg_value;
}
uint8_t Si7021_ReadHeaterCRegister() 
{
	esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, R_Heater_C_reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;

	// receive the answer
	uint8_t reg_value;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &reg_value, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return 0;
	
	ESP_LOGI(TAG, "HeaterControlRegValue: 0x%X\n", reg_value);
	return reg_value;
}
uint8_t Si7021_WriteUserRegister(uint8_t value) 
{
	esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, W_RHT_U_reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	switch(ret) {

		case ESP_ERR_INVALID_ARG:
			return SI7021_ERR_INVALID_ARG;

		case ESP_FAIL:
			return SI7021_ERR_FAIL;

		case ESP_ERR_INVALID_STATE:
			return SI7021_ERR_INVALID_STATE;

		case ESP_ERR_TIMEOUT:
			return SI7021_ERR_TIMEOUT;
	}
	return SI7021_ERR_OK;
}
uint8_t Si7021_WriteHeaterCRegister(uint8_t value) 
{
	esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, W_Heater_C_reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	switch(ret) {

		case ESP_ERR_INVALID_ARG:
			return SI7021_ERR_INVALID_ARG;

		case ESP_FAIL:
			return SI7021_ERR_FAIL;

		case ESP_ERR_INVALID_STATE:
			return SI7021_ERR_INVALID_STATE;

		case ESP_ERR_TIMEOUT:
			return SI7021_ERR_TIMEOUT;
	}
	return SI7021_ERR_OK;
}
uint8_t Si7021_GetResolution() 
{

	uint8_t reg_value = Si7021_ReadUserRegister();

	ESP_LOGI(TAG, "GetResolutionValue: 0x%0X, GetUserRegisterValue: 0x%0X\n", (reg_value & 0b10000001), reg_value);
	return reg_value & 0b10000001;
}

uint8_t Si7021_SetResolution(Si7021_resolution_t resolution)
 {
  int8_t reg_value;
  uint8_t temp = user_register_1;

  switch(resolution)
  {
    case H12_T14:
    {
      user_register_1 &= (uint8_t)(~(1<<RES1) & ~(1<<RES0));
      reg_value = Si7021_WriteUserRegister(user_register_1);
      break;
    }
    case H8_T12:
    {
      user_register_1 &= (uint8_t)(~(1<<RES1));
      user_register_1 |= (1<<RES0);
      reg_value = Si7021_WriteUserRegister(user_register_1);
      break;
    }
    case H10_T13:
    {
      user_register_1 &= ~(1<<RES0);
      user_register_1 |= (1<<RES1);
      reg_value = Si7021_WriteUserRegister(user_register_1);
      break;
    }
    case H11_T11:
    {
      user_register_1 |= (1<<RES1) | (1<<RES0);
      reg_value = Si7021_WriteUserRegister(user_register_1);
      break;
    }
    default: return -1;
  }

  /* in case of write error restore local copy of the register value */
  if(reg_value < 0)
    user_register_1 = temp;

  return reg_value;
}
uint8_t Si7021_EnableHeater(uint8_t cmd_code)
{
	uint8_t reg_value;
	uint8_t reg_write = Si7021_ReadUserRegister();

	if(cmd_code == 0)
	{
		reg_write &= ~(1<<HTRE);
		reg_value = Si7021_WriteUserRegister(reg_write);
	}
	else
	{
		reg_write |= (1<<HTRE);
		reg_value = Si7021_WriteUserRegister(reg_write);
	}
  
  return reg_value;
}
uint8_t Si7021_ReadFirmwareRev()
{
  esp_err_t ret;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, R_Firm_rev1, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, R_Firm_rev2, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return -1;  

 	// wait for the sensor (50ms)
	vTaskDelay(50 / portTICK_RATE_MS); 
	
	uint8_t data;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI7021_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &data, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) return -1;
	ESP_LOGI(TAG, "Data: 0x%0X\n", data);
  
	switch(data)
  	{
    case 0xFF: return 1;
    case 0x20: return 2;
    default: return -1;
  	}
}
uint8_t Si7021_ReadHeaterCurrent()
{
	if(Si7021_ReadHeaterCRegister < 0)	return -1;
	return ((heater_control_register & (0x0F)) * HEATER_CURRENT_STEP) + HEATER_CURRENT_OFFSET;
}
uint8_t Si7021_SetHeaterCurrent(uint8_t current)
{
	uint8_t reg_value = (current - HEATER_CURRENT_OFFSET) / HEATER_CURRENT_STEP;

	if(reg_value > 0x0F)	reg_value = 0x0F;
	if(Si7021_WriteHeaterCRegister(reg_value) < 0)	return -1;
	
	heater_control_register = reg_value;
	return 0;
}
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "rom/ets_sys.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_err.h"

#include "nvs_flash.h"

#include <stdlib.h>
#include "app_wifi.h"

#include "driver/adc.h"

#include "esp_http_client.h"

#include "si7021.h"


#define MAX_HTTP_RECV_BUFFER 512

#define THINGSPEAK_API_KEY CONFIG_THINGSPEAK_API

static const char *TAG = "main";

typedef struct{
	float temperature;
	float humidity;
	float voltage;
}sensor_data_t;

sensor_data_t sensor_data;

void system_deep_sleep_instant(uint32_t us);

/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem
   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null
   The CA root cert is the last cert given in the chain of certs.
   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static void post_sensor_data_Thingspeak(sensor_data_t* sensor_data)
{
    esp_http_client_config_t config = {
        .url = "https://api.thingspeak.com",
        .event_handler = _http_event_handler,
        .cert_pem = howsmyssl_com_root_cert_pem_start,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);


    char post_data[128];
    char url[128];
    sprintf(post_data,"field1=%d.%d&field2=%d.%d&field3=%d.%d",(uint16_t)sensor_data->humidity,(uint16_t)(sensor_data->humidity * 100) % 100,
   (uint16_t)sensor_data->temperature, (uint16_t)(sensor_data->temperature * 100) % 100,(uint16_t)sensor_data->voltage, (uint16_t)(sensor_data->voltage * 100) % 100);
    sprintf(url,"https://api.thingspeak.com/update?api_key=%s&",THINGSPEAK_API_KEY);
    esp_http_client_set_url(client, url);
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %d", err);
    }
    esp_http_client_cleanup(client);
}

float batery_voltage_measure(void){
    adc_config_t adc_config;
    float voltage = 0;
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8; // ADC sample collection clock = 80MHz/clk_div = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));
    uint16_t adc_data;
    if (ESP_OK == adc_read(&adc_data)) {
       voltage = (float)adc_data * 0.003;
    }
    return voltage;
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    app_wifi_initialise();

    Si7021_Init();
    uint8_t Si7021_ErrCode;
    Si7021_ReadFirmwareRev();
    Si7021_ErrCode = Si7021_SetResolution(H11_T11);
    ESP_LOGI(TAG, "Si7021_ErrCode: 0x%0X\n", Si7021_ErrCode);
    sensor_data.humidity = Si7021_ReadHumidity();
    sensor_data.temperature = Si7021_ReadTemperature();

   	ESP_LOGI(TAG, "Humidity:%d.%d, Temperature: %d.%d\n",
                           (uint16_t)sensor_data.humidity,(uint16_t)(sensor_data.humidity * 100) % 100,
                           (uint16_t)sensor_data.temperature, (uint16_t)(sensor_data.temperature * 100) % 100 );

   sensor_data.voltage = batery_voltage_measure();

   app_wifi_wait_connected(1000*30);

   post_sensor_data_Thingspeak(&sensor_data);

   system_deep_sleep_instant(1000*1000*60*5);
}

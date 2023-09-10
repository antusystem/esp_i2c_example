#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-master";

#define DATA_LENGTH 512                         /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128                      /*!< Data length for r/w test, [0,DATA_LENGTH] */

#define I2C_MASTER_SCL_IO 22                    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21                    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_SLAVE_NUM 1                         /*!< I2C port number for slave dev */
#define SLAVE_ADDRESS 0x28                     /*!< ESP32 slave address, you can set any 7bit value */

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_master_send(uint8_t value[], int len)
{
    ESP_LOGI(TAG, "Sending Message = %s", value);   
    
    esp_err_t ret; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SLAVE_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, value, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave_buff(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    // Could not make it work for reading the slave
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        esp_err_t rett = i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    } else {
        esp_err_t rett = 0;
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{

    // const uint8_t  on_command[] = "LED_ON";
    // const uint8_t  off_command[] = "LED_OFF";
    uint8_t  value[] = "1";

    int ret;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    uint8_t *data_rd = (uint8_t *)malloc(value);

    uint8_t rx_data[DATA_LENGTH];


    while(1)
    {
        ESP_LOGI(TAG, "Value: %s\n", value);
        ESP_LOGI(TAG, "Writing to Slave");
        ret = i2c_master_send(value, sizeof(value));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Informacion Enviada");
            // value[0]++;
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else {
            ESP_LOGE(TAG, "ESP ERROR: %s", esp_err_to_name(ret));
        }
        // vTaskDelay(1000/ portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "Reading Slave Buffer");
        // ret = i2c_master_read_slave_buff(I2C_MASTER_NUM, data_rd, sizeof(value));
        ret = i2c_master_read_from_device(I2C_MASTER_NUM, SLAVE_ADDRESS, rx_data, DATA_LENGTH, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Read sucessful");
            ESP_LOGI(TAG, "rx_data: %s\n", rx_data);
            value[0] = rx_data[0];
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else {
            ESP_LOGE(TAG, "ESP ERROR: %s", esp_err_to_name(ret));
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "---------------------------------------------------------------");
    }
    

}
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


static const char *TAG = "i2c-slave";

#define LED_PIN 2

#define DATA_LENGTH 512                         /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128                      /*!< Data length for r/w test, [0,DATA_LENGTH] */
// #define DELAY_TIME_BETWEEN_ITEMS_MS 1000        /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO 5                      /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 4                      /*!< gpio number for i2c slave data */
// #define I2C_SLAVE_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)  /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)  /*!< I2C slave rx buffer size */
#define ESP_SLAVE_ADDR 0x28                     /*!< ESP32 slave address, you can set any 7bit value */

#define I2C_SLAVE_NUM 0                         /*!< I2C port number for slave dev */
#define I2C_MASTER_NUM 0                        /*!< I2C port number for master dev */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void app_main(void)
{
    uint8_t  received_data[I2C_SLAVE_RX_BUF_LEN] = {0};

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_LOGI(TAG, "I2C Slave initialized successfully");

    while(1) {
        ESP_LOGW(TAG, "------------------------------");
        i2c_slave_read_buffer(I2C_SLAVE_NUM, received_data, I2C_SLAVE_RX_BUF_LEN, 1000 / portTICK_PERIOD_MS);
        i2c_reset_rx_fifo(I2C_SLAVE_NUM);

        // Checkign if data is not empty
        if (memcmp(received_data, (uint8_t[I2C_SLAVE_RX_BUF_LEN]){0}, sizeof(received_data)) == 0) {
            ESP_LOGI(TAG, "received_data is NULL (not initialized)");
        } else {
            gpio_set_level(LED_PIN, 1);
            ESP_LOGW(TAG, "received_data is NOT NULL (initialized)");
            ESP_LOGW(TAG, "received_data = %s", received_data);
            received_data[0]++;
            ESP_LOGW(TAG, "New value = %s", received_data);
            size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, received_data, DATA_LENGTH, 1000 / portTICK_PERIOD_MS);
            ESP_LOGW(TAG, "Data sent");
            ESP_LOGW(TAG, "d_size = %d", d_size);
            gpio_set_level(LED_PIN, 0);
        }

        // Reseting received_data
        memset(received_data, 0, I2C_SLAVE_RX_BUF_LEN);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

}
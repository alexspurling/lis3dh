#include <stdio.h>
#include <esp_log.h>
#include "driver/i2c.h"

#define LED_PIN GPIO_NUM_5
#define HIGH 1
#define LOW 0

#define ACCEL 0x19 /* Address of LIS3DH Accelerometer */

#define I2C_MASTER_SCL_IO  GPIO_NUM_22 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  GPIO_NUM_21 /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM  I2C_NUM_0      /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE  0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

//Error handler for when things go wrong
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);


/**
 * Initalise the accelerometer as an I2C slave
 */
void init_i2c_device()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);


}

esp_err_t write_byte(uint8_t value)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (ACCEL << 1) | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, value, ACK_CHECK_DIS);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   // ESP_ERROR_CHECK(ret);
   if (ret != ESP_OK) {
     printf("Error writing i2c byte %d. Ret: %d\n", value, ret);
   } else {
     printf("Successfully wrote i2c byte %d. Ret: %d\n", value, ret);
   }
   return ret;
}

/**
 * Write value to register
 */
esp_err_t write_reg(uint8_t reg_addr, uint8_t value)
{
    write_byte(reg_addr);
    return write_byte(value);
}

/**
 * Read register
 */
uint8_t read_reg(uint8_t reg_addr)
{
  // Specify register to read
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte (cmd, (ACCEL<<1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte (cmd, reg_addr, ACK_CHECK_EN);
  esp_err_t ret = i2c_master_cmd_begin (I2C_MASTER_NUM, cmd, 100/ portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    printf("Error writing i2c address byte %d. Ret: %d\n", reg_addr, ret);
  } else {
    printf("Successfully wrote i2c address byte %d. Ret: %d\n", reg_addr, ret);
  }

  // Now read a byte from that register
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte (cmd, (ACCEL<<1) | I2C_MASTER_READ, ACK_CHECK_EN);
  uint8_t res = 0;
  i2c_master_read_byte (cmd, &res, NACK_VAL);
  ret = i2c_master_cmd_begin (I2C_MASTER_NUM, cmd, 100/ portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    printf("Error reading register %d: %d. Ret: %d\n", reg_addr, res, ret);
  } else {
    printf("Successfully read register %d: %d. Ret: %d\n", reg_addr, res, ret);
  }

  return res;
}



void loop(void *pvParameter) {
  while (1) {
    uint8_t id = read_reg(0x0F); //Read the WHO_AM_I register
    printf("i2c id: %d\n", id);
    fflush(stdout);

    gpio_set_level(LED_PIN, HIGH);
    vTaskDelay(500);
    gpio_set_level(LED_PIN, LOW);
    vTaskDelay(500);
  }
}


void app_main()
{
  init_i2c_device();

  printf("Accelerometer enabled\n");
  fflush(stdout);

  xTaskCreate(&loop, "loop", 2048, NULL, 5, NULL);
}

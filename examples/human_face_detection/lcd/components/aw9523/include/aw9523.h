#ifndef __AW9523_H__
#define __AW9523_H__

#include <stdio.h>
#include <i2cdev.h>

#define I2C_FREQ_HZ             100000
#define AW9523_ADDR 	        0x58

#define AW9523_LED_MODE 	    0x3
#define AW9523_REG_CHIPID 	    0x10
#define AW9523_REG_SOFTRESET 	0x7F
#define AW9523_REG_INPUT0 	    0x00  
#define AW9523_REG_OUTPUT0 	    0x02   
#define AW9523_REG_CONFIG0 	    0x04 
#define AW9523_REG_INTENABLE0 	0x06
#define AW9523_REG_GCR 	        0x11       
#define AW9523_REG_LEDMODE  	0x12
#define AW9523_REG_LED_DIM 	    0x20

esp_err_t aw9523_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t aw9523_free_desc(i2c_dev_t *dev);

esp_err_t aw9523_poweron_init(i2c_dev_t *dev);

esp_err_t aw9523_enable_cam_pwr(i2c_dev_t *dev);
esp_err_t aw9523_disable_cam_pwr(i2c_dev_t *dev);

esp_err_t aw9523_enable_bat_pwr(i2c_dev_t *dev);
esp_err_t aw9523_disable_bat_pwr(i2c_dev_t *dev);

esp_err_t aw9523_enable_bg95_pwr(i2c_dev_t *dev);
esp_err_t aw9523_disable_bg95_pwr(i2c_dev_t *dev);

esp_err_t aw9523_set_stu_led_brightness(i2c_dev_t *dev, uint8_t r, uint8_t g, uint8_t b);

esp_err_t aw9523_set_flashlight_brightness(i2c_dev_t *dev, uint8_t idx, uint8_t brightness);
esp_err_t aw9523_set_all_flashlight_brightness(i2c_dev_t *dev, uint8_t brightness);

#endif
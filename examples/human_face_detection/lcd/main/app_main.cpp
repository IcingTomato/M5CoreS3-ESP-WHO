#include "who_camera.h"
#include "who_human_face_detection.hpp"
// #include "who_cat_face_detection.hpp"
// #include "who_trace.h" //for cat face
#include "who_lcd.h"
#include "aw9523.h"

#define TAG "CORE-S3"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL;

static esp_err_t i2c_master_init(uint8_t sda_io_num, uint8_t scl_io_num);
static esp_err_t i2c_write(uint8_t addr, const uint8_t out_reg,
                           size_t out_reg_size, const uint8_t out_data,
                           size_t out_size);
static esp_err_t i2c_read(uint8_t addr, const uint8_t out_data, size_t out_size,
                          uint8_t *in_data, size_t in_size);
static void aw9523_init(void);
static void axp2101_turn_on_bl(void);

extern "C" void app_main()
{
    // // for v0.1 hardware => axp2101
    // if (i2c_master_init(33, 34) != ESP_OK) printf("Core-S3 i2c init
    // failed\n");

    // i2c init & scan
    vTaskDelay(30 / portTICK_RATE_MS);
    if (i2c_master_init(34, 33) != ESP_OK) printf("Core-S3 i2c init failed\n");

    axp2101_turn_on_bl();
    aw9523_init();

    printf("\r\nM5 Core S3 say \"Hello world!\"\r\n");

    printf("i2c scan: \n");
    for (uint8_t i = 1; i < 127; i++) {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Found device at: 0x%2x\n", i);
        }
    }
    printf("Core-S3 i2c scan done\n");

    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
    register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
    //register_cat_face_detection(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
    register_lcd(xQueueLCDFrame, NULL, true);
}

///////////////////////////////////////////////////////////////////////////////
static esp_err_t i2c_master_init(uint8_t sda_io_num, uint8_t scl_io_num) {
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = sda_io_num;
    conf.scl_io_num       = scl_io_num;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags        = 0;

    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write(uint8_t addr, const uint8_t out_reg,
                           size_t out_reg_size, const uint8_t out_data,
                           size_t out_size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1, true);
    if (out_reg && out_reg_size)
        i2c_master_write(cmd, &out_reg, out_reg_size, true);
    i2c_master_write(cmd, &out_data, out_size, true);
    i2c_master_stop(cmd);
    esp_err_t res =
        i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d", addr, 0,
                 res);
    i2c_cmd_link_delete(cmd);

    return res;
}

static esp_err_t i2c_read(uint8_t addr, const uint8_t out_data, size_t out_size,
                          uint8_t *in_data, size_t in_size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (out_data && out_size) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, addr << 1, true);
        i2c_master_write(cmd, &out_data, out_size, true);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | 1, true);
    i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd,
                                         pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", addr,
                 I2C_NUM_0, res);

    i2c_cmd_link_delete(cmd);
    return res;
}

static void aw9523_init(void) {
    // RST
    i2c_write(0x58, AW9523_REG_SOFTRESET, 1, 0x00, 1);
    vTaskDelay(30 / portTICK_RATE_MS);

    // P0 I/O Mode  0-OUTPUT 1-INPUT
    // P0_0 LED_R  OUTPUT  => 闪光灯
    // P0_1 BST_EN OUTPUT
    // P0_2 AW_RST OUTPUT   HIGH
    // P0_3 ES_INT INPUT
    // P0_4 TF_DET INPUT
    // P0_5 NC
    // P0_6 AXP_INT INPUT
    // P0_7 CAM_RST OUTPUT  HIGH
    // 0b01111000
    i2c_write(0x58, AW9523_REG_CONFIG0, 1, 0b01111000, 1);
    // P1 I/O Mode  0-OUTPUT 1-INPUT
    // P1_0 LED_B   OUTPUT => 闪光灯
    // P1_1 LED_W   OUTPUT => 闪光灯
    // P1_2 LED_G   OUTPUT => 闪光灯
    // P1_3 AW_INT  INPUT
    // P1_4 NC
    // P1_5 LCD_RST OUTPUT  HIGH
    // P1_6 TP_INT  INPUT
    // P1_7 TP_RST  OUTPUT  HIGH
    // 0b01011000
    i2c_write(0x58, AW9523_REG_CONFIG0 + 1, 1, 0b01011000, 1);
    // P0 LED/GPIO Mode
    // P0_0 LED Mode
    // P0_1 ~ P0_7 GPIO Mode
    // 0b11111110
    i2c_write(0x58, AW9523_REG_LEDMODE, 1, 0b11111110, 1);
    // P1 LED/GPIO Mode
    // P1_0 ~ P1_2 LED Mode
    // P1_3 ~ P1_7 GPIO Mode
    // 0b11111000
    i2c_write(0x58, AW9523_REG_LEDMODE + 1, 1, 0b11111000, 1);

    // GPIO Push-Pull mode
    i2c_write(0x58, AW9523_REG_GCR, 1, (1 << 4), 1);

    // P0 GPIO Default HIGH OUTPUT PIN
    // 0b10000100
    i2c_write(0x58, AW9523_REG_OUTPUT0, 1, 0b10000100,
              1);  // AW_RST  CAM_RST
    // P1 GPIO Default HIGH OUTPUT PIN
    // 0b10100000
    i2c_write(0x58, AW9523_REG_OUTPUT0 + 1, 1, 0b10100000,
              1);  // LCD_RST TP_RST
}

static void axp2101_turn_on_bl(void) {
    uint8_t cfg = 0;
    i2c_read(0x34, 0x90, 1, &cfg, 1);
    // enable DLDO1  -> BL
    // enable BLDO1  -> CAM AVDD
    // enable BLDO2  -> CAM DVDD
    i2c_write(0x34, 0x90, 1, (cfg | (1 << 7) | (1 << 4) | (1 << 5)), 1);

    // DLDO1  -> BL
    // from 0.5V to 3.5V 100mv/step
    // 0b00000  0.5V
    // 0b11110  3.5V
    i2c_write(0x34, 0x99, 1, (0b11110 - 2), 1);  // 3.3V

    // BLDO1 -> CAM AVDD 2.8V
    // from 0.5V to 3.5V 100mv/step
    // 0b00000  0.5V
    // 0b11110  3.5V
    i2c_write(0x34, 0x96, 1, (0b11110 - 7), 1);  // 2.8V

    // BLDO2 -> CAM DVDD 1.5V
    // from 0.5V to 3.5V 100mv/step
    // 0b00000  0.5V
    // 0b11110  3.5V
    i2c_write(0x34, 0x97, 1, (0b00000 + 7), 1);  // 1.2V
}

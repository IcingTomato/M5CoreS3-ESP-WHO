#include "aw9523.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static inline esp_err_t read_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(dev, reg, val, 1);
}

static inline esp_err_t write_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(dev, reg, &val, 1);
}

static esp_err_t update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    CHECK(read_reg_nolock(dev, reg, &v));
    CHECK(write_reg_nolock(dev, reg, (v & ~mask) | val));
    return ESP_OK;
}

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, reg, mask, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

////////////////////////////////////////////////////////////////////////////////
esp_err_t aw9523_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{

    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = AW9523_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t aw9523_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t aw9523_poweron_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_SOFTRESET, 0x00));
    vTaskDelay(30 / portTICK_RATE_MS);

    uint8_t aw9523_id;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_CHIPID, &aw9523_id));
    // if (aw9523_id != 0x23) {
    //     printf("Invalid id read from AW9523, abort init. %d\n", aw9523_id);
    //     return ESP_FAIL;
    // }

    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_CONFIG0, 0x00)); // Set all to output mode

    // Set all to ouput mode execpt for button to input mode
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_CONFIG0 + 1, 0x80));

    // Change to LED mode
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_LEDMODE, 0xF8));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_LEDMODE + 1, 0x88));

    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_GCR, (1 << 4))); // GPOOMD=1, Push-Pull
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, 0x00)); // P0 All Low
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0 + 1, 0x00)); // P1 All Low

    for (uint8_t i = 0x20; i < 0x2F; i++) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, i, 0x00)); // All led off
    }
    return ESP_OK;
}

esp_err_t aw9523_enable_cam_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg & ~(1 << 4); // BAT_PWR to 0, pull the NMOS
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_disable_cam_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg | (1 << 4); // BAT_PWR to 1, release the NMOS
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_enable_bat_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg | (1 << 7); // BAT_PWR to 1
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_disable_bat_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg  & ~(1 << 7); // BAT_PWR to 0
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_enable_bg95_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg & ~(1 << 5); // PWRKEY to 0, Release PWR Key
    p0_reg = p0_reg | (1 << 6); // PWR_EN to 1
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));

    vTaskDelay(100 / portTICK_RATE_MS);
    p0_reg = p0_reg | (1 << 5); // PWRKEY to 1, Pull PWR Key
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));

    vTaskDelay(500 / portTICK_RATE_MS);
    p0_reg = p0_reg & ~(1 << 5); // PWRKEY to 0, Release PWR Key
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_disable_bg95_pwr(i2c_dev_t *dev)
{
    uint8_t p0_reg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(dev, AW9523_REG_OUTPUT0, &p0_reg));
    p0_reg = p0_reg | (1 << 5); // PWRKEY to 1
    p0_reg = p0_reg & ~(1 << 6); // PWR_EN to 0
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_OUTPUT0, p0_reg));
    return ESP_OK;
}

esp_err_t aw9523_set_stu_led_brightness(i2c_dev_t *dev, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_LED_DIM + 0xE, r));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_LED_DIM + 0xD, g));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, AW9523_REG_LED_DIM + 0xC, b));
    return ESP_OK;
}

esp_err_t aw9523_set_flashlight_brightness(i2c_dev_t *dev, uint8_t idx, uint8_t brightness)
{
    const uint8_t idx_addr_lut[6] = {0x26, 0x20, 0x25, 0x21, 0x24, 0x22};
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, idx_addr_lut[idx], brightness));
    return ESP_OK;
}

esp_err_t aw9523_set_all_flashlight_brightness(i2c_dev_t *dev, uint8_t brightness)
{
    static const uint8_t idx_addr_lut[6] = {0x26, 0x20, 0x25, 0x21, 0x24, 0x22};
    for (uint8_t i = 0; i < 6; i++) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(dev, idx_addr_lut[i], brightness));
    }
    return ESP_OK;
}
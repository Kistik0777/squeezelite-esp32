/*
 * Basic interface code for the PI4IOE5V6408 GPIO Expander.
 * This should be written to use a generic API
 * the future if there is any interest in other devices.
 */
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "globdefs.h"
#include "platform_config.h"
#include "tools.h"
#include "driver/i2c.h"



#define GPIO_EX_TASK_SIZE             (2048)
#define GPIO_EX_TASK_PRIORITY         (configMAX_PRIORITIES - 1)


#define PI4IO_ADDR0 0x43
#define PI4IO_ADDR1 0x44


static const char * TAG = "gpio_expander";

#define PI4IO_CHIP_ID 0x1
#define PI4IO_INPUT_STATUS 0xF
#define PI4IO_INPUT_DEFAULT_STATE 0x09
#define PI4IO_PULL_ENABLE 0x0B
#define PI4IO_PULL_SELECT 0x0D
#define PI4IO_INTERRUPT_STATUS 0x13

#if 0
#define PI4IO_IO_DIRECTION 0x3
#define PI4IO_OUTPUT 0x5
#define PI4IO_OUTPUT_HI_IMPEDANCE 0x7


#define PI4IO_CHIP_ID_VAL 0xA0
#define PI4IO_CHIP_ID_MASK 0xFC
#define PI4IO_DIRECTION_TO_GPIOD(x) ((x) ? GPIOF_DIR_OUT : GPIOF_DIR_IN)
#define GPIOD_DIRECTION_TO_PI4IO(x) ((x) == GPIOF_DIR_OUT ? 1 : 0)
#define GPIO_OUT_LOW 0
#define GPIO_OUT_HIGH 1
static bool
pi4io_readable_reg(struct device* dev, unsigned int reg)
{
    // All readable registers are odd-numbered.
    return (reg % 2) == 1;
}
static bool
pi4io_writeable_reg(struct device* dev, unsigned int reg)
{
    // All odd-numbered registers are writable except for 0xF.
    if ((reg % 2) == 1) {
        if (reg != PI4IO_INPUT_STATUS) { return true; }
    }
    return false;
}
static bool
pi4io_volatile_reg(struct device* dev, unsigned int reg)
{
    if (reg == PI4IO_INPUT_STATUS || reg == PI4IO_INTERRUPT_STATUS) {
        return true;
    }
    return false;
}
static const struct regmap_config pi4io_regmap = {
    .reg_bits      = 8,
    .val_bits      = 8,
    .max_register  = 0x13,
    .writeable_reg = pi4io_writeable_reg,
    .readable_reg  = pi4io_readable_reg,
    .volatile_reg  = pi4io_volatile_reg,
};
struct pi4io_priv {
    struct i2c_client* i2c;
    struct regmap*     regmap;
    struct gpio_chip   gpio;
};
static int
pi4io_gpio_get_direction(struct gpio_chip* chip, unsigned offset)
{
    int                ret;
    int                io_dir;
    struct pi4io_priv* pi4io = gpiochip_get_data(chip);
    struct device*     dev   = &pi4io->i2c->dev;
    ret = regmap_read(pi4io->regmap, PI4IO_IO_DIRECTION, &io_dir);
    if (ret) {
        dev_err(dev, "Failed to read I/O direction: %d", ret);
        return ret;
    }
    return PI4IO_DIRECTION_TO_GPIOD((io_dir >> offset) & 1);
}
static int
pi4io_gpio_set_direction(struct gpio_chip* chip, unsigned offset, int direction)
{
    int                ret;
    struct pi4io_priv* pi4io = gpiochip_get_data(chip);
    struct device*     dev   = &pi4io->i2c->dev;
    ret                      = regmap_update_bits(pi4io->regmap,
                             PI4IO_IO_DIRECTION,
                             1 << offset,
                             GPIOD_DIRECTION_TO_PI4IO(direction) << offset);
    if (ret) {
        dev_err(dev, "Failed to set direction: %d", ret);
        return ret;
    }
    // We desire the hi-impedance state to track the output state.
    ret = regmap_update_bits(pi4io->regmap,
                             PI4IO_OUTPUT_HI_IMPEDANCE,
                             1 << offset,
                             direction << offset);
    return ret;
}
static int
pi4io_gpio_get(struct gpio_chip* chip, unsigned offset)
{
    int                ret;
    int                out;
    struct pi4io_priv* pi4io = gpiochip_get_data(chip);
    struct device*     dev   = &pi4io->i2c->dev;
    ret                      = regmap_read(pi4io->regmap, PI4IO_OUTPUT, &out);
    if (ret) {
        dev_err(dev, "Failed to read output: %d", ret);
        return ret;
    }
    if (out & (1 << offset)) { return 1; }
    return 0;
}
static void
pi4io_gpio_set(struct gpio_chip* chip, unsigned offset, int value)
{
    int                ret;
    struct pi4io_priv* pi4io = gpiochip_get_data(chip);
    struct device*     dev   = &pi4io->i2c->dev;
    ret                      = regmap_update_bits(pi4io->regmap,
                             PI4IO_OUTPUT,
                             1 << offset,
                             value << offset);
    if (ret) { dev_err(dev, "Failed to write output: %d", ret); }
}
static int
pi4io_gpio_direction_input(struct gpio_chip* chip, unsigned offset)
{
    return pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_IN);
}
static int
pi4io_gpio_direction_output(struct gpio_chip* chip, unsigned offset, int value)
{
    int                ret;
    struct pi4io_priv* pi4io = gpiochip_get_data(chip);
    struct device*     dev   = &pi4io->i2c->dev;
    ret = pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_OUT);
    if (ret) {
        dev_err(dev, "Failed to set direction: %d", ret);
        return ret;
    }
    pi4io_gpio_set(chip, offset, value);
    return 0;
}
static int
pi4io_gpio_setup(struct pi4io_priv* pi4io)
{
    int               ret;
    struct device*    dev = &pi4io->i2c->dev;
    struct gpio_chip* gc  = &pi4io->gpio;
    gc->ngpio             = 8;
    gc->label             = pi4io->i2c->name;
    gc->parent            = &pi4io->i2c->dev;
    gc->owner             = THIS_MODULE;
    gc->base              = -1;
    gc->can_sleep         = true;
    gc->get_direction     = pi4io_gpio_get_direction;
    gc->direction_input   = pi4io_gpio_direction_input;
    gc->direction_output  = pi4io_gpio_direction_output;
    gc->get               = pi4io_gpio_get;
    gc->set               = pi4io_gpio_set;
    ret                   = devm_gpiochip_add_data(dev, gc, pi4io);
    if (ret) {
        dev_err(dev, "devm_gpiochip_add_data failed: %d", ret);
        return ret;
    }
    return 0;
}


static int
pi4io_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    int                ret, chip_id;
    struct device*     dev = &client->dev;
    struct pi4io_priv* pi4io;
    pi4io = devm_kzalloc(dev, sizeof(struct pi4io_priv), GFP_KERNEL);
    if (!pi4io) { return -ENOMEM; }
    i2c_set_clientdata(client, pi4io);
    pi4io->i2c    = client;
    pi4io->regmap = devm_regmap_init_i2c(client, &pi4io_regmap);
    ret           = regmap_read(pi4io->regmap, PI4IO_CHIP_ID, &chip_id);
    if (ret < 0) {
        dev_err(dev, "Failed to read Chip ID: %d", ret);
        return ret;
    }
    if ((chip_id & PI4IO_CHIP_ID_MASK) != PI4IO_CHIP_ID_VAL) {
        dev_err(dev, "Invalid Chip ID!");
        return -EINVAL;
    }
    ret = pi4io_gpio_setup(pi4io);
    if (ret < 0) {
        dev_err(dev, "Failed to setup GPIOs: %d", ret);
        return ret;
    }
    dev_dbg(dev, "PI4IO probe finished");
    return 0;
}
#endif


/****************************************************************************************
 * gpio expander read register
 */
esp_err_t
gpio_ex_read(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data)
{
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd,
                          addr << 1 | I2C_MASTER_WRITE,
                          I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, reg, I2C_MASTER_ACK);

    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, addr << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(i2c_cmd, data, I2C_MASTER_NACK);

    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(port, i2c_cmd, 500 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPIO EX reg 0x%x, data 0x%x", reg, *data);
    }
    return ret;
}


/****************************************************************************************
 * gpio expander read register
 */
esp_err_t
gpio_ex_write(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t data)
{
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd,
                          addr << 1 | I2C_MASTER_WRITE,
                          I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, reg, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, data, I2C_MASTER_ACK);

    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(port, i2c_cmd, 500 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPIO EX wrote reg 0x%x, data 0x%x", reg, data);
    }
    return ret;
}



typedef struct {
    
    gpio_num_t gpio; // Must be less than GPIO_NUM_33
    SemaphoreHandle_t access_semaphore;
} gpio_expander_t;

static gpio_expander_t gpio_expander_config = {.gpio             = -1,
                                               .access_semaphore = NULL};

static void
gpio_expander_task(void* arg)
{
    gpio_expander_t* config = (gpio_expander_t*) arg;
    uint8_t data;

    while (1) {

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // xSemaphoreTake(led_strip->access_semaphore, portMAX_DELAY);

        /* read the interrupt register */
        gpio_ex_read(i2c_system_port, PI4IO_ADDR0, PI4IO_CHIP_ID, &data);
        gpio_ex_read(i2c_system_port, PI4IO_ADDR0, PI4IO_INPUT_STATUS, &data);
        gpio_ex_read(i2c_system_port, PI4IO_ADDR0, PI4IO_INTERRUPT_STATUS, &data);
        // xSemaphoreGive(led_strip->access_semaphore);
    }

    vTaskDelete(NULL);
}

/****************************************************************************************
 * GPIO expander low-level handler.  Called when there is a state change on the expander,
 * we need to figure out which expander gpio changed, and which pin on the expander it is
 * mapped to.
 */

static void IRAM_ATTR gpio_ex_isr_handler(void* arg)
{
//	struct button_s *button = (struct button_s*) arg;
//	BaseType_t woken = pdFALSE;
/*
	if (xTimerGetPeriod(button->timer) > button->debounce / portTICK_RATE_MS) xTimerChangePeriodFromISR(button->timer, button->debounce / portTICK_RATE_MS, &woken); // does that restart the timer? 
	else xTimerResetFromISR(button->timer, &woken);
	if (woken) portYIELD_FROM_ISR();
*/
	ESP_EARLY_LOGI(TAG, "INT gpio expander");
}

/****************************************************************************************
 * Set up the GPIO expander interrupt handler.
 */
void
set_expander_gpio(int gpio)
{
    ESP_LOGI(TAG, "In set expander");
    ESP_LOGI(TAG,
             "Installing expander interrupt on gpio %d with a pull up",
             gpio);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(gpio, gpio_ex_isr_handler, (void*) NULL);
    gpio_intr_enable(gpio);
}

static bool
create_gpio_ex_task(gpio_expander_t* config)
{
    /*
        if ((led_strip == NULL) ||
            (led_strip->rmt_channel == RMT_CHANNEL_MAX) ||
            (led_strip->gpio > GPIO_NUM_33) ||  // only inputs above 33
            (led_strip->led_strip_buf_1 == NULL) ||
            (led_strip->led_strip_buf_2 == NULL) ||
            (led_strip->led_strip_length == 0) ||
            (led_strip->access_semaphore == NULL)) {
            return false;
        }

        if(led_strip->led_strip_buf_1 == led_strip->led_strip_buf_2) {
            return false;
        }

        memset(led_strip->led_strip_buf_1, 0, sizeof(struct led_color_t) *
       led_strip->led_strip_length); memset(led_strip->led_strip_buf_2, 0,
       sizeof(struct led_color_t) * led_strip->led_strip_length);

        bool init_rmt = led_strip_init_rmt(led_strip);
        if (!init_rmt) {
            return false;
        }

        xSemaphoreGive(led_strip->access_semaphore);
    */
    BaseType_t task_created = xTaskCreate(gpio_expander_task,
                                          "gpio_expander_task",
                                          GPIO_EX_TASK_SIZE,
                                          config,
                                          GPIO_EX_TASK_PRIORITY,
                                          NULL);

    if (!task_created) { return false; }

    return true;
}


/****************************************************************************************
 * 
 */
void
gpio_expander_init()
{
    bool  init   = false;
    char* config = config_alloc_get_str("gpio_expander", NULL, "N/A");

    int   interrupt_gpio = -1;
    char* p;
    uint8_t data;

    /* in future look for drivers here */
    if ((p = strcasestr(config, "interrupt")) != NULL) {
        interrupt_gpio = atoi(strchr(p, '=') + 1);
    }
    // so far so good
    if (interrupt_gpio > 0 && interrupt_gpio < 40) {

        // Detect driver interface
        if (i2c_system_port != -1 &&
            gpio_ex_read(i2c_system_port, PI4IO_ADDR0, PI4IO_CHIP_ID, &data) ==
                ESP_OK) {
            init = true;
            ESP_LOGI(TAG,
                     "gpio expander is I2C with interrupt %d",
                     interrupt_gpio);
        } else {
            ESP_LOGI(TAG, "Error configuring gpio expander");
        }
    }

    if (init) {
        /* disable pull up/downs on gpios for now */
        gpio_ex_write(i2c_system_port,
                      PI4IO_ADDR0,
                      PI4IO_PULL_ENABLE,
                      0x20); // enable pull up on rotary switch only
        gpio_ex_write(i2c_system_port,
                      PI4IO_ADDR0,
                      PI4IO_PULL_SELECT,
                      0x20); // enable pull up on rotary switch only
        gpio_ex_write(i2c_system_port,
                      PI4IO_ADDR0,
                      PI4IO_INPUT_DEFAULT_STATE,
                      0x38); // enable pull up on rotary switch only

        /* install interrupt handlers and configure gpio (buttons?) here. */
        set_expander_gpio(interrupt_gpio);
        create_gpio_ex_task(&gpio_expander_config);
    }

    free(config);
}

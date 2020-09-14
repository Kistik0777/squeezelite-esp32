/*
 * Basic interface code for the MCP23017 GPIO Expander.
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


#define MCP23017_ADDR0 0x20

static const char * TAG = "gpio_expander";

#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_INPUT_STATUS 0xF
#define MCP23017_INPUT_DEFAULT_STATE 0x09
#define MCP23017_PULL_ENABLE 0x0B
#define MCP23017_PULL_SELECT 0x0D
#define MCP23017_INTERRUPT_STATUS 0x13


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
        /*
        gpio_ex_read(i2c_system_port, MCP23017_ADDR0, MCP23017_CHIP_ID, &data);
        gpio_ex_read(i2c_system_port, MCP23017_ADDR0, MCP23017_INPUT_STATUS, &data);
        gpio_ex_read(i2c_system_port, MCP23017_ADDR0, MCP23017_INTERRUPT_STATUS, &data);
    */
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

    if ((p = strcasestr(config, "interrupt")) != NULL) {
        interrupt_gpio = atoi(strchr(p, '=') + 1);
    }
    // so far so good
    if (interrupt_gpio > 0 && interrupt_gpio < 40) {

        ESP_LOGI(TAG, "gpio expander is I2C with interrupt %d", interrupt_gpio);
        // Detect driver interface
        if (i2c_system_port != -1 && gpio_ex_read(i2c_system_port,
                                                  MCP23017_ADDR0,
                                                  MCP23017_IODIRA,
                                                  &data) == ESP_OK) {
            if (data == 0xFF) {
                init = true;
                ESP_LOGI(TAG,
                         "gpio expander is I2C with interrupt %d",
                         interrupt_gpio);
            }
        }
    }

    if (0 && init) {
        /* disable pull up/downs on gpios for now */
        gpio_ex_write(i2c_system_port,
                      MCP23017_ADDR0,
                      MCP23017_PULL_ENABLE,
                      0x20); // enable pull up on rotary switch only
        gpio_ex_write(i2c_system_port,
                      MCP23017_ADDR0,
                      MCP23017_PULL_SELECT,
                      0x20); // enable pull up on rotary switch only
        gpio_ex_write(i2c_system_port,
                      MCP23017_ADDR0,
                      MCP23017_INPUT_DEFAULT_STATE,
                      0x38); // enable pull up on rotary switch only

        /* install interrupt handlers and configure gpio (buttons?) here. */
        set_expander_gpio(interrupt_gpio);
        create_gpio_ex_task(&gpio_expander_config);
    }

    free(config);
}

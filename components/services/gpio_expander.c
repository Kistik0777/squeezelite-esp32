/* 
 *  Some more crude button management based on the MCP23017 GPIO expander
 *
 *  (c) Chuck R. 2020, chuck@zethus.ca
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
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
#include "gpio_expander.h"



#define GPIO_EX_TASK_SIZE             (2048)
#define GPIO_EX_TASK_PRIORITY         (configMAX_PRIORITIES - 1)


#define MCP23017_ADDR0 0x20

static const char * TAG = "gpio_expander";

/* MCP23017 Register variables */
static const uint8_t REG_IODIR = 0x00;
// static const uint8_t REG_IPOL    = 0x2;
static const uint8_t REG_GPINTEN = 0x04;
// static const uint8_t REG_DEFVAL  = 0x6;
// static const uint8_t REG_INTCON  = 0x8;
static const uint8_t REG_IOCON = 0x0A;
static const uint8_t REG_GPPU  = 0x0C;
static const uint8_t REG_INTF  = 0x0E;
// static const uint8_t REG_INTCAP  = 0x10;
static const uint8_t REG_GPIO = 0x12;
// static const uint8_t REG_OLAT    = 0x14;

static const uint8_t REG_BIT_MIRROR = 0x06;

#define EXGPIO_MAX 16
typedef struct {
    TaskHandle_t task;
    gpio_isr_t   isr_handlers[EXGPIO_MAX];
    void*        isr_parm[EXGPIO_MAX];
} gpio_expander_t;

static gpio_expander_t gpio_expander_ctx = {
    .isr_handlers = {0},
    .isr_parm     = {0},
};

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
 //       ESP_LOGI(TAG, "GPIO EX read  reg 0x%x, data 0x%x", reg, *data);
    }
    return ret;
}


/****************************************************************************************
 * gpio expander write register
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

static void
gpio_expander_task(void* arg)
{
    gpio_expander_t* config = (gpio_expander_t*) arg;
    uint8_t          data;
    const TickType_t xBlockTime = pdMS_TO_TICKS(2000);
    uint32_t         ulNotifiedValue;

    while (1) {
        ulNotifiedValue = ulTaskNotifyTake(pdFALSE, xBlockTime);
        if (ulNotifiedValue > 0) {
            /* read the interrupt register */
            gpio_ex_read(i2c_system_port, MCP23017_ADDR0, REG_INTF, &data);
            if (data) {
                unsigned i   = 1;
                unsigned pos = 0;
                while (!(i & data)) {
                    i <<= 1;
                    ++pos;
                }
                //ESP_LOGI(TAG, "Detected a change on low port bit %d", pos);
                // figure out what button changed state, and deal with it
                if (config->isr_handlers[pos]) {
                    //gpio_ex_get_levels(false); /* reset the interrupt */
                    config->isr_handlers[pos](config->isr_parm[pos]);
                }
            }
            /* read the second bank */
            gpio_ex_read(i2c_system_port, MCP23017_ADDR0, REG_INTF + 1, &data);
            if (data) {
                unsigned i   = 1;
                unsigned pos = 8;
                while (!(i & data)) {
                    // Unset current bit and set the next bit in 'i'
                    i <<= 1;
                    // increment position
                    ++pos;
                }
                ESP_LOGI(TAG, "Detected a change on high port bit %d", pos);
                // figure out what button changed state, and deal with it
                if (config->isr_handlers[pos]) {
                    //gpio_ex_get_levels(false); /* reset the interrupt */
                    config->isr_handlers[pos](config->isr_parm[pos]);
                }
            }
        } else {
            /* Did not receive a notification within the expected
            time. */
            /* read the registers in case the interrupt is stuck */
            gpio_ex_get_levels(true);  /* reset the interrupt */
            gpio_ex_get_levels(false); /* reset the interrupt */
        }
    }
}

/****************************************************************************************
 * GPIO expander low-level handler.  Called when there is a state change on the
 * expander, This isr simply wakes the expander task to deal with the gpio
 * change event.
 */

static void IRAM_ATTR
gpio_ex_isr_handler(void* arg)
{
    gpio_expander_t *config = ((gpio_expander_t *) arg);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR( config->task, &woken );
    portYIELD_FROM_ISR( );
    //ESP_EARLY_LOGI(TAG, "INT gpio expander");
}

/****************************************************************************************
 * Set up the GPIO expander interrupt handler.
 */
void
set_expander_interrupt_gpio(uint8_t gpio, gpio_expander_t *config)
{
    if (gpio >= GPIO_NUM_MAX) {
        ESP_LOGI(TAG, "gpio out of range %d", gpio);
        return;
    }

    ESP_LOGI(TAG,
             "Installing expander interrupt on gpio %d with a pull up",
             gpio);
    //gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(gpio, GPIO_INTR_NEGEDGE);  /* interrupt on low only */
    gpio_isr_handler_add(gpio, gpio_ex_isr_handler, (void*) config);
    gpio_intr_enable(gpio);
}

static bool
create_gpio_ex_task(gpio_expander_t* config)
{

    BaseType_t task_created = xTaskCreate(gpio_expander_task,
                                          "gpio_expander_task",
                                          GPIO_EX_TASK_SIZE,
                                          config,
                                          GPIO_EX_TASK_PRIORITY,
                                          &config->task);

    if (!task_created) { return false; }

    return true;
}

/* This is the button API that mimics the internal gpio code */
void gpio_ex_pad_select_gpio(uint8_t gpio){
    //This does nothing
}
bool gpio_ex_verify_pin (uint8_t gpio){
    return (gpio < EXGPIO_MAX);
}

/* if mode matches target mode then set the bit for the gpio pin */

void
gpio_ex_reg_set(uint8_t gpio, uint8_t reg, uint8_t mode, uint8_t target_mode)
{
    uint8_t data;
    if (gpio_ex_verify_pin(gpio)) {
        if ( gpio > 7) {reg ++;} // use the second bank if we need to
        gpio_ex_read(i2c_system_port, MCP23017_ADDR0, reg, &data);
        if (mode == target_mode) {
            data |= (1 << (gpio & 0x07));
        } else {
            data &= ~(1 << (gpio & 0x07));
        }
        gpio_ex_write(i2c_system_port, MCP23017_ADDR0, reg, data);
    }
}

void
gpio_ex_set_direction(uint8_t gpio, uint8_t mode)
{
    gpio_ex_reg_set(gpio, REG_IODIR, mode, GPIO_MODE_INPUT);
}

void
gpio_ex_set_pull_mode(uint8_t gpio, uint8_t mode)
{
    gpio_ex_reg_set(gpio, REG_GPPU, mode, GPIO_PULLUP_ONLY);
}
void
gpio_ex_set_intr_type(uint8_t gpio, uint8_t mode)
{
    gpio_ex_reg_set(gpio, REG_GPINTEN, mode, GPIO_INTR_ANYEDGE);
    /* default for incon defval are used, i.e. interrupt is generated when data
     * is different from previous value */
}

uint8_t gpio_ex_get_level(uint8_t gpio)
{
    uint8_t data;
    if (gpio_ex_verify_pin(gpio)) {
        gpio_ex_read(i2c_system_port, MCP23017_ADDR0, (gpio > 7) ? REG_GPIO + 1
                     : REG_GPIO, &data);
        data >>= (gpio & 0x07);
        return data & 0x01;
    }
    return 0;
}

uint8_t gpio_ex_get_levels(bool upper_bank)
{
    uint8_t data;
    gpio_ex_read(i2c_system_port,
                 MCP23017_ADDR0,
                 (upper_bank) ? REG_GPIO + 1 : REG_GPIO,
                 &data);
    return data;
}

void
gpio_ex_isr_handler_add(gpio_num_t gpio, gpio_isr_t isr_handler, void* args)
{
    if (gpio_ex_verify_pin(gpio)) {
        gpio_expander_ctx.isr_handlers[gpio] = isr_handler;
        gpio_expander_ctx.isr_parm[gpio] = args;
        /* read both banks to clear registers */
        gpio_ex_get_levels(false);
        gpio_ex_get_levels(true);
    }
}

/****************************************************************************************
 *
 */
void
gpio_expander_init()
{
    bool  init   = false;
    char* config = config_alloc_get_str("gpio_expander", NULL, "N/A");

    int     interrupt_gpio = -1;
    char*   p;
    uint8_t data;

    /* get the interrupt pin for the gpio expander */
    if ((p = strcasestr(config, "interrupt")) != NULL) {
        interrupt_gpio = atoi(strchr(p, '=') + 1);
    }
    if (interrupt_gpio > 0 && interrupt_gpio < 40) {
        /* See if the chip is there */
        if (i2c_system_port != -1 &&
            gpio_ex_read(i2c_system_port, MCP23017_ADDR0, REG_IODIR, &data) ==
                ESP_OK) {
            /* this is only true out of reset, this may need to be
             * revisited */
            if (data == 0xFF) {
                init = true;
                ESP_LOGI(TAG,
                         "gpio expander is I2C with interrupt %d",
                         interrupt_gpio);
            }
        }
    }
    if (init) {
        /* install interrupt handlers and configure gpio (buttons?)
         * here. */
        create_gpio_ex_task(&gpio_expander_ctx);
        set_expander_interrupt_gpio(interrupt_gpio, &gpio_expander_ctx);

        /* mirror the interrupts */
        gpio_ex_reg_set(REG_BIT_MIRROR, REG_IOCON, 1, 1);
    }
  
    free(config);
}

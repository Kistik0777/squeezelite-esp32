/* 
 *  Some more crude button management based on the MCP23017 GPIO expander
 *
 *  (c) Chuck R. 2020, chuck@zethus.ca
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

/* This is the button API that mimics the internal gpio code */
void    gpio_ex_pad_select_gpio(uint8_t gpio);
void    gpio_ex_set_direction(uint8_t gpio, uint8_t mode);
void    gpio_ex_set_pull_mode(uint8_t gpio, uint8_t mode);
void    gpio_ex_set_intr_type(uint8_t gpio, uint8_t mode);
uint8_t gpio_ex_get_level(uint8_t gpio);
uint8_t gpio_ex_get_levels(bool upper_bank);
void    gpio_ex_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args);

/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef _NRF_24_CONF_H_
#define _NRF_24_CONF_H_
#include "main.h"

#define hspiX hspi1
#define spi_w_timeout 1000
#define spi_r_timeout 1000
#define spi_rw_timeout 1000

#define csn_gpio_port NRF_CSN_GPIO_Port
#define csn_gpio_pin NRF_CSN_Pin

#define ce_gpio_port NRF_SS_GPIO_Port
#define ce_gpio_pin NRF_SS_Pin

#endif


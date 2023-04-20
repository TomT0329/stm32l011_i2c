/** @file   main.h */

#ifndef MAIN_H
#define MAIN_H

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>


/*setting timer*/
#define GOAL_FREQUENCY (1) /* Goal frequency in Hz. */
#define TIMER_CLOCK (rcc_apb1_frequency * 1) /* f_timer. */
#define COUNTER_CLOCK (1000) /* f_counter (CK_CNT). */
#define TIMER_PRESCALER (TIMER_CLOCK / COUNTER_CLOCK - 1) /* PSC */
#define TIMER_PERIOD (((TIMER_CLOCK) / ((TIMER_PRESCALER + 1) * GOAL_FREQUENCY)) - 1) /* ARR */


/*setting I2C*/
#define I2C_SLAVE_ADDRESS ((uint8_t)0x50)
#define RCC_I2C_GPIO (RCC_GPIOB)
#define GPIO_I2C_PORT (GPIOB)
#define GPIO_I2C_SCL_PIN (GPIO6) /* D5. */
#define GPIO_I2C_SDA_PIN (GPIO7) /* D4. */
#define GPIO_I2C_AF (GPIO_AF1)   /* Ref: Table-14 in stm32l011d4. */

/*setting USART*/
#define USART_BAUDRATE (9600)
#define RCC_USART_TXRX_GPIO (RCC_GPIOA)
#define GPIO_USART_TXRX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (A7). */
#define GPIO_USART_RX_PIN (GPIO3) /* ST-Link (A2). */
#define GPIO_USART_AF (GPIO_AF4)  /* Table-14 in stm32l011d4 */

/*setting GPIO*/
#define RCCLEDPORT (RCC_GPIOB)
#define LEDPORT (GPIOB)
#define LEDPIN (GPIO3)

/*function prototype*/
static void rcc_setup(void);
static void i2c_setup(void);
static void timer_setup(void);
static void gpio_setup(void);
static void usart_setup(void);
static void delay(uint32_t value);
int _write(int file, char *ptr, int len);
#endif /* MAIN_H. */
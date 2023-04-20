/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <main.h>

/*Gobal parameters for debugging*/
int count = 0;
uint8_t cmd;

static void rcc_setup(void)
{
  /*SET the sysclk and prescale*/
  rcc_set_sysclk_source(RCC_HSI16);
  rcc_set_hpre(2);
  rcc_set_ppre1(1);
  rcc_set_ppre1(1);

  /*enable the periph*/
  rcc_periph_clock_enable(RCC_I2C_GPIO);
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_USART_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCCLEDPORT);
  // rcc_periph_clock_enable(RCC_TIM2);
  // rcc_periph_reset_pulse(RST_TIM2);
}

static void i2c_setup(void)
{
  /* Set SCL & SDA pin to open-drain alternate function. */
  gpio_mode_setup(GPIO_I2C_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_output_options(GPIO_I2C_PORT,
                          GPIO_OTYPE_OD,
                          GPIO_OSPEED_50MHZ,
                          GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_af(GPIO_I2C_PORT,
              GPIO_I2C_AF,
              GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  uint32_t i2c = I2C1;

  i2c_peripheral_disable(i2c);
  i2c_reset(i2c);

  i2c_set_speed(i2c,
                i2c_speed_fm_400k,         /* 400 kHz Fast mode. */
                rcc_apb1_frequency / 1e6); /* I2C clock in MHz. */

  i2c_peripheral_enable(i2c);                                  
}

static void timer_setup(void)
{
  timer_set_mode(TIM2,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);

  timer_set_prescaler(TIM2, TIMER_PRESCALER); /* Setup TIMx_PSC register. */
  timer_set_period(TIM2, TIMER_PERIOD);       /* Setup TIMx_ARR register. */
  
  /* Setup interrupt. */
  timer_enable_irq(TIM2, TIM_DIER_UIE);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  
  timer_enable_counter(TIM2);
}


static void gpio_setup(void)
{
	/* Set pin to 'output push-pull'. */
	/* Using API functions: */
  gpio_mode_setup(LEDPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LEDPIN);
  gpio_set_output_options(LEDPORT,
                        GPIO_OTYPE_PP,
                        GPIO_OSPEED_2MHZ,
                        LEDPIN);
}


static void usart_setup(void)
{
  /* Set USART-Tx pin to alternate function. */
  gpio_mode_setup(GPIO_USART_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  gpio_set_af(GPIO_USART_TXRX_PORT,
              GPIO_USART_AF,
              GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  /* Setup interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);
  usart_enable_rx_interrupt(USART2);            

  /* Congif USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX); 

  usart_enable(USART2);
}


 static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}



/*Replace the weak fun "printf"*/
int _write(int file, char *ptr, int len)
{
  /*The first three file descriptors, 0, 1, and 2, are reserved
  for standard input, standard output, and standard error, respectively.*/
  int i;

  if (file == 1)
  {
    for (i = 0; i < len; i++)
    {
      usart_send_blocking(USART2, ptr[i]);
    }
    return i;
  }

  /*errno is a gobal variable which is defined to report
  errors that occur during the execution of certain library functions.
  EI0->input/output error
  */
  errno = EIO;
  return -1;
}

int main(void)
{
  rcc_setup();
  i2c_setup();
  //timer_setup();  
  gpio_setup();
  usart_setup();
  while (1)
  {/*do nothing*/}

  return 0;
}

/**
 * @brief Timer2 Interrupt service routine.
 */
void tim2_isr(void)
{
  if (timer_get_flag(TIM2, TIM_SR_UIF)) /* Check 'Update interrupt flag'. */
  {
    timer_clear_flag(TIM2, TIM_SR_UIF);
    gpio_toggle(LEDPORT, LEDPIN); /* LED on/off. */
    printf("Hello World! %i\r\n", count++);
  }
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{ 
  // gpio_set(LEDPORT, LEDPIN); /* LED on. */
  // uint8_t indata = usart_recv(USART2); /* Read. */
  // usart_send_blocking(USART2, indata); /* Send. */
  // gpio_clear(LEDPORT, LEDPIN); /* LED off. */

  // /*i2c*/
  usart_disable_rx_interrupt(USART2);

  cmd = usart_recv(USART2);
  if (cmd == 'a') /* Write command. */
  {
    count++;
    uint8_t i2c_rx_data[1];
    uint8_t i2c_tx_data[3];
    i2c_tx_data[0] = usart_recv_blocking(USART2); /* Address 1. */
    i2c_tx_data[1] = usart_recv_blocking(USART2); /* Address 2. */
    i2c_tx_data[2] = usart_recv_blocking(USART2); /* Data. */

    i2c_transfer7(I2C1,
                  I2C_SLAVE_ADDRESS,
                  i2c_tx_data, /* Tx data array. */
                  3,           /* Tx data length. */
                  i2c_rx_data, /* Rx data array. */
                  0);          /* Rx data lenght. */

    usart_send_blocking(USART2, 0xF0); /* Write done ACK. */
  }
  else if (cmd == 0x01) /* Read command. */
  {
    uint8_t i2c_rx_data[1];
    uint8_t i2c_tx_data[2];
    i2c_tx_data[0] = usart_recv_blocking(USART2); /* Address 1. */
    i2c_tx_data[1] = usart_recv_blocking(USART2); /* Address 2. */

    i2c_transfer7(I2C1,
                  I2C_SLAVE_ADDRESS,
                  i2c_tx_data, /* Tx data array. */
                  2,           /* Tx data length. */
                  i2c_rx_data, /* Rx data array. */
                  1);          /* Rx data lenght. */

    usart_send_blocking(USART2, i2c_rx_data[0]);
  }
  else /* Unknown command. */
  {
    usart_send_blocking(USART2, 0xFF); 
  }

  usart_enable_rx_interrupt(USART2);

  USART_ISR(USART2) &= ~USART_ISR_RXNE; /* Clear 'Read data register not empty' flag. */
}


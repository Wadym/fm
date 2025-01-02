/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/printk.h>
//#include <zephyr/kernel.h>

//#include <zephyr/drivers/uart.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <string.h>
#include <stdbool.h>
//#include <zephyr/device.h>

#include <globals.h>

/*** Function Prototypes ************************************************************/
void uart6_main();
static void serial_cb(const struct device *dev, void *user_data);
static void print_uart(char *buf);

#define UART_BUFFER_SIZE 128       // Software buffer size for UART
#define CMD_BUFFER_SIZE 64         // Command buffer size

#define LOG_MODULE_NAME uart6

#define START_DELAY     500
#define STACK_SIZE      500
//#define PRIORITY        -5

/*** Constants **********************************************************************/
//#define LOG_MODULE_NAME sys
//LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

//exteral struct k_msgq data_message_q;

#define UART_DEVICE_NODE DT_NODELABEL(lpuart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);


/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/* 1000 msec = 1 sec */
//#define SLEEP_TIME_MS   100

//long sleepTime = 100;

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 50000

/*** Macros *************************************************************************/
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

K_THREAD_DEFINE(uart6_id, STACK_SIZE, uart6_main, NULL, NULL, NULL,
PRIORITY, 0, START_DELAY);

/*** Function implementation ********************************************************/

void uart6_main(void) {

  char tx_buf[MSG_SIZE];

  if (!device_is_ready(uart_dev)) {
    printk("UART device not found!");
    return;
  }

  uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
  uart_irq_rx_enable(uart_dev);

  print_uart("Hello! I'm your echo bot.\r\n");
  print_uart("Tell me something and press enter:\r\n");


  while (1) {
    k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
    print_uart("Echo: ");
    print_uart(tx_buf);
    print_uart("\r\n");
  }
}


/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
static void serial_cb(const struct device *dev, void *user_data)
{
  uint8_t c;

  if (!uart_irq_update(uart_dev)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }

  /* read until FIFO empty */
  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
      /* terminate string */
      rx_buf[rx_buf_pos] = '\0';

      /* if queue is full, message is silently dropped */
      k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

      /* reset the buffer (it was copied to the msgq) */
      rx_buf_pos = 0;
    } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
      rx_buf[rx_buf_pos++] = c;
    }
    /* else: characters beyond buffer size are dropped */
  }
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
static void print_uart(char *buf)
{
  int msg_len = strlen(buf);

  for (int i = 0; i < msg_len; i++) {
    uart_poll_out(uart_dev, buf[i]);
  }
}

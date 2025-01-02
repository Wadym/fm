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
#include <zephyr.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <string.h>
#include <stdbool.h>

//#include <globals.h>

#define UART_BUFFER_SIZE 128       // Software buffer size for UART
#define CMD_BUFFER_SIZE 64         // Command buffer size

/*** Constants **********************************************************************/
#define LOG_MODULE_NAME sys
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

struct k_msgq data_message_q;

#define UART_DEVICE_NODE DT_NODELABEL(lpuart6)
const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

//#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
//K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
//static char rx_buf[MSG_SIZE];
//static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
//void serial_cb(const struct device *dev, void *user_data)
//{
//  uint8_t c;
//
//  if (!uart_irq_update(uart_dev)) {
//    return;
//  }
//
//  if (!uart_irq_rx_ready(uart_dev)) {
//    return;
//  }
//
//  /* read until FIFO empty */
//  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
//    if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
//      /* terminate string */
//      rx_buf[rx_buf_pos] = '\0';
//
//      /* if queue is full, message is silently dropped */
//      k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
//
//      /* reset the buffer (it was copied to the msgq) */
//      rx_buf_pos = 0;
//    } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
//      rx_buf[rx_buf_pos++] = c;
//    }
//    /* else: characters beyond buffer size are dropped */
//  }
//}

/*
 * Print a null-terminated string character by character to the UART interface
 */
//void print_uart(char *buf)
//{
//  int msg_len = strlen(buf);
//
//  for (int i = 0; i < msg_len; i++) {
//    uart_poll_out(uart_dev, buf[i]);
//  }
//}


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

long sleepTime = 100;
static bool sw = false;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0    ""
// built in led pin 13
#define PIN    13
#define FLAGS    0
#endif

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 500

/*
 * @param my_name      thread identification string
 * @param my_sem       thread's own semaphore
 * @param other_sem    other thread's semaphore
 */
void helloLoop(const char *my_name,
               struct k_sem *my_sem, struct k_sem *other_sem) {
  const char *tname;
  uint8_t cpu;
  struct k_thread *current_thread;

  while (1) {
    /* take my semaphore */
    k_sem_take(my_sem, K_FOREVER);

    current_thread = k_current_get();
    tname = k_thread_name_get(current_thread);
#if CONFIG_SMP
    cpu = arch_curr_cpu()->id;
#else
    cpu = 0;
#endif
    /* say "hello" */
    if (tname == NULL) {
      printk("%s: Hello World from cpu %d on %s!\n",
             my_name, cpu, CONFIG_BOARD);
      sleepTime = 50;
    } else {
      printk("%s: Hello World from cpu %d on %s!\n",
             tname, cpu, CONFIG_BOARD);
      sleepTime = sw ? 500 : 700;
      sw = !sw;

    }

    /* wait a while, then let other thread have a turn */
    k_busy_wait(2000);
    k_msleep(SLEEPTIME);
    k_sem_give(other_sem);
  }
}

/* define semaphores */

K_SEM_DEFINE(threadA_sem, 1, 1);    /* starts off "available" */
K_SEM_DEFINE(threadB_sem, 0, 1);    /* starts off "not available" */

/* threadB is a dynamic thread that is spawned by threadA */

void threadB(void *dummy1, void *dummy2, void *dummy3) {
  ARG_UNUSED(dummy1);
  ARG_UNUSED(dummy2);
  ARG_UNUSED(dummy3);

  /* invoke routine to ping-pong hello messages with threadA */
  helloLoop(__func__, &threadB_sem, &threadA_sem);
}

K_THREAD_STACK_DEFINE(threadB_stack_area, STACKSIZE);
static struct k_thread threadB_data;

/* threadA is a static thread that is spawned automatically */

void threadA(void *dummy1, void *dummy2, void *dummy3) {
  ARG_UNUSED(dummy1);
  ARG_UNUSED(dummy2);
  ARG_UNUSED(dummy3);

  /* spawn threadB */
  k_tid_t tid = k_thread_create(&threadB_data, threadB_stack_area,
                                STACKSIZE, threadB, NULL, NULL, NULL,
                                PRIORITY, 0, K_FOREVER);

  k_thread_name_set(tid, "thread_b");
#if CONFIG_SCHED_CPU_MASK
  k_thread_cpu_mask_disable(&threadB_data, 1);
  k_thread_cpu_mask_enable(&threadB_data, 0);
#endif
  k_thread_start(&threadB_data);

  /* invoke routine to ping-pong hello messages with threadB */
  helloLoop(__func__, &threadA_sem, &threadB_sem);
}

K_THREAD_DEFINE(thread_a, STACKSIZE, threadA, NULL, NULL, NULL, PRIORITY, 0, 0);

const struct device *dev;
bool led_is_on = true;
int ret;

void main(void) {

//  char tx_buf[MSG_SIZE];
//
//  if (!device_is_ready(uart_dev)) {
//    printk("UART device not found!");
//    return;
//  }

  /* configure interrupt and callback to receive data */
//uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  //uart_irq_rx_enable(uart_dev);
//
//  print_uart("Hello! I'm your echo bot.\r\n");
//  print_uart("Tell me something and press enter:\r\n");




  dev = device_get_binding(LED0);
  if (dev == NULL) {
    return;
  }

  ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
  if (ret < 0) {
    return;
  }

  while (1) {
    gpio_pin_set(dev, PIN, (int) led_is_on);
    led_is_on = !led_is_on;
    //k_msleep(SLEEP_TIME_MS);
    k_msleep(sleepTime);

//    k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
//    print_uart("Echo: ");
//    print_uart(tx_buf);
//    print_uart("\r\n");
  }
}



#ifdef UART_AVAILABLE
// UART callback function
void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
  switch (evt->type) {
    case UART_RX_RDY:
      for (size_t i = 0; i < evt->data.rx.len; i++) {
        char c = evt->data.rx.buf[evt->data.rx.offset + i];
        if (c == '\n' || c == '\r') {
          // End of command
          cmd_buffer[cmd_pos] = '\0';  // Null-terminate the command
          process_command(cmd_buffer); // Process the received command
          cmd_pos = 0;                 // Reset command buffer position
        } else if (cmd_pos < CMD_BUFFER_SIZE - 1) {
          // Add character to the command buffer
          cmd_buffer[cmd_pos++] = c;
        } else {
          // Command buffer overflow
          printk("Command buffer overflow\n");
          cmd_pos = 0;  // Reset buffer
        }
      }
      break;

    case UART_RX_BUF_REQUEST:
      uart_rx_buf_rsp(dev, uart_rx_buffer, sizeof(uart_rx_buffer));
      break;

    case UART_RX_DISABLED:
      uart_rx_enable(dev, uart_rx_buffer, sizeof(uart_rx_buffer), 100);
      break;

    default:
      break;
  }
}

// Function to process received commands
void process_command(const char *cmd) {
  printk("Received command: %s\n", cmd);

  if (strcmp(cmd, "LED ON") == 0) {
    printk("Turning LED ON\n");
    // Add code to turn on the LED here
  } else if (strcmp(cmd, "LED OFF") == 0) {
    printk("Turning LED OFF\n");
    // Add code to turn off the LED here
  } else if (strcmp(cmd, "STATUS?") == 0) {
    printk("Returning STATUS\n");
    // Add code to send device status here
  } else {
    printk("Unknown command: %s\n", cmd);
  }
}
#endif
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/devicetree.h>


#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>


#include <zephyr/sys/printk.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <string.h>
#include <stdbool.h>

#include "globals.h"


/* size of stack area used by each thread */
#define STACKSIZE 1024
#define UART_BUFFER_SIZE 128       // Software buffer size for UART
#define CMD_BUFFER_SIZE 64         // Command buffer size

/*** Constants **********************************************************************/
#define LOG_MODULE_NAME sys
//LOG_MODULE_REGISTER(main);
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

struct k_msgq data_message_q;

#define UART_DEVICE_NODE DT_NODELABEL(lpuart6)
const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100
#define SLEEPTIME 500

/* scheduling priority used by each thread */
#define PRIORITY 7

long sleepTime = 100;
static bool sw = false;

/* The devicetree node identifier for the "led0" alias. */
//#define LED0_NODE DT_ALIAS(led0)
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});

/* SBUS UART */
#define SBUS_UART_NODE DT_NODELABEL(lpuart3)
static const struct device *sbus_uart = DEVICE_DT_GET(SBUS_UART_NODE);
/* SBUS buffer (25 bytes) */
#define SBUS_FRAME_SIZE 25
static uint8_t sbus_buf[SBUS_FRAME_SIZE];

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

//static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED_NODE, gpios, {0});

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
//static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


/* SBUS reading thread */
void sbus_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (!device_is_ready(sbus_uart)) {
        LOG_INF("Waiting for SBUS UART...");
        k_msleep(100);
    }

    LOG_INF("SBUS UART ready");

    while (1) {
        /* Blocking read for a full SBUS frame */
        int n = uart_fifo_read(sbus_uart, sbus_buf, SBUS_FRAME_SIZE);
        if (n == SBUS_FRAME_SIZE) {
            /* Parse channels 0-15 (11 bits each) */
            uint16_t ch[16] = {0};

            ch[0]  = ((sbus_buf[1]     | sbus_buf[2]<<8) & 0x07FF);
            ch[1]  = ((sbus_buf[2]>>3  | sbus_buf[3]<<5) & 0x07FF);
            ch[2]  = ((sbus_buf[3]>>6  | sbus_buf[4]<<2 | sbus_buf[5]<<10) & 0x07FF);
            ch[3]  = ((sbus_buf[5]>>1  | sbus_buf[6]<<7) & 0x07FF);
            ch[4]  = ((sbus_buf[6]>>4  | sbus_buf[7]<<4) & 0x07FF);
            ch[5]  = ((sbus_buf[7]>>7  | sbus_buf[8]<<1 | sbus_buf[9]<<9) & 0x07FF);
            ch[6]  = ((sbus_buf[9]>>2  | sbus_buf[10]<<6) & 0x07FF);
            ch[7]  = ((sbus_buf[10]>>5 | sbus_buf[11]<<3) & 0x07FF);
            ch[8]  = ((sbus_buf[12]    | sbus_buf[13]<<8) & 0x07FF);
            ch[9]  = ((sbus_buf[13]>>3 | sbus_buf[14]<<5) & 0x07FF);
            ch[10] = ((sbus_buf[14]>>6 | sbus_buf[15]<<2 | sbus_buf[16]<<10) & 0x07FF);
            ch[11] = ((sbus_buf[16]>>1 | sbus_buf[17]<<7) & 0x07FF);
            ch[12] = ((sbus_buf[17]>>4 | sbus_buf[18]<<4) & 0x07FF);
            ch[13] = ((sbus_buf[18]>>7 | sbus_buf[19]<<1 | sbus_buf[20]<<9) & 0x07FF);
            ch[14] = ((sbus_buf[20]>>2 | sbus_buf[21]<<6) & 0x07FF);
            ch[15] = ((sbus_buf[21]>>5 | sbus_buf[22]<<3) & 0x07FF);

            //LOG_INF("SBUS channels: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
            //    ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
            //    ch[8], ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15]);
            LOG_INF("SBUS channels: %u %u %u", ch[0], ch[1], ch[2]);
        }
        k_msleep(10000); //10
    }
}

K_THREAD_DEFINE(sbus_tid, 1024, sbus_thread, NULL, NULL, NULL, -9, 0, 0);
//K_THREAD_DEFINE(sbus_tid, 1024, sbus_thread, NULL, NULL, NULL, -9, 0, 0);




int main(void)
{
	// int ret;
	// bool led_state = true;

	// if (!gpio_is_ready_dt(&led)) {
	// 	return 0;
	// }

	// ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	// if (ret < 0) {
	// 	return 0;
	// }

	// while (1) {
	// 	ret = gpio_pin_toggle_dt(&led);
	// 	if (ret < 0) {
	// 		return 0;
	// 	}

	// 	led_state = !led_state;
	// 	printf("LED state: %s\n", led_state ? "ON" : "OFF");
	// 	LOG_INF("LED state: %s", led_state ? "ON" : "OFF");
	// 	k_msleep(SLEEP_TIME_MS);
	// }
	// return 0;

	int ret;
    // led.c added: bool led_state = true;

    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED device not ready");
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED");
        return 0;
    }

    while (1) {
        // led.c added: ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            LOG_ERR("Failed to toggle LED");
            return 0;
        }

        // led.c added: led_state = !led_state;
        // led.c added: LOG_INF("LED state: %s", led_state ? "ON" : "OFF");

        k_msleep(SLEEP_TIME_MS*100);
    }
}

/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    cmt2300a.h
 * @brief   CMT2300A transceiver RF chip driver
 *
 * @version 1.3
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#ifndef __CMT2300A_H
#define __CMT2300A_H
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include<linux/ktime.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include "cmt2300a.h"
#include "cmt2300a_defs.h"
#include "cmt2300a_params.h"


#define BOOL bool
#define TRUE true
#define FALSE false
#define INFINITE 0xFFFFFFFF
#define RF_PACKET_SIZE 32               /* Define the payload size here */

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;


typedef enum {
    RF_STATE_IDLE = 0,
    RF_STATE_RX_START,
    RF_STATE_RX_WAIT,
    RF_STATE_RX_DONE,
    RF_STATE_RX_TIMEOUT,
    RF_STATE_TX_START,
    RF_STATE_TX_WAIT,
    RF_STATE_TX_DONE,
    RF_STATE_TX_TIMEOUT,
    RF_STATE_ERROR,
} EnumRFStatus;

/* RF process function results */
typedef enum {
    RF_IDLE = 0,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_ERROR,
} EnumRFResult;


int cmt2300a_en;
int cmt2300a_clk;
int cmt2300a_sdio;
int cmt2300a_csb;
int cmt2300a_fcsb;
int cmt2300a_gpio1;
int cmt2300a_gpio2;
int cmt2300a_gpio3;

static u16 g_nRecvCount = 0;
static u16 g_nSendCount = 0;
static u16 g_nErrCount  = 0;

static EnumRFStatus g_nNextRFState = RF_STATE_IDLE;
static u8* g_pRxBuffer = NULL;
static u8* g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;
static struct completion tx_done_complete;
static struct completion rx_done_complete;
struct wakeup_source tx_wake_lock;
struct wakeup_source rx_wake_lock;
static struct task_struct *cmt2300a_send_thread=NULL;
static struct task_struct *cmt2300a_receive_thread=NULL;
static long long cmt2300a_send_count;
static long long cmt2300a_receive_count;
static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static ktime_t g_nRxTimeCount = 0;
static ktime_t g_nTxTimeCount = 0;
static u8 g_rxBuffer[RF_PACKET_SIZE];   /* RF Rx buffer */
static u8 g_txBuffer[RF_PACKET_SIZE];   /* RF Tx buffer */

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */

#define CMT2300A_SetGpio1In()           gpio_direction_input(cmt2300a_gpio1)
#define CMT2300A_SetGpio2In()           gpio_direction_input(cmt2300a_gpio2)
#define CMT2300A_SetGpio3In()           gpio_direction_input(cmt2300a_gpio3)
#define CMT2300A_ReadGpio1()            gpio_get_value(cmt2300a_gpio1)
#define CMT2300A_ReadGpio2()            gpio_get_value(cmt2300a_gpio2)
#define CMT2300A_ReadGpio3()            gpio_get_value(cmt2300a_gpio3)

//#define cmt_spi3_csb_out()      gpio_direction_output(cmt2300a_csb,1)
//#define cmt_spi3_fcsb_out()     gpio_direction_output(cmt2300a_fcsb,1)
//#define cmt_spi3_sclk_out()     gpio_direction_output(cmt2300a_clk,1)
//#define cmt_spi3_sdio_out()     gpio_direction_output(cmt2300a_sdio,1)
#define cmt_spi3_sdio_in()      gpio_direction_input(cmt2300a_sdio)

#define cmt_spi3_csb_1()        gpio_direction_output(cmt2300a_csb,1)
#define cmt_spi3_csb_0()        gpio_direction_output(cmt2300a_csb,0)

#define cmt_spi3_fcsb_1()       gpio_direction_output(cmt2300a_fcsb,1)
#define cmt_spi3_fcsb_0()       gpio_direction_output(cmt2300a_fcsb,0)

#define cmt_spi3_sclk_1()       gpio_direction_output(cmt2300a_clk,1)
#define cmt_spi3_sclk_0()       gpio_direction_output(cmt2300a_clk,0)

#define cmt_spi3_sdio_1()       gpio_direction_output(cmt2300a_sdio,1)
#define cmt_spi3_sdio_0()       gpio_direction_output(cmt2300a_sdio,0)
#define cmt_spi3_sdio_read()    gpio_get_value(cmt2300a_sdio)
#define CMT2300A_DelayMs(ms)	mdelay(ms)
#define CMT2300A_GetTickCount() ktime_get()
#define g_nSysTickCount		ktime_get()
#endif

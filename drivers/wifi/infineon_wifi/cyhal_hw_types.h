/***********************************************************************************************//**
 * \file cyhal_hw_types_template.h
 *
 * \brief
 * Provides a template for configuration resources used by the HAL. Items
 * here need to be implemented for each HAL port. It is up to the environment
 * being ported into what the actual types are. There are some suggestions below
 * but these are not required. All that is required is that the type is defined;
 * it does not matter to the HAL what type is actually chosen for the
 * implementation
 * All TODOs and references to 'PORT' need to be replaced by with meaningful
 * values for the device being supported.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2021 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************/

/**
 * \addtogroup group_hal_hw_types PORT Hardware Types
 * \ingroup group_hal_PORT
 * \{
 * Struct definitions for configuration resources in the PORT.
 *
 * \defgroup group_hal_hw_types_data_structures Data Structures
 */

#pragma once

#include <stdbool.h>
#include <string.h>

//ANN added below
#include "stm32_cyhal_common.h"
//#if defined (TARGET_STM32F4xx)
#include "stm32f4xx_hal_conf.h" //ANN added for f4 & commented for U5
//#elif defined (TARGET_STM32U5xx)
//#include "stm32u5xx_hal_conf.h"
//#endif

//#include "cybsp.h" //ANN commented this
//#include "cybsp_wifi.h" //ANN added this due compile errors
#include "stm32_cyhal_gpio_pin.h"
//ANN added below 1 line to remove LPTIM_HandleTypeDef undefined error. stm32f4 not having lptim
#undef HAL_LPTIM_MODULE_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup group_hal_hw_types_data_structures
 * \{
 */

/***************************************************************************************************
 *      Private macros
 **************************************************************************************************/

#if !defined(CYHAL_UART_RX_DMA_BUFFER_SIZE)
    #define CYHAL_UART_RX_DMA_BUFFER_SIZE         (512UL)
#endif /* !defined(CYHAL_UART_RX_DMA_BUFFER_SIZE) */

#if !defined(CYHAL_UART_TX_DMA_BUFFER_SIZE)
    #define CYHAL_UART_TX_DMA_BUFFER_SIZE         (512UL)
#endif /* !defined(CYHAL_UART_TX_DMA_BUFFER_SIZE) */

//ANN: added below; this should be assinged as a default priority number of Zephyr & can be located in this file or in cyhal_drivers.h
#define CYHAL_ISR_PRIORITY_DEFAULT 2 
// ANN added below (although it's defined in stm32f4xx_hal_conf.h, it's not reflecting here, so...)
//#define HAL_SD_MODULE_ENABLED

/***************************************************************************************************
 *      Types
 **************************************************************************************************/
//ANN commented below temp
//typedef int32_t  cyhal_clock_t;

/* SPI object */
typedef struct
{
    #if defined(HAL_SPI_MODULE_ENABLED)
    SPI_HandleTypeDef* hspi;
    #else
    void* hspi;
    #endif /* defined(HAL_SPI_MODULE_ENABLED) */
} cyhal_spi_t;


/* RNG object */
typedef struct
{
    #if defined(HAL_RNG_MODULE_ENABLED)
    RNG_HandleTypeDef* hrng;
    #else
    void* hrng;
    #endif /* defined(HAL_RNG_MODULE_ENABLED) */
} cyhal_trng_t;


/* UART context structure
 * This structure need for backward compatibility with PSoC6 implementation */

typedef struct
{
    uint32_t rxBufIdx;    /**< The current location in the receive buffer */
} cy_uart_context_t;

/* UART buffers for DMA */
typedef struct __ALIGNED(_CYHAL_DMA_BUFFER_ALIGN_BYTES)
{
    uint8_t rx[CYHAL_UART_RX_DMA_BUFFER_SIZE];
    uint8_t tx[CYHAL_UART_TX_DMA_BUFFER_SIZE];
} cy_uart_dma_buf_t;


/* UART object */
typedef struct
{
    #if defined(HAL_UART_MODULE_ENABLED)
    cy_uart_dma_buf_t   dma_buff;
    UART_HandleTypeDef* huart;
    void*               callback;
    void*               callback_arg;
    uint32_t            irq;
    void*               rx_async_buff;
    uint32_t            rx_async_buff_len;
    cy_uart_context_t   context;
    #else
    void* huart;
    #endif /* defined(HAL_UART_MODULE_ENABLED) */
} cyhal_uart_t;

/* GPIO type */
typedef cyhal_gpio_def_t cyhal_gpio_t;

/* SDIO type */
typedef struct
{
    #if defined(HAL_SD_MODULE_ENABLED)
    SD_HandleTypeDef* hsd;
    uint32_t          block_size;
    uint32_t          frequencyhal_hz;
    void*             callback;
    void*             callback_arg;
    uint32_t          irq;
    #else
    void* hsd;
    #endif /* defined(HAL_SD_MODULE_ENABLED) */
} cyhal_sdio_t;


/* LPTIMER type */
typedef struct
{
    #if defined(HAL_LPTIM_MODULE_ENABLED)
    LPTIM_HandleTypeDef* hlptimer;
    void*                callback;
    void*                callback_arg;
    volatile uint32_t    cur_time_in_lpticks;
    volatile uint32_t    match;
    bool                 event_enable;
    #else
    void* hlptimer;
    #endif /* defined(HAL_LPTIM_MODULE_ENABLED) */
} cyhal_lptimer_t;


/* This is the best place to put this so that it is seen by
   Pack/Middlewares/Third_Party/whd-bsp-integration/cybsp_wifi.c */
cyhal_sdio_t* cybsp_get_wifi_sdio_obj(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

/** \} group_hal_hw_types */

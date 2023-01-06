/***************************************************************************************************
 * \file stm32_cyhal_sdio.c
 *
 * \brief
 * Provides a high level interface for interacting with STM32 SDIO.
 * This is a wrapper around the lower level STM32 SDIO HAL API.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2021 Cypress Semiconductor Corporation
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 **************************************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif


#pragma GCC push_options
#pragma GCC optimize ("O0")


#include "cyhal_sdio.h"
#include "stm32_cyhal_sdio_ex.h"
#include "stdio.h"
#include "string.h"


#include "stm32f4xx_ll_sdmmc.h"
#include "stm32f4xx_ll_rcc.h"
//ANN added below header
#include "stm32f4xx_hal_sd.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f429xx.h"

#include <zephyr/kernel.h>


#define MY_SDIO_IRQ  SDIO_IRQn       /* device uses IRQ 49 */
#define MY_SDIO_PRIO  2       /* device uses interrupt priority 2 *//* argument passed to my_isr(), in this case a pointer to the device */
#define MY_IRQ_FLAGS 0       /* IRQ flags */

#define MY_DMA2_IRQ  DMA2_Stream3_IRQn       /* device uses IRQ 49 */
#define MY_DMA2_PRIO  1       /* device uses interrupt priority 2 *//* argument passed to my_isr(), in this case a pointer to the device */

#define WICED 1

#if defined (WICED)
#include "whd_types.h"
#include "cyabs_rtos.h"
#define BUS_LEVEL_MAX_RETRIES                (5)
//#include "stm32f4xx_rcc.h"
#define SDIO_IRQ_CHANNEL                     ((uint8_t)0x31)
#define DMA2_3_IRQ_CHANNEL                   ((uint8_t)DMA2_Stream3_IRQn)
#define SDIO_ERROR_MASK                      ( SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_DTIMEOUT | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR | SDIO_STA_STBITERR )

/** @defgroup SDIO_Data_Block_Size
  * @{
  */

#define SDIO_DataBlockSize_1b               ((uint32_t)0x00000000)
#define SDIO_DataBlockSize_2b               ((uint32_t)0x00000010)
#define SDIO_DataBlockSize_4b               ((uint32_t)0x00000020)
#define SDIO_DataBlockSize_8b               ((uint32_t)0x00000030)
#define SDIO_DataBlockSize_16b              ((uint32_t)0x00000040)
#define SDIO_DataBlockSize_32b              ((uint32_t)0x00000050)
#define SDIO_DataBlockSize_64b              ((uint32_t)0x00000060)
#define SDIO_DataBlockSize_128b             ((uint32_t)0x00000070)
#define SDIO_DataBlockSize_256b             ((uint32_t)0x00000080)
#define SDIO_DataBlockSize_512b             ((uint32_t)0x00000090)
#define SDIO_DataBlockSize_1024b            ((uint32_t)0x000000A0)
#define SDIO_DataBlockSize_2048b            ((uint32_t)0x000000B0)
#define SDIO_DataBlockSize_4096b            ((uint32_t)0x000000C0)
#define SDIO_DataBlockSize_8192b            ((uint32_t)0x000000D0)
#define SDIO_DataBlockSize_16384b           ((uint32_t)0x000000E0)
#define IS_SDIO_BLOCK_SIZE(SIZE) (((SIZE) == SDIO_DataBlockSize_1b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16b) || \
                                  ((SIZE) == SDIO_DataBlockSize_32b) || \
                                  ((SIZE) == SDIO_DataBlockSize_64b) || \
                                  ((SIZE) == SDIO_DataBlockSize_128b) || \
                                  ((SIZE) == SDIO_DataBlockSize_256b) || \
                                  ((SIZE) == SDIO_DataBlockSize_512b) || \
                                  ((SIZE) == SDIO_DataBlockSize_1024b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2048b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4096b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8192b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16384b))

typedef enum
{
    SDIO_1B_BLOCK    =  1,
    SDIO_2B_BLOCK    =  2,
    SDIO_4B_BLOCK    =  4,
    SDIO_8B_BLOCK    =  8,
    SDIO_16B_BLOCK   =  16,
	SDIO_32B_BLOCK  =   32,
    SDIO_64B_BLOCK   =  64,
    SDIO_128B_BLOCK  =  128,
    SDIO_256B_BLOCK  =  256,
    SDIO_512B_BLOCK  =  512,
    SDIO_1024B_BLOCK = 1024,
    SDIO_2048B_BLOCK = 2048
} sdio_block_size_t;

//ANN: imp needs more buffer
//ALIGNED_PRE(4) static uint8_t       temp_dma_buffer[MAX(2*1024,WICED_LINK_MTU+ 64)] ALIGNED(4);
//CYHAL_ALIGN_DMA_BUFFER(static uint32_t  _temp_dma_buffer[_CYHAL_SDIO_DMA_BUFFER_SIZE]);
static uint8_t*                     user_data;
static uint32_t                     user_data_size;
static sdio_block_size_t find_optimal_block_size    ( uint32_t data_size );
static void              sdio_prepare_data_transfer ( cyhal_transfer_t direction, sdio_block_size_t block_size, uint8_t* data, uint16_t data_size );
static uint32_t                     current_command;

static whd_bus_transfer_direction_t current_transfer_direction;
//static cy_semaphore_t  sdio_transfer_finished_semaphore;
struct k_sem sdio_transfer_finished_semaphore;
static uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size);

K_SEM_DEFINE(sdio_transfer_finished_semaphore, 0, 1);
//K_SEM_DEFINE(threadB_sem, 0, 1);	/* starts off "not available" */


#endif

#define HAL_SD_MODULE_ENABLED 1 //ANN added temp

#if defined(HAL_SD_MODULE_ENABLED)



/***************************************************************************************************
 *      Private macros
 **************************************************************************************************/

#define __SDMMC_GET_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT) __SDIO_GET_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT)
#define __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_FLAG_CCRCFAIL) __SDIO_CLEAR_FLAG(SDMMCx, SDIO_FLAG_CCRCFAIL)
//#define __SDMMC_CMDTRANS_ENABLE(obj->hsd->Instance) __SDIO_CMDTRANS_ENABLE(obj->hsd->Instance)


/* Number of cycles for read/write operation complete */
#define _CYHAL_SDIO_RW_RETRY_CYCLES             (1000U)

#define _CYHAL_SDIO_400KHZ                      (400000U)

/* Masks for errors in an R5 response */
#define SDMMC_R5_COM_CRC_ERROR                  ((uint32_t)0x00008000U)
#define SDMMC_R5_ILLEGAL_COMMAND                ((uint32_t)0x00004000U)
#define SDMMC_R5_IO_CURRENT_STATE               ((uint32_t)0x00003000U)
#define SDMMC_R5_ERROR                          ((uint32_t)0x00000400U)
#define SDMMC_R5_FUNCTION_NUMBER                ((uint32_t)0x00000200U)
#define SDMMC_R5_OUT_OF_RANGE                   ((uint32_t)0x00000100U)
#define SDMMC_R5_ERRORBITS                      (SDMMC_R5_COM_CRC_ERROR   | \
                                                 SDMMC_R5_ILLEGAL_COMMAND | \
                                                 SDMMC_R5_ERROR           | \
                                                 SDMMC_R5_FUNCTION_NUMBER | \
                                                 SDMMC_R5_OUT_OF_RANGE)
/* Data Block Size */
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_2B          (2U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_4B          (4U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_8B          (8U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_16B         (16U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_32B         (32U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_64B         (64U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_128B        (128U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_256B        (256U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_512B        (512U)

/* Set default SDIO priority */
#if !defined (CYHAL_SDIO_IRQ_PRIORITY)
    #define CYHAL_SDIO_IRQ_PRIORITY             (5U)
#endif /* !defined (CYHAL_SDIO_IRQ_PRIORITY) */

/* SDIO DMA buffer used in cyhal_sdio_bulk_transfer function. The default value
 * is 1568 bytes, it required by WHD as max backplane transfer size.
 * Overwrite _CYHAL_SDIO_DMA_BUFFER_SIZE in cybsp.h if need to increase the
 * SDIO DMA buffer size */
#if !defined (_CYHAL_SDIO_DMA_BUFFER_SIZE)
    #define _CYHAL_SDIO_DMA_BUFFER_SIZE         (1568 / 4)  /* size in words */
#endif /* !defined (_CYHAL_SDIO_DMA_BUFFER_SIZE) */

#if defined(WICED)
#define SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS    900000//(100000)
//CYHAL_ALIGN_DMA_BUFFER(static uint32_t  temp_dma_buffer[_CYHAL_SDIO_DMA_BUFFER_SIZE]);
static uint32_t  temp_dma_buffer[_CYHAL_SDIO_DMA_BUFFER_SIZE];
#endif

/***************************************************************************************************
 *      Private variables
 **************************************************************************************************/

static SD_HandleTypeDef* _cyhal_sdio_handle = NULL;
static SD_HandleTypeDef hsd;
static SDIO_InitTypeDef sdio_init_structure;
#if defined(WICED) //ANN taken from WICED
#define SDIO_TransferDir_ToCard             ((uint32_t)0x00000000)
#define SDIO_TransferDir_ToSDIO             ((uint32_t)0x00000002)

#define SDIO_TransferMode_Block             ((uint32_t)0x00000000)
#define SDIO_TransferMode_Stream            ((uint32_t)0x00000004)

#define SDIO_DPSM_Disable                    ((uint32_t)0x00000000)
#define SDIO_DPSM_Enable                     ((uint32_t)0x00000001)

static uint8_t*                     dma_data_source;
static uint32_t                     dma_transfer_size;

/*typedef enum
{
     If updating this enum, the bus_direction_mapping variable will also need to be updated
    BUS_READ,
    BUS_WRITE
} wwd_bus_transfer_direction_t;*/

static const uint32_t bus_direction_mapping[] =
{
    [BUS_READ]  = SDIO_TransferDir_ToSDIO,
    [BUS_WRITE] = SDIO_TransferDir_ToCard
};

static bool sdio_transfer_failed;

#endif
/***************************************************************************************************
 *      Private functions
 **************************************************************************************************/
//ANN: replaced SDMMC_TypeDef by SDIO_TypeDef as the stm32f429xx.h has sdio instead sdmmc
static uint32_t _stm32_sdio_cmd_rw_extended(SDIO_TypeDef* SDMMCx, uint32_t argument,
                                            uint32_t* response);
static uint32_t _stm32_sdio_cmd_send_op_cond(SDIO_TypeDef* SDMMCx);

static uint32_t _stm32_sdio_cmd_rw_direct(SDIO_TypeDef* SDMMCx, uint32_t argument,
                                          uint32_t* response);
static uint32_t _stm32_sdio_convert_block_size(uint16_t block_size);
static uint32_t _stm32_sdio_find_optimal_block_size(uint32_t data_size);

static void _stm32_sdio_enable_irq(const SD_TypeDef* instance, uint32_t priority, bool en_irq);

//...dma ? static void _stm32_sdio_enable_irq(const SD_TypeDef* instance, uint32_t priority, bool en_irq);

static void _stm32_sdio_enable_hw_block(cyhal_sdio_t* obj);
static void _stm32_sdio_disable_hw_block(const cyhal_sdio_t* obj);

static uint32_t _stm32_sdio_get_cmd_resp4(SDIO_TypeDef* SDMMCx);
static uint32_t _stm32_sdio_get_cmd_resp5(SDIO_TypeDef* SDMMCx, uint8_t SD_CMD, uint32_t* data);

static uint32_t _stm32_safe_divide(uint32_t num, uint32_t denom);


/***************************************************************************************************
 * stm32_cypal_sdio_hw_init
 **************************************************************************************************/
uint32_t stm32_cypal_sdio_hw_init(SD_HandleTypeDef* hsd)
{
    /* Check the parameters */
    assert_param(NULL != hsd);

    _cyhal_sdio_handle = hsd;
    return 0;
}


void my_sdio_isr_installer(void)
{
	IRQ_DIRECT_CONNECT(MY_SDIO_IRQ, MY_SDIO_PRIO, stm32_cyhal_sdio_irq_handler, MY_IRQ_FLAGS);
	irq_enable(MY_SDIO_IRQ);
}


void my_dma2_isr_installer(void)
{
	IRQ_DIRECT_CONNECT(MY_DMA2_IRQ, MY_DMA2_PRIO, stm32_cyhal_dma2_irq_handler, MY_IRQ_FLAGS);
	irq_enable(MY_DMA2_IRQ);
}





/***************************************************************************************************
 * cyhal_sdio_init
 **************************************************************************************************/

cy_rslt_t cyhal_sdio_init(cyhal_sdio_t* obj, cyhal_gpio_t cmd, cyhal_gpio_t clk, cyhal_gpio_t data0,
                          cyhal_gpio_t data1, cyhal_gpio_t data2, cyhal_gpio_t data3)
{
    (void)cmd;
    (void)clk;
    (void)data0;
    (void)data1;
    (void)data2;
    (void)data3;
    uint32_t sdmmc_clk;


    /* ANN: Wiced adopted code*/
#if defined (WICED)
    /*  Turn on SDIO IRQ */
    SDIO->ICR = (uint32_t) 0xffffffff;
    /* Must be lower priority than the value of configMAX_SYSCALL_INTERRUPT_PRIORITY */
    /* otherwise FreeRTOS will not be able to mask the interrupt */
    /* keep in mind that ARMCM3 interrupt priority logic is inverted, the highest value */
    /* is the lowest priority */
    NVIC_EnableIRQ( ( IRQn_Type ) SDIO_IRQ_CHANNEL );
    NVIC_EnableIRQ( ( IRQn_Type ) DMA2_3_IRQ_CHANNEL );
#endif


    //ANN: WICED - SDIO_DeInit( );
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_SDIO_CLK_DISABLE();

    hsd.Instance = SDIO;

    // Update obj sd handle
      obj->hsd = &hsd; //_cyhal_sdio_handle;

      stm32_cypal_sdio_hw_init(&hsd);

    // Init the low level hardware : GPIO, CLOCK, CORTEX...etc
       if (obj->hsd->State == HAL_SD_STATE_RESET)
       {
           // Allocate lock resource and initialize it
           obj->hsd->Lock = HAL_UNLOCKED;

           HAL_SD_MspInit(obj->hsd);
       }
#if defined (WICED)
       /*!< Enable the SDIO AHB Clock and the DMA2 Clock */
    do {
	   __IO uint32_t tmpreg;
	   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

	   // Delay after an RCC peripheral clock enabling
	   tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
	   UNUSED(tmpreg);
	   } while(0);

#if defined (SDIO)
    {
    	if (obj->hsd->Instance == SDIO)
    	{
    	do {
    	   __IO uint32_t tmpreg;
    	   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);

    	   // Delay after an RCC peripheral clock enabling
    	   tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);
    	   UNUSED(tmpreg);
    	   } while(0);
    	}
    }
    #endif
#endif //WICED

//ANN added below
    sdio_init_structure.ClockDiv = (uint8_t)0;
    sdio_init_structure.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    //sdio_init_structure.ClockBypass = 0;
    sdio_init_structure.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    sdio_init_structure.BusWide = SDIO_BUS_WIDE_4B; //ANN: as per WICED 1B
    sdio_init_structure.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;


    (void)SDIO_Init(obj->hsd->Instance, sdio_init_structure);
    (void)SDIO_PowerState_ON(obj->hsd->Instance);
    (void)SDIO_SetSDMMCReadWaitMode(obj->hsd->Instance, SDIO_READ_WAIT_MODE_CLK);
/*
    hsd.Init.ClockDiv = (uint8_t)120; //ANN from WICED
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B; //ANN: as per WICED
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;


    // Check the parameters
    assert_param(NULL != obj);

    // Update obj sd handle
    obj->hsd = &hsd; //_cyhal_sdio_handle;

    // Check the SD handle allocation
    if (obj->hsd == NULL)
    {
        return (cy_rslt_t)HAL_ERROR;
    }

    // Enable SDIO block
    _stm32_sdio_enable_hw_block(obj);
*/

    // Init Clock should be less or equal to 400Khz
    //sdmmc_clk = (LL_RCC_GetSDIOClockFreq(LL_RCC_SDIO_CLKSOURCE))/120;

    //__HAL_RCC_SDIO_CLK_DISABLE();

    hsd.Instance->CLKCR |= SDIO_CLKCR_CLKEN;

       // Initialize the error code
    obj->hsd->ErrorCode = HAL_SD_ERROR_NONE;

    // store object in handle context
    obj->hsd->Context = (uint32_t)obj;

    // Initialize the SD state
    obj->hsd->State = HAL_SD_STATE_READY;

    // Enable interrupts
    _stm32_sdio_enable_irq(obj->hsd->Instance, CYHAL_SDIO_IRQ_PRIORITY, true);

    my_sdio_isr_installer();
    my_dma2_isr_installer();

    return CY_RSLT_SUCCESS;
}

//ANN added below function
//void SDMMC1_IRQHandler(void )
//void SDIO_IRQHandler(void)
/*void SDIO_irq(void)
{
    stm32_cyhal_sdio_irq_handler();
}*/



/***************************************************************************************************
 * cyhal_sdio_configure
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_configure(cyhal_sdio_t* obj, const cyhal_sdio_cfg_t* config)
{
    uint32_t clk_freq;

    /* Check the parameters */
    assert_param(NULL != obj);

    /* Update obj sd handle */
    if (obj->hsd == NULL)
    {
        obj->hsd = _cyhal_sdio_handle;
    }

    hsd.Instance = SDIO;

/*
#if defined (WICED)
       //*!< Enable the SDIO AHB Clock and the DMA2 Clock
       //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );
       //RCC_APB2PeriphClockCmd( RCC_APB2Periph_SDIO, ENABLE );

#if defined (SDIO)
    {
    	if (obj->hsd->Instance == SDIO)
    	{
    	do {
    	   __IO uint32_t tmpreg;
    	   //SET_BIT(RCC->AHB2ENR, 0x00000200);
    	   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);

    	   // Delay after an RCC peripheral clock enabling
    	   tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);
    	   UNUSED(tmpreg);
    	   } while(0);
    	}
    }
    #endif
#endif //WICED
*/


    /* Do not change frequency if requested value is zero */
    if (config->frequencyhal_hz != 0u)
    {
        /* Override the SDMMC Clock frequency Configuration if defined */
        #ifdef SDMMC_CLK_FREQ_OVERRIDE
        clk_freq = SDMMC_CLK_FREQ_OVERRIDE;
        #else
        clk_freq = config->frequencyhal_hz;
        #endif
        obj->frequencyhal_hz = clk_freq;

        /* Calculate SDMMC Clock divider. */
        /* ClockDiv = SDMMC input clock / (2 * Expected SDMMC Clock) */
        //uint32_t sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(STM32_RCC_PERIPHCLK_SDMMC);

        uint32_t sdmmc_clk = LL_RCC_GetSDIOClockFreq(LL_RCC_SDIO_CLKSOURCE);
        obj->hsd->Init.ClockDiv = _stm32_safe_divide(sdmmc_clk, 2U * clk_freq);

        //ANN commented as CKCR.CLKEN bit is getting reset & causing CMD52 to CMDACT flag set always
        /* Reset and Enable SDIO block */
        //_stm32_sdio_enable_hw_block(obj);
    }

    /* Do not change block size if requested value is zero */
    if (config->block_size != 0u)
    {
        obj->block_size = config->block_size;
    }

    return CY_RSLT_SUCCESS;
}


/***************************************************************************************************
 * cyhal_sdio_send_cmd
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_send_cmd(const cyhal_sdio_t* obj, cyhal_transfer_t direction,
                              cyhal_sdio_command_t command, uint32_t argument, uint32_t* response)
{
    (void)direction;
    uint32_t ret;
    /* Check the parameters */
    assert_param(obj != NULL);
    assert_param(obj->hsd != NULL);

    k_msleep(1);

    switch (command)
    {
        /* CMD0 */
        case CYHAL_SDIO_CMD_GO_IDLE_STATE:
        	ret = SDMMC_CmdGoIdleState(obj->hsd->Instance);
            break;

        /* CMD3 */
        case CYHAL_SDIO_CMD_SEND_RELATIVE_ADDR:
        {
            uint16_t resp;
            assert_param(response != NULL);
            ret = SDMMC_CmdSetRelAdd(obj->hsd->Instance, &resp);
            *response = resp;
            //*response = resp<<16; //In WICED, we are not bothered about different error flags as in function ll "SDMMC_GetCmdResp6"
            // we are getting the same R6 reponse of 0x10000 there, but its been >>16 times based on some flag setting
            break;
        }

        /* CMD5 */
        case CYHAL_SDIO_CMD_IO_SEND_OP_COND:
        	ret = _stm32_sdio_cmd_send_op_cond(obj->hsd->Instance);
        	  // *response = SDIO->RESP1; //ANN added this from WICED
        	break;

        /* CMD7 */
        case CYHAL_SDIO_CMD_SELECT_CARD:
        	ret = SDMMC_CmdSelDesel(obj->hsd->Instance, (uint64_t)argument << 16);
        	//ret = SDMMC_CmdSelDesel(obj->hsd->Instance, (uint64_t)argument);
            break;

        /* CMD52 */
        case CYHAL_SDIO_CMD_IO_RW_DIRECT:
            /* this one already has  != NULL check inside */
            ret = _stm32_sdio_cmd_rw_direct(obj->hsd->Instance, argument, response);
            break;

        /* CMD53 */
        case CYHAL_SDIO_CMD_IO_RW_EXTENDED:
            /* this one already has  != NULL check inside */
            ret = _stm32_sdio_cmd_rw_extended(obj->hsd->Instance, argument, response);
            break;

        case CYHAL_SDIO_CMD_GO_INACTIVE_STATE:
        default:
            ret = CYHAL_SDIO_RSLT_ERR_BAD_PARAM;
            break;
    }
    return ret;
}


/***************************************************************************************************
 * cyhal_sdio_bulk_transfer
 **************************************************************************************************/
//#if(0)
cy_rslt_t cyhal_sdio_bulk_transfer(cyhal_sdio_t* obj, cyhal_transfer_t direction, uint32_t argument,
                                   const uint32_t* data, uint16_t length, uint32_t* response)
{
    uint32_t loop_count = 0;
    whd_result_t result;
    uint16_t attempts = 0;

    sdio_block_size_t block_size;
    uint16_t data_size = length;

   //whd_assert("Bad args", !((command == CYHAL_SDIO_CMD_IO_RW_EXTENDED) && (data == NULL)));

    if ( response != NULL )
    {
        *response = 0;
    }

    //platform_mcu_powersave_disable();

    /* Ensure the bus isn't stuck half way through transfer */
    DMA2_Stream3->CR   = 0;

restart:
    SDIO->ICR = (uint32_t) 0xFFFFFFFF;
    sdio_transfer_failed = 0;
    ++attempts;

    /* Check if we've tried too many times */
    if (attempts >= (uint16_t) BUS_LEVEL_MAX_RETRIES)
    {
    	//ANN temp
        result = 1051; //WWD_SDIO_RETRIES_EXCEEDED; this prpares the exception to return, so skipped now
        goto exit;
    }
    /* Prepare the data transfer register */

//    if ( command == SDIO_CMD_53 )
    {
        //sdio_enable_bus_irq();
    	SDIO->MASK = SDIO_MASK_SDIOITIE | SDIO_MASK_CMDRENDIE | SDIO_MASK_CMDSENTIE;

        /* Dodgy STM32 hack to set the CMD53 byte mode size to be the same as the block size */

        //if ( mode == SDIO_BYTE_MODE )
        {
            block_size = find_optimal_block_size( data_size );
            if ( block_size < SDIO_512B_BLOCK )
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) ) | block_size;
            }
            else
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) );
            }
        }

        /* Prepare the SDIO for a data transfer */
        current_transfer_direction = direction;
        sdio_prepare_data_transfer( direction, block_size, (uint8_t*) data, data_size );

        /* Send the command */
        SDIO->ARG = argument;
        SDIO->CMD = (uint32_t) ( CYHAL_SDIO_CMD_IO_RW_EXTENDED | SDIO_RESPONSE_SHORT | SDIO_WAIT_NO | SDIO_CPSM_ENABLE );

        /* Wait for the whole transfer to complete */
        //k_sem_init(&sdio_transfer_finished_semaphore, 0, 1);
        //result = cy_rtos_semaphore_get( &sdio_transfer_finished_semaphore, (uint32_t) 50 );
        //result = k_sem_take(&sdio_transfer_finished_semaphore, K_MSEC(50));

//        for (int i=0;i<10000;i++);for (int i=0;i<10000;i++);for (int i=0;i<10000;i++);

//        if ( result != WHD_SUCCESS )
//        {
//            goto exit;
//        }

        if ( sdio_transfer_failed == true )
        {
            goto restart;
        }

        /* Check if there were any SDIO errors */
        if ( ( SDIO->STA & ( SDIO_STA_DTIMEOUT | SDIO_STA_CTIMEOUT ) ) != 0 )
        {
            goto restart;
        }
        else if ( ( ( SDIO->STA & ( SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR ) ) != 0 ) )
        {
           // wiced_assert( "SDIO communication failure", 0 );
            goto restart;
        }

   //     for (int i=0;i<10000;i++);for (int i=0;i<10000;i++);for (int i=0;i<10000;i++);for (int i=0;i<10000;i++);

        /* Wait till complete */
        loop_count = (uint32_t) SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS;
        do
        {
            loop_count--;
            if ( loop_count == 0 || ( ( SDIO->STA & SDIO_ERROR_MASK ) != 0 ) )
            {
                goto restart;
            }
        } while ( ( SDIO->STA & ( SDIO_STA_TXACT | SDIO_STA_RXACT ) ) != 0 );

        if ( direction == BUS_READ )
        {
            memcpy( data, dma_data_source, (size_t) user_data_size );
        }
    }

	    if ( response != NULL )
    {
        *response = SDIO->RESP1;
    }
    result = WHD_SUCCESS;

exit:
    //platform_mcu_powersave_enable();
    SDIO->MASK = SDIO_MASK_SDIOITIE;
    return result;

}
//#endif
/***************************************************************************************************
 * cyhal_sdio_register_callback
 **************************************************************************************************/
void cyhal_sdio_register_callback(cyhal_sdio_t* obj, cyhal_sdio_event_callback_t callback,
                                  void* callback_arg)
{
    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != callback);

    /* Store callback info */
    obj->callback     = (void*)callback;
    obj->callback_arg = callback_arg;
}


/***************************************************************************************************
 * cyhal_sdio_enable_event
 **************************************************************************************************/
void cyhal_sdio_enable_event(cyhal_sdio_t* obj, cyhal_sdio_event_t event, uint8_t intr_priority,
                             bool enable)
{
    (void)intr_priority;
    if (enable)
    {
        __HAL_SD_ENABLE_IT(obj->hsd, SDIO_IT_SDIOIT);
        obj->irq |= (uint32_t)event;
    }
    else
    {
        __HAL_SD_DISABLE_IT(obj->hsd, SDIO_IT_SDIOIT);
        obj->irq &= ~(uint32_t)event;
    }
}

ISR_DIRECT_DECLARE(stm32_cyhal_dma2_irq_handler)
{
    whd_result_t result;

    /* Clear interrupt */
    DMA2->LIFCR = (uint32_t) (0x3F << 22);

    //result = host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WHD_TRUE );

    /* check result if in debug mode */
    //WPRINT_WHD_DEBUG( "failed to set dma semaphore", result == WHD_SUCCESS );

    /*@-noeffect@*/
    (void) result; /* ignore result if in release mode */
    /*@+noeffect@*/
}

/***************************************************************************************************
 * stm32_cyhal_sdio_irq_handler
 **************************************************************************************************/
//void stm32_cyhal_sdio_irq_handler(void)
ISR_DIRECT_DECLARE(stm32_cyhal_sdio_irq_handler)
{
#if defined (WICED)
	uint32_t intstatus = SDIO->STA;

	    //WHD_BUS_STATS_INCREMENT_VARIABLE( sdio_intrs );
	#if defined(STM32F412xG)
	    if (current_command == SDIO_CMD_5)
	        SDIO->ICR = SDIO_ICR_CCRCFAILC;
	#endif
	    if ( ( intstatus & ( SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR  | SDIO_STA_STBITERR )) != 0 )
	    {
	       // WHD_BUS_STATS_INCREMENT_VARIABLE( error_intrs );
	        //wiced_assert("sdio error flagged",0);
	        sdio_transfer_failed = WHD_TRUE;
	        SDIO->ICR = (uint32_t) 0xffffffff;

	       // host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WHD_TRUE );
	    }
	    else
	    {
	        if ((intstatus & (SDIO_STA_CMDREND | SDIO_STA_CMDSENT)) != 0)
	        {
	            /*if ( ( SDIO->RESP1 & 0x800 ) != 0 )
	            {
	                sdio_transfer_failed = WICED_TRUE;
	                host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
	            }
	            else if (current_command == SDIO_CMD_53)*/
	            {
	                if (current_transfer_direction == BUS_WRITE)
	                {
	                    DMA2_Stream3->CR = DMA_MEMORY_TO_PERIPH |
	                    		DMA_CHANNEL_4 | DMA_PINC_DISABLE | DMA_MINC_ENABLE |
								DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD |
								DMA_NORMAL | DMA_PRIORITY_VERY_HIGH |
								DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_SxCR_PFCTRL | DMA_SxCR_EN | DMA_SxCR_TCIE;
	                }
	                else
	                {
	                    DMA2_Stream3->CR = DMA_PERIPH_TO_MEMORY |
	                    		DMA_CHANNEL_4 | DMA_PINC_DISABLE | DMA_MINC_ENABLE |
								DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD |
								DMA_NORMAL | DMA_PRIORITY_VERY_HIGH |
								DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_SxCR_PFCTRL | DMA_SxCR_EN | DMA_SxCR_TCIE;
	                }
	            }

	            /* Clear all command/response interrupts */
	            SDIO->ICR = (SDIO_STA_CMDREND | SDIO_STA_CMDSENT);
	        }

	        /* Check whether the external interrupt was triggered */
	    //ANN temp    if ( ( intstatus & SDIO_STA_SDIOIT ) != 0 )
	        {
	            /* Clear the interrupt and then inform WICED thread */
	            SDIO->ICR = SDIO_ICR_SDIOITC;
	            //ANN temp whd_thread_notify_irq( );

	            /* Clear the interrupt */
	                    __HAL_SD_CLEAR_FLAG(_cyhal_sdio_handle, SDIO_FLAG_SDIOIT);

	                    /* Mask interrupt, to be unmasked later by Tx Path */
	                //    __HAL_SD_DISABLE_IT(_cyhal_sdio_handle, SDIO_IT_SDIOIT);

	                    /* Set IRQ Flag which will be used for setting IRQ MASK back */
	                   // obj->irq |= (uint32_t)CYHAL_SDIO_CARD_INTERRUPT;

	        }
	    }
#endif //WICED

#if(0)
	if ((_cyhal_sdio_handle != NULL) &&
        (__HAL_SD_GET_FLAG(_cyhal_sdio_handle, SDIO_STA_SDIOIT) != RESET))
    {
        cyhal_sdio_t* obj = (cyhal_sdio_t*)_cyhal_sdio_handle->Context;
        ((cyhal_sdio_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_SDIO_CARD_INTERRUPT);

        /* Clear the interrupt */
        __HAL_SD_CLEAR_FLAG(_cyhal_sdio_handle, SDIO_FLAG_SDIOIT);

        /* Mask interrupt, to be unmasked later by Tx Path */
        __HAL_SD_DISABLE_IT(_cyhal_sdio_handle, SDIO_IT_SDIOIT);

        /* Set IRQ Flag which will be used for setting IRQ MASK back */
        obj->irq |= (uint32_t)CYHAL_SDIO_CARD_INTERRUPT;
    }
#endif
}


/***************************************************************************************************
 * Private Functions
 **************************************************************************************************/

/***************************************************************************************************
 * _stm32_safe_divide
 **************************************************************************************************/
static uint32_t _stm32_safe_divide(uint32_t num, uint32_t denom)
{
    /* Safe divide */
    uint32_t divres;

    assert_param(num != 0u);
    assert_param(denom != 0u);
    divres = num / denom;
    if ((num % denom) >= (denom>>1))
    {
        divres++;
    }
    return divres;
}


/***************************************************************************************************
 * _stm32_sdio_enable_hw_block
 **************************************************************************************************/
static void _stm32_sdio_enable_hw_block(cyhal_sdio_t* obj)
{
    /* Disable/reset SDIO Block */
    _stm32_sdio_disable_hw_block(obj);

    /* Enable the SDIO Clock */

    #if defined (SDMMC1)
    if (obj->hsd->Instance == SDMMC1)
    {
        __HAL_RCC_SDMMC1_CLK_ENABLE();
    }
    #endif
    #if defined (SDMMC2)
    else
    {
        __HAL_RCC_SDMMC2_CLK_ENABLE();
    }
    #endif
#if(0) //ifndef(WICED)
    //ANN added below
    #if defined (SDIO)
    {
    	if (obj->hsd->Instance == SDIO)
    	{
     		 //Enable clocks for SDIO and DMA2
   			//rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SDIOEN);
     		 //__HAL_RCC_PWR_CLK_ENABLE();

    		 //Enable the SDIO APB2 Clock
    		  //RCC_APB2PeriphClockCmd(RCC_APB2ENR_SDIOEN, ENABLE);

    		//__HAL_RCC_SDIO_CLK_ENABLE();
    	do {
    	   __IO uint32_t tmpreg;
    	   //SET_BIT(RCC->AHB2ENR, 0x00000200);
    	   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);

    	   // Delay after an RCC peripheral clock enabling
    	   tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN);
    	   UNUSED(tmpreg);
    	   } while(0);
    	}
    }
    #endif
#endif //if(0)


    /* Initialize SDMMC peripheral interface with default configuration */
    //(void)SDMMC_Init(obj->hsd->Instance, obj->hsd->Init);

      //obj->hsd->Init.ClockDiv = (uint32_t)0x76;

      (void)SDIO_Init(obj->hsd->Instance, obj->hsd->Init);

    /* Set Power State to ON */
    //(void)SDMMC_PowerState_ON(obj->hsd->Instance);
    (void)SDIO_PowerState_ON(obj->hsd->Instance);
//#if(0)
    /* Wait clock */
    //(void)SDMMC_SetSDMMCReadWaitMode(obj->hsd->Instance, SDIO_READ_WAIT_MODE_CLK);
    (void)SDIO_SetSDMMCReadWaitMode(obj->hsd->Instance, SDIO_READ_WAIT_MODE_CLK);
    //(void)SDIO_SetSDIOReadWaitMode(obj->hsd->Instance, SDIO_READ_WAIT_MODE_CLK);


    //uint32_t sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(STM32_RCC_PERIPHCLK_SDMMC) /
    //                     (2U * obj->hsd->Init.ClockDiv);


//#endif
    uint32_t sdmmc_clk = LL_RCC_GetSDIOClockFreq(LL_RCC_SDIO_CLKSOURCE) /
                        (/*2U **/ obj->hsd->Init.ClockDiv);


    if (sdmmc_clk != 0U)
    {
        /* Wait 74 Cycles: required power up waiting time
         * before starting the SD initialization sequence */
        HAL_Delay(1U + (74U * 1000U / (sdmmc_clk)));
    }
    else
    {
        HAL_Delay(2U);
    }

}

/***************************************************************************************************
 * _stm32_sdio_disable_hw_block
 **************************************************************************************************/
static void _stm32_sdio_disable_hw_block(const cyhal_sdio_t* obj)
{
    /* Reset SDIO Block */
    //(void)SDMMC_PowerState_OFF(obj->hsd->Instance);
	(void)SDIO_PowerState_OFF(obj->hsd->Instance);
    #if defined (SDMMC1)
    if (obj->hsd->Instance == SDMMC1)
    {
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
    }
    #endif
    #if defined (SDMMC2)
    else
    {
        __HAL_RCC_SDMMC2_FORCE_RESET();
        __HAL_RCC_SDMMC2_RELEASE_RESET();
    }
    #endif
//ANN added below
    #if defined (SDIO)
    {
    	 __HAL_RCC_SDIO_FORCE_RESET();
    	 __HAL_RCC_SDIO_RELEASE_RESET();
    }
	#endif
}


/***************************************************************************************************
 * _stm32_sdio_find_optimal_block_size
 **************************************************************************************************/
static  uint32_t _stm32_sdio_find_optimal_block_size(uint32_t data_size)
{
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_256B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_512B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_128B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_256B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_64B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_128B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_32B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_64B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_16B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_32B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_8B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_16B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_4B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_8B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_2B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_4B;
    }
    return _CYHAL_SDIO_DATA_BLOCK_SIZE_4B;
}


/***************************************************************************************************
 * _stm32_sdio_convert_block_size
 **************************************************************************************************/
static uint32_t _stm32_sdio_convert_block_size(uint16_t block_size)
{
    switch (block_size)
    {
        case 1:
            return SDIO_DATABLOCK_SIZE_1B;

        case 2:
            return SDIO_DATABLOCK_SIZE_2B;

        case 4:
            return SDIO_DATABLOCK_SIZE_4B;

        case 8:
            return SDIO_DATABLOCK_SIZE_8B;

        case 16:
            return SDIO_DATABLOCK_SIZE_16B;

        case 32:
            return SDIO_DATABLOCK_SIZE_32B;

        case 64:
            return SDIO_DATABLOCK_SIZE_64B;

        case 128:
            return SDIO_DATABLOCK_SIZE_128B;

        case 256:
            return SDIO_DATABLOCK_SIZE_256B;

        case 512:
            return SDIO_DATABLOCK_SIZE_512B;

        case 1024:
            return SDIO_DATABLOCK_SIZE_1024B;

        case 2048:
            return SDIO_DATABLOCK_SIZE_2048B;

        case 4096:
            return SDIO_DATABLOCK_SIZE_4096B;

        case 8192:
            return SDIO_DATABLOCK_SIZE_8192B;

        case 16384:
            return SDIO_DATABLOCK_SIZE_16384B;

        default:
            assert_param(false); /* wrong value */
            return SDIO_DATABLOCK_SIZE_512B;
    }
}


/***************************************************************************************************
 * _stm32_sdio_enable_irq
 **************************************************************************************************/
static void _stm32_sdio_enable_irq(const SD_TypeDef* instance, uint32_t priority, bool en_irq)
{
    IRQn_Type IRQn = (IRQn_Type)0;

    if (instance == SDIO)
       {
           IRQn = SDIO_IRQn;
       }

    #if defined (SDMMC1)
    if (instance == SDMMC1)
    {
        IRQn = SDMMC1_IRQn;
    }
    else
    #endif

    #if defined (SDMMC2)
    if (instance == SDMMC2)
    {
        IRQn = SDMMC2_IRQn;
    }


    #endif

    else
    {
        assert_param(false); // wrong instance
    }

    if (en_irq)
    {
        HAL_NVIC_SetPriority(IRQn, priority, 0);
        HAL_NVIC_EnableIRQ(IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(IRQn);
    }
}


/***************************************************************************************************
 * _stm32_sdio_get_cmd_resp4
 **************************************************************************************************/
static uint32_t _stm32_sdio_get_cmd_resp4(SDIO_TypeDef* SDMMCx)
{
    uint32_t sta_reg;
    /* 8 is the number of required instructions cycles for the below loop statement.
       The SDMMC_CMDTIMEOUT is expressed in ms */
    uint32_t count = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do
    {
        if (count-- == 0U)
        {
            return SDMMC_ERROR_TIMEOUT;
        }
        sta_reg = SDMMCx->STA;
    } while(((sta_reg & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) == 0U) ||
            ((sta_reg & SDIO_FLAG_CMDACT) != 0U));

    if (__SDMMC_GET_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT);

        return SDMMC_ERROR_CMD_RSP_TIMEOUT;
    }
    else
    {
        /* Clear all the static flags */
        __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_STATIC_CMD_FLAGS);
    }

    return SDMMC_ERROR_NONE;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_send_op_cond
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_send_op_cond(SDIO_TypeDef* SDMMCx)
{
    SDIO_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_SEN_OP_COND;
    sdmmc_cmdinit.Response         = SDIO_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDIO_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDIO_CPSM_ENABLE;

    (void)SDIO_SendCommand(SDMMCx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp4(SDMMCx);

    return errorstate;
    //return 0;
}


/***************************************************************************************************
 * _stm32_sdio_get_cmd_resp5
 **************************************************************************************************/
static uint32_t _stm32_sdio_get_cmd_resp5(SDIO_TypeDef* SDMMCx, uint8_t SD_CMD, uint32_t* data)
{
    uint32_t response_r1;
    uint32_t sta_reg;

    /* 8 is the number of required instructions cycles for the below loop statement.
       The SDMMC_CMDTIMEOUT is expressed in ms */
    uint32_t count = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do
    {
        if (count-- == 0U)
        {
            return SDMMC_ERROR_TIMEOUT;
        }
        sta_reg = SDMMCx->STA;
    } while(((sta_reg & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) == 0U) ||
            ((sta_reg & SDIO_FLAG_CMDACT) != 0U));

    if (__SDMMC_GET_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_FLAG_CTIMEOUT);
    	return SDMMC_ERROR_CMD_RSP_TIMEOUT;
    }
    else if (__SDMMC_GET_FLAG(SDMMCx, SDIO_FLAG_CCRCFAIL))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_FLAG_CCRCFAIL);

        return SDMMC_ERROR_CMD_CRC_FAIL;
    }
    else
    {
        // Nothing to do
    }

    /* Check response received is of desired command */

    if (SDIO_GetCommandResponse(SDMMCx) != SD_CMD)
    {
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    __SDMMC_CLEAR_FLAG(SDMMCx, SDIO_STATIC_CMD_FLAGS);

    /* We have received response, retrieve it.  */

    response_r1 = SDIO_GetResponse(SDMMCx, SDIO_RESP1);

    if ((response_r1 & SDMMC_R5_ERRORBITS) == SDMMC_ALLZERO)
    {
        if (data != NULL)
        {
            *data = response_r1;
        }
        return SDMMC_ERROR_NONE;
    }
    else if ((response_r1 & SDMMC_R5_COM_CRC_ERROR) == SDMMC_R5_COM_CRC_ERROR)
    {
        return SDMMC_ERROR_COM_CRC_FAILED;
    }
    else if ((response_r1 & SDMMC_R5_ILLEGAL_COMMAND) == SDMMC_R5_ILLEGAL_COMMAND)
    {
        return SDMMC_ERROR_ILLEGAL_CMD;
    }
    else if ((response_r1 & SDMMC_R5_ERROR) == SDMMC_R5_ERROR)
    {
        return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    }
    else if ((response_r1 & SDMMC_R5_FUNCTION_NUMBER) == SDMMC_R5_FUNCTION_NUMBER)
    {
        return SDMMC_ERROR_INVALID_PARAMETER;
    }
    else if ((response_r1 & SDMMC_R5_OUT_OF_RANGE) == SDMMC_R5_OUT_OF_RANGE)
    {
        return SDMMC_ERROR_ADDR_OUT_OF_RANGE;
    }
    else
    {
        /* Nothing to do */
    }

    /* Should not get here, but this is needed to make the compiler happy */
    return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_rw_direct
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_rw_direct(SDIO_TypeDef* SDMMCx, uint32_t argument,
                                          uint32_t* response)
{
    SDIO_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_RW_DIRECT;
    sdmmc_cmdinit.Response         = SDIO_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDIO_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDIO_CPSM_ENABLE;

    (void)SDIO_SendCommand(SDMMCx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp5(SDMMCx, SDMMC_CMD_SDMMC_RW_DIRECT, response);
    return errorstate;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_rw_extended
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_rw_extended(SDIO_TypeDef* SDMMCx, uint32_t argument,
                                            uint32_t* response)
{
    SDIO_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_RW_EXTENDED;
    sdmmc_cmdinit.Response         = SDIO_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDIO_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDIO_CPSM_ENABLE;

    (void)SDIO_SendCommand(SDMMCx, &sdmmc_cmdinit);
//for(int i=0;i<10000;i++);for(int i=0;i<10000;i++);for(int i=0;i<10000;i++);for(int i=0;i<10000;i++);
    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp5(SDMMCx, SDMMC_CMD_SDMMC_RW_EXTENDED, response);
    return errorstate;
}


/**
* @brief SD MSP Initialization
* This function configures the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hsd->Instance==SDIO)
  {
  /* USER CODE BEGIN SDMMC1_MspInit 0 */

  /* USER CODE END SDMMC1_MspInit 0 */
    /* Peripheral clock enable */
//    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    PD2     ------> SDMMC1_CMD
    PC8     ------> SDMMC1_D0
    PC9     ------> SDMMC1_D1
    */
    //ANN modified below as PC12 CLK may not have any pupd
    /*GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_8
                          |GPIO_PIN_9;*/

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_8
                              |GPIO_PIN_9;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;


    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN SDMMC1_MspInit 1 */

  /* USER CODE END SDMMC1_MspInit 1 */
  }

}

static void sdio_prepare_data_transfer( cyhal_transfer_t direction, sdio_block_size_t block_size, /*@unique@*/ uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/
{
    /* Setup a single transfer using the temp buffer */
    user_data         = data;
    user_data_size    = data_size;
    dma_transfer_size = (uint32_t) ( ( ( data_size + (uint16_t) block_size - 1 ) / (uint16_t) block_size ) * (uint16_t) block_size );

    if ( direction == BUS_WRITE )
    {
        dma_data_source = data;
    }
    else
    {
        dma_data_source = temp_dma_buffer;
    }

    SDIO->DTIMER = (uint32_t) 0xFFFFFFFF;
    SDIO->DLEN   = dma_transfer_size;
    SDIO->DCTRL  = (uint32_t)sdio_get_blocksize_dctrl(block_size) | bus_direction_mapping[(int)direction] | SDIO_TransferMode_Block | SDIO_DPSM_Enable | (1 << 3) | (1 << 11);

    /* DMA2 Stream3 */
    DMA2_Stream3->CR   = 0;
    DMA2->LIFCR        = (uint32_t) ( 0x3F << 22 );
    DMA2_Stream3->FCR  = (uint32_t) ( 0x00000021 | DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_FULL );
    DMA2_Stream3->PAR  = (uint32_t) &SDIO->FIFO;
    DMA2_Stream3->M0AR = (uint32_t) dma_data_source;
    DMA2_Stream3->NDTR = dma_transfer_size/4;
}

static sdio_block_size_t find_optimal_block_size( uint32_t data_size )
{
    if ( data_size > (uint32_t) 256 )
        return SDIO_512B_BLOCK;
    if ( data_size > (uint32_t) 128 )
        return SDIO_256B_BLOCK;
    if ( data_size > (uint32_t) 64 )
        return SDIO_128B_BLOCK;
    if ( data_size > (uint32_t) 32 )
        return SDIO_64B_BLOCK;
    if ( data_size > (uint32_t) 16 )
        return SDIO_32B_BLOCK;
    if ( data_size > (uint32_t) 8 )
        return SDIO_16B_BLOCK;
    if ( data_size > (uint32_t) 4 )
        return SDIO_8B_BLOCK;
    if ( data_size > (uint32_t) 2 )
        return SDIO_4B_BLOCK;

    return SDIO_4B_BLOCK;
}

static uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size)
{
    switch (block_size)
    {
        case SDIO_1B_BLOCK:    return SDIO_DataBlockSize_1b;
        case SDIO_2B_BLOCK:    return SDIO_DataBlockSize_2b;
        case SDIO_4B_BLOCK:    return SDIO_DataBlockSize_4b;
        case SDIO_8B_BLOCK:    return SDIO_DataBlockSize_8b;
        case SDIO_16B_BLOCK:   return SDIO_DataBlockSize_16b;
        case SDIO_32B_BLOCK:   return SDIO_DataBlockSize_32b;
        case SDIO_64B_BLOCK:   return SDIO_DataBlockSize_64b;
        case SDIO_128B_BLOCK:  return SDIO_DataBlockSize_128b;
        case SDIO_256B_BLOCK:  return SDIO_DataBlockSize_256b;
        case SDIO_512B_BLOCK:  return SDIO_DataBlockSize_512b;
        case SDIO_1024B_BLOCK: return SDIO_DataBlockSize_1024b;
        case SDIO_2048B_BLOCK: return SDIO_DataBlockSize_2048b;
        default: return 0;
    }
}

#if (0)
//ANN added below temp
/*
 * Set up the GPIO pins and peripheral clocks for the SDIO
 * system. The code should probably take an option card detect
 * pin, at the moment it uses the one used by the Embest board.
 */
void
sdio_init(void)
{
    /* Enable clocks for SDIO and DMA2 */
	//rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SDIOEN);

#ifdef WITH_DMA2
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
#endif

	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO15);

    /* Setup GPIO Pins for SDIO:
        PC8 - PC11 - DAT0 thru DAT3
              PC12 - CLK
               PD2 - CMD
    */
    // All SDIO lines are push-pull, 25Mhz
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            GPIO12 );

    // All SDIO lines are push-pull, 25Mhz
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            GPIO8 | GPIO9 | GPIO10 | GPIO11 );

    // D0 - D3 enable pullups (bi-directional)
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                    GPIO8 | GPIO9 | GPIO10 | GPIO11);
    // CLK line no pullup
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,  GPIO12);

	gpio_set_af(GPIOC, GPIO_AF12,
                GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12);
    gpio_set_af(GPIOD, GPIO_AF12, GPIO2);

    /* GPIOD setup */
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);

#ifdef SDIO_HAS_CARD_DETECT
    /* SDIO Card Detect pin on the Embest Baseboard */
    /*     PB15 as a hacked Card Detect (active LOW for card present) */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15);
#endif
}

#endif





//ANN commented
#endif /* defined(HAL_SDMMM_MODULE_ENABLED) */

#if(0)
cy_rslt_t cyhal_sdio_bulk_transfer(cyhal_sdio_t* obj, cyhal_transfer_t direction, uint32_t argument,
                                   const uint32_t* data, uint16_t length, uint32_t* response)
{
    cy_rslt_t result = CYHAL_SDIO_RSLT_CANCELED;
    CYHAL_ALIGN_DMA_BUFFER(static uint32_t  _temp_dma_buffer[_CYHAL_SDIO_DMA_BUFFER_SIZE]);

     /* Check data buffer if aligned on 32-bytes (needed for cache maintenance purpose),
     * and Length is multiple by block size. If NOT, use internal buffer for DMA */
    bool use_temp_dma_buffer = (((uint32_t)data % _CYHAL_DMA_BUFFER_ALIGN_BYTES) != 0u) ||
                               ((length % obj->block_size) != 0u);

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(0u != length);
    assert_param(!use_temp_dma_buffer || (length <= sizeof(_temp_dma_buffer)));

    SDIO_DataInitTypeDef config;
    sdio_cmd_argument_t   arg = { .value = argument };

    uint32_t  timeout = _CYHAL_SDIO_RW_RETRY_CYCLES;
    uint32_t  number_of_blocks;
    uint32_t  block_size;
    uint32_t  flags;
    uint32_t* p_dma_buffer = NULL;

    if (obj->hsd->State == HAL_SD_STATE_READY)
    {
        /* Block size not initialized */
        if (obj->block_size == 0U)
        {
            obj->block_size = _CYHAL_SDIO_DATA_BLOCK_SIZE_64B;
        }

        /* Block mode */
        if (length >= obj->block_size)
        {
            block_size       = obj->block_size;
            number_of_blocks = (length + block_size - 1u) / block_size;
        }
        /* Byte mode */
        else
        {
            block_size       = _stm32_sdio_find_optimal_block_size(length);
            number_of_blocks = 1UL;
        }

        /* Set the CMD53 byte mode size to be the same as the block size */
        if (arg.cmd53.block_mode == 0U)
        {
            if (_stm32_sdio_find_optimal_block_size(arg.cmd53.count) <
                _CYHAL_SDIO_DATA_BLOCK_SIZE_512B)
            {
                arg.cmd53.count = _stm32_sdio_find_optimal_block_size(arg.cmd53.count);
            }
            else
            {
                arg.cmd53.count = 0u;
            }
        }

        obj->hsd->ErrorCode = HAL_SD_ERROR_NONE;
        obj->hsd->State     = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        obj->hsd->Instance->DCTRL = 0u;

        if (use_temp_dma_buffer)
        {
            if ((length == 0) || (length > sizeof(_temp_dma_buffer)))
            {
                obj->hsd->State = HAL_SD_STATE_READY;
                return CYHAL_SDIO_BAD_ARGUMENT;
            }

            /* Using internal buffer */
            p_dma_buffer = _temp_dma_buffer;

            /* Clean internal buffer and copy data */
            (void)memcpy((void*)_temp_dma_buffer, (void*)data, length);

            if (length != sizeof(_temp_dma_buffer))
            {
                (void)memset((void*)&((uint8_t*)_temp_dma_buffer)[length], 0,
                             sizeof(_temp_dma_buffer) - length);
            }
        }
        else
        {
            /* Use app data buffer */
            p_dma_buffer = (uint32_t*)data;
        }

        /* Maintenance D-cache for DMA buffers */
        #if defined(_CYHAL_DCACHE_MAINTENANCE)
        if (direction == CYHAL_WRITE)
        {
            SCB_CleanDCache_by_Addr((uint32_t*)p_dma_buffer, block_size * number_of_blocks);
        }
        else
        {
            /* Cache-Invalidate the output from DMA */
            SCB_InvalidateDCache_by_Addr((uint32_t*)p_dma_buffer, block_size * number_of_blocks);
        }
        #endif /* if defined(_CYHAL_DCACHE_MAINTENANCE) */

        /* DMA configuration (use single buffer) */
        //obj->hsd->Instance->IDMACTRL  = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
        //ANN: the STM32F4 variant does not support DMA under internal SDIO/SDMMC, so, need to use the specific DMA Channel.
        //Enable the DMA2 Clock

        //__HAL_RCC_DMA2_CLK_ENABLE();//ANN commented as DMA clk is enabled earlier

        //obj->hsd->Instance->IDMABASE0 = (uint32_t)p_dma_buffer;
        //DMA2_Stream3->PAR = (uint32_t)p_dma_buffer;          // peripheral base address

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut = SDMMC_DATATIMEOUT;
        config.TransferDir = (direction == CYHAL_WRITE) ? SDIO_TRANSFER_DIR_TO_CARD :
                             SDIO_TRANSFER_DIR_TO_SDIO;
        config.TransferMode  = SDIO_TRANSFER_MODE_BLOCK;
        config.DPSM          = SDIO_DPSM_DISABLE;
        config.DataLength    = block_size * number_of_blocks;
        config.DataBlockSize = _stm32_sdio_convert_block_size((uint16_t)block_size);

        obj->hsd->Instance->DCTRL |= SDIO_DCTRL_SDIOEN; /* SD I/O enable functions */

        /* obj->hsd->Instance->DCTRL |= (uint32_t)sdio_get_blocksize_dctrl(block_size) | bus_direction_mapping[(int)direction] |
        		SDIO_TransferMode_Block | SDIO_DPSM_Enable | (1 << 3) | (1 << 11);*/

        //(void)SDMMC_ConfigData(obj->hsd->Instance, &config);
        (void)SDIO_ConfigData(obj->hsd->Instance, &config);


        //ANN added below from WICED
#if defined(WICED)
        DMA2_Stream3->CR   = 0;
        DMA2->LIFCR        = (uint32_t) ( 0x3F << 22 );
        DMA2_Stream3->FCR  = (uint32_t) ( 0x00000021 | 4 | 3 );
        DMA2_Stream3->PAR  = (uint32_t) &SDIO->FIFO;
        DMA2_Stream3->M0AR = (uint32_t) dma_data_source;
        //DMA2_Stream3->M0AR = (uint32_t) p_dma_buffer;//ANN
        DMA2_Stream3->NDTR = dma_transfer_size/4;
#endif


//ANN commented below for sometime. Below macro does ORing with SDMMC_CMD_CMDTRANS which is for CPSM Treats command as a Data Transfer  
  // __SDMMC_CMDTRANS_ENABLE(obj->hsd->Instance);


        /* Call CMD53 (CYHAL_SDIO_CMD_IO_RW_EXTENDED) */
        result = cyhal_sdio_send_cmd(obj, direction, CYHAL_SDIO_CMD_IO_RW_EXTENDED,
                                     arg.value, response);

        /* Check error from CMD53 */
        if (result != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            __HAL_SD_CLEAR_FLAG(obj->hsd, SDMMC_STATIC_FLAGS);
            obj->hsd->State = HAL_SD_STATE_READY;
            return result;
        }

        /* Check if there were any SDIO errors */
		if ( ( SDIO->STA & ( SDIO_STA_DTIMEOUT | SDIO_STA_CTIMEOUT ) ) != 0 )
		{
			return CYHAL_SDIO_RET_CMD_TIMEOUT;
		}
		else if ( ( ( SDIO->STA & ( SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR ) ) != 0 ) )
		{
			return CYHAL_SDIO_RET_CMD_TIMEOUT;
		}


/*
         Set polling flags
        flags = SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DATAEND |
                SDIO_FLAG_TXUNDERR | SDIO_FLAG_RXOVERR;
*/
        /* Poll on SDMMC flags */
        while ((!__HAL_SD_GET_FLAG(obj->hsd, flags)) && (timeout > 0u))
        {
            // returning ERROR code for timeout
            //HAL_Delay(1U);
            k_msleep(1);
            //K_MSEC	(1);
        	//for(int i=0;i<1000;i++);
            timeout--;
            if (timeout == 0u)
            {
                obj->hsd->State = HAL_SD_STATE_READY;
                return CYHAL_SDIO_RET_CMD_TIMEOUT;
            }
        }


        /* Check for SDMMC interrupt flags */
        if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_DATAEND) != RESET)
        {
            __HAL_SD_CLEAR_FLAG(obj->hsd, SDIO_FLAG_DATAEND);
        }
        else if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT |
                                   SDIO_FLAG_RXOVERR | SDIO_FLAG_TXUNDERR) != RESET)
        {
            /* Set Error code */
            if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_DCRCFAIL) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_DATA_CRC_FAIL;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_DTIMEOUT) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_DATA_TIMEOUT;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_RXOVERR) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_RX_OVERRUN;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDIO_FLAG_TXUNDERR) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_TX_UNDERRUN;
            }
        }
        else
        {
            /* Nothing to do */
        }

        /* Set SDIO to init state */
        //ANN: please ensure the obj->hsd is pointing to correct base address of required stream/channel
        __HAL_SD_DISABLE_IT(obj->hsd, SDIO_IT_TXFIFOF);
//        __HAL_SD_DISABLE_IT(obj->hsd, SDIO_IT_TXFIFOF);
        //ANN: please ensure (STM32F4) SDIO_IT_TXFIFOF == (STM32H7) SDMMC_IT_IDMABTC. refer stm32l4xx_ll_sdmmc.h & stm32f4xx_ll_sdmmc.h

        //ANN commented below as for STM32F4
        //__SDMMC_CMDTRANS_DISABLE(obj->hsd->Instance);

        obj->hsd->Instance->DLEN     = 0;
        obj->hsd->Instance->DCTRL    = SDIO_DCTRL_SDIOEN;
        //obj->hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
        __SDIO_OPERATION_DISABLE(obj->hsd->Instance);
        obj->hsd->Instance->CMD      = 0;
        //ANN: please check if it's required to reset FIFO, if so, then do it by finding right bit.
        //obj->hsd->Instance->DCTRL   |= SDMMC_DCTRL_FIFORST;

        /* Clear all the static flags */
        __HAL_SD_CLEAR_FLAG(obj->hsd, SDIO_STATIC_DATA_FLAGS);

        /* Copy received data to user */
        if ((direction == CYHAL_READ) && (use_temp_dma_buffer) && (!obj->hsd->ErrorCode))
        {
            (void)memcpy((void*)data, (void*)_temp_dma_buffer, (size_t)length);
        }

        /* Set the SD state to ready to be able to start again the process */
        _cyhal_sdio_handle->State = HAL_SD_STATE_READY;

        /* This interrupt is disabled in interrupt handler so need to enable it here */
        if (0U != ((uint32_t)CYHAL_SDIO_CARD_INTERRUPT & obj->irq))
        {
            __HAL_SD_ENABLE_IT(obj->hsd, SDIO_IT_SDIOIT);
        }
    }
    else
    {
        obj->hsd->ErrorCode |= HAL_SD_ERROR_BUSY;
        return CYHAL_SDIO_RSLT_ERR_PM_PENDING;
    }

    if (obj->hsd->ErrorCode == HAL_SD_ERROR_NONE)
    {
        result = CY_RSLT_SUCCESS;
    }
    else
    {
        result = CYHAL_SDIO_RET_NO_SP_ERRORS; /**< Non-specific error code*/
    }
    return result;
}

#endif

#pragma GCC pop_options

#if defined(__cplusplus)
}
#endif

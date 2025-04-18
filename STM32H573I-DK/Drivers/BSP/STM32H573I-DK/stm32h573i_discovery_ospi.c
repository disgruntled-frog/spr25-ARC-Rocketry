/**
  ******************************************************************************
  * @file    stm32h573i_discovery_ospi.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the MX25LM51245G
  *          OSPI memory mounted on the STM32H573I-DK board.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the MX25LM51245G Octal NOR external memory
       mounted on STM32H573I_DK board.

   (#) This driver need specific component driver (MX25LM51245G) to be included with.

   (#) MX25LM51245G Initialization steps:
       (++) Initialize the OSPI external memory using the BSP_OSPI_NOR_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            OSPI interface with the external memory.

   (#) MX25LM51245G Octal NOR memory operations
       (++) OSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_OSPI_NOR_Read()/BSP_OSPI_NOR_Write().
       (++) The function BSP_OSPI_NOR_GetInfo() returns the configuration of the OSPI memory.
            (see the OSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_OSPI_NOR_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function BSP_OSPI_NOR_Erase_Chip().
       (++) The function BSP_OSPI_NOR_GetStatus() returns the current status of the OSPI memory.
            (see the OSPI memory data sheet)
       (++) The memory access can be configured in memory-mapped mode with the call of
            function BSP_OSPI_NOR_EnableMemoryMapped(). To go back in indirect mode, the
            function BSP_OSPI_NOR_DisableMemoryMapped() should be used.
       (++) The erase operation can be suspend and resume with using functions
            BSP_OSPI_NOR_SuspendErase() and BSP_OSPI_NOR_ResumeErase()
       (++) It is possible to put the memory in deep power-down mode to reduce its consumption.
            For this, the function BSP_OSPI_NOR_EnterDeepPowerDown() should be called. To leave
            the deep power-down mode, the function BSP_OSPI_NOR_LeaveDeepPowerDown() should be called.
       (++) The function BSP_OSPI_NOR_ReadID() returns the identifier of the memory
            (see the OSPI memory data sheet)
       (++) The configuration of the interface between peripheral and memory is done by
            the function BSP_OSPI_NOR_ConfigFlash(), three modes are possible :
            - SPI : instruction, address and data on one line
            - STR OPI : instruction, address and data on eight lines with sampling on one edge of clock
            - DTR OPI : instruction, address and data on eight lines with sampling on both edgaes of clock

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h573i_discovery_ospi.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32H573I_DK
  * @{
  */

/** @defgroup STM32H573I_DK_OSPI OSPI
  * @{
  */

/* Exported variables --------------------------------------------------------*/
/** @addtogroup STM32H573I_DK_OSPI_NOR_Exported_Variables
  * @{
  */
XSPI_HandleTypeDef hospi_nor[OSPI_NOR_INSTANCES_NUMBER] = {0};
OSPI_NOR_Ctx_t Ospi_Nor_Ctx[OSPI_NOR_INSTANCES_NUMBER] =
{
  {
    OSPI_ACCESS_NONE,
    MX25LM51245G_SPI_MODE,
    MX25LM51245G_STR_TRANSFER
  }
};
/**
  * @}
  */


/* Private constants --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32H573I_DK_OSPI_NOR_Private_Variables OSPI_NOR Private Variables
  * @{
  */
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
static uint32_t OspiNor_IsMspCbValid[OSPI_NOR_INSTANCES_NUMBER] = {0};
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32H573I_DK_OSPI_NOR_Private_Functions OSPI_NOR Private Functions
  * @{
  */
static void    OSPI_NOR_MspInit(XSPI_HandleTypeDef *hospi);
static void    OSPI_NOR_MspDeInit(XSPI_HandleTypeDef *hospi);
static int32_t OSPI_NOR_ResetMemory(uint32_t Instance);
static int32_t OSPI_NOR_EnterDOPIMode(uint32_t Instance);
static int32_t OSPI_NOR_EnterSOPIMode(uint32_t Instance);
static int32_t OSPI_NOR_ExitOPIMode(uint32_t Instance);
static void OSPI1_DLYB_Enable(uint32_t Instance);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32H573I_DK_OSPI_NOR_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the OSPI interface.
  * @param  Instance   OSPI Instance
  * @param  Init       OSPI Init structure
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_Init(uint32_t Instance, BSP_OSPI_NOR_Init_t *Init)
{
  int32_t ret;
  BSP_OSPI_NOR_Info_t pInfo;
  MX_OSPI_InitTypeDef ospi_init;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (Ospi_Nor_Ctx[Instance].IsInitialized == OSPI_ACCESS_NONE)
    {
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      /* Msp OSPI initialization */
      OSPI_NOR_MspInit(&hospi_nor[Instance]);
#else
      /* Register the OSPI MSP Callbacks */
      if (OspiNor_IsMspCbValid[Instance] == 0UL)
      {
        if (BSP_OSPI_NOR_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

      /* Get Flash information of one memory */
      (void)MX25LM51245G_GetFlashInfo(&pInfo);

      /* Fill config structure */
      ospi_init.ClockPrescaler = 1;
      ospi_init.MemorySize     = (uint32_t)POSITION_VAL((uint32_t)pInfo.FlashSize);
      ospi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
      ospi_init.TransferRate   = (uint32_t)Init->TransferRate;

      /* STM32 OSPI interface initialization */
      if (MX_OSPI_NOR_Init(&hospi_nor[Instance], &ospi_init) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        /* OSPI Delay Block enable */
        OSPI1_DLYB_Enable(Instance);

        /* OSPI memory reset */
        if (OSPI_NOR_ResetMemory(Instance) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }/* Check if memory is ready */
        else if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                                  Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }/* Configure the memory */
        else if (BSP_OSPI_NOR_ConfigFlash(Instance, Init->InterfaceMode, Init->TransferRate) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  De-Initializes the OSPI interface.
  * @param  Instance   OSPI Instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (Ospi_Nor_Ctx[Instance].IsInitialized != OSPI_ACCESS_NONE)
    {
      /* Disable Memory mapped mode */
      if (Ospi_Nor_Ctx[Instance].IsInitialized == OSPI_ACCESS_MMP)
      {
        if (BSP_OSPI_NOR_DisableMemoryMappedMode(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_COMPONENT_FAILURE;
        }
      }

      /* Set default Ospi_Nor_Ctx values */
      Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_NONE;
      Ospi_Nor_Ctx[Instance].InterfaceMode = BSP_OSPI_NOR_SPI_MODE;
      Ospi_Nor_Ctx[Instance].TransferRate  = BSP_OSPI_NOR_STR_TRANSFER;

#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      OSPI_NOR_MspDeInit(&hospi_nor[Instance]);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 0) */

      /* Call the DeInit function to reset the driver */
      if (HAL_XSPI_DeInit(&hospi_nor[Instance]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Initializes the OSPI interface.
  * @param  hospi          OSPI handle
  * @param  Init           OSPI config structure
  * @retval BSP status
  */
__weak HAL_StatusTypeDef MX_OSPI_NOR_Init(XSPI_HandleTypeDef *hospi, MX_OSPI_InitTypeDef *Init)
{
  /* OctoSPI initialization */
  hospi->Instance = OCTOSPI1;

  hospi->Init.FifoThresholdByte       = 1;
  hospi->Init.MemoryMode              = HAL_XSPI_SINGLE_MEM;
  hospi->Init.MemorySize              = Init->MemorySize; /* 512 MBits */
  hospi->Init.ChipSelectHighTimeCycle = 2;
  hospi->Init.FreeRunningClock        = HAL_XSPI_FREERUNCLK_DISABLE;
  hospi->Init.ClockMode               = HAL_XSPI_CLOCK_MODE_0;
  hospi->Init.WrapSize                = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hospi->Init.ClockPrescaler          = Init->ClockPrescaler;
  hospi->Init.SampleShifting          = Init->SampleShifting;
  hospi->Init.ChipSelectBoundary      = 0;

  if (Init->TransferRate == (uint32_t) BSP_OSPI_NOR_DTR_TRANSFER)
  {
    hospi->Init.MemoryType            = HAL_XSPI_MEMTYPE_MACRONIX;
    hospi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  }
  else
  {
    hospi->Init.MemoryType            = HAL_XSPI_MEMTYPE_MICRON;
    hospi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  }

  return HAL_XSPI_Init(hospi);
}

#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief Default BSP OSPI Msp Callbacks
  * @param Instance      OSPI Instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hospi_nor[Instance], HAL_XSPI_MSP_INIT_CB_ID, OSPI_NOR_MspInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hospi_nor[Instance], HAL_XSPI_MSP_DEINIT_CB_ID, OSPI_NOR_MspDeInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      OspiNor_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief BSP OSPI Msp Callback registering
  * @param Instance     OSPI Instance
  * @param CallBacks    pointer to MspInit/MspDeInit callbacks functions
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_RegisterMspCallbacks(uint32_t Instance, BSP_OSPI_Cb_t *CallBacks)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hospi_nor[Instance], HAL_XSPI_MSP_INIT_CB_ID, CallBacks->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hospi_nor[Instance],
                                       HAL_XSPI_MSP_DEINIT_CB_ID, CallBacks->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      OspiNor_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Reads an amount of data from the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pData     Pointer to data to be read
  * @param  ReadAddr  Read start address
  * @param  Size      Size of data to read
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_Read(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (Ospi_Nor_Ctx[Instance].TransferRate == BSP_OSPI_NOR_STR_TRANSFER)
    {
      if (MX25LM51245G_ReadSTR(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               MX25LM51245G_4BYTES_SIZE, pData, ReadAddr, Size) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      if (MX25LM51245G_ReadDTR(&hospi_nor[Instance], pData, ReadAddr, Size) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pData     Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size      Size of data to write
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_Write(uint32_t Instance, const uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t end_addr;
  uint32_t current_size;
  uint32_t current_addr;
  uint32_t data_addr;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Calculation of the size between the write address and the end of the page */
    current_size = MX25LM51245G_PAGE_SIZE - (WriteAddr % MX25LM51245G_PAGE_SIZE);

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size)
    {
      current_size = Size;
    }

    /* Initialize the address variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;
    data_addr = (uint32_t)pData;

    /* Perform the write page by page */
    do
    {
      /* Check if Flash busy ? */
      if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                           Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }/* Enable write operations */
      else if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                        Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        if (Ospi_Nor_Ctx[Instance].TransferRate == BSP_OSPI_NOR_STR_TRANSFER)
        {
          /* Issue page program command */
          if (MX25LM51245G_PageProgram(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                       MX25LM51245G_4BYTES_SIZE, (uint8_t *)data_addr, current_addr,
                                       current_size) != MX25LM51245G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
        }
        else
        {
          /* Issue page program command */
          if (MX25LM51245G_PageProgramDTR(&hospi_nor[Instance], (uint8_t *)data_addr, current_addr,
                                          current_size) != MX25LM51245G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
        }

        if (ret == BSP_ERROR_NONE)
        {
          /* Configure automatic polling mode to wait for end of program */
          if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                               Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
          else
          {
            /* Update the address and size variables for next page programming */
            current_addr += current_size;
            data_addr += current_size;
            current_size = ((current_addr + MX25LM51245G_PAGE_SIZE) > end_addr)
                           ? (end_addr - current_addr)
                           : MX25LM51245G_PAGE_SIZE;
          }
        }
      }
    } while ((current_addr < end_addr) && (ret == BSP_ERROR_NONE));
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Erases the specified block of the OSPI memory.
  * @param  Instance     OSPI instance
  * @param  BlockAddress Block address to erase
  * @param  BlockSize    Erase Block size: MX25LM51245G_ERASE_4K or MX25LM51245G_ERASE_64K
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_Erase_Block(uint32_t Instance, uint32_t BlockAddress, BSP_OSPI_NOR_Erase_t BlockSize)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check Flash busy ? */
    if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                         Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Enable write operations */
    else if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                      Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Issue Block Erase command */
    else if (MX25LM51245G_BlockErase(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                     Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_4BYTES_SIZE,
                                     BlockAddress, BlockSize) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Erases the entire OSPI memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_Erase_Chip(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check Flash busy ? */
    if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                         Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Enable write operations */
    else if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                      Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Issue Chip erase command */
    else if (MX25LM51245G_ChipErase(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                    Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Reads current status of the OSPI memory.
  * @param  Instance  OSPI instance
  * @retval OSPI memory status: whether busy or not
  */
int32_t BSP_OSPI_NOR_GetStatus(uint32_t Instance)
{
  static uint8_t reg[2];
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (MX25LM51245G_ReadSecurityRegister(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                          Ospi_Nor_Ctx[Instance].TransferRate, reg) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Check the value of the register */
    else if ((reg[0] & (MX25LM51245G_SECR_P_FAIL | MX25LM51245G_SECR_E_FAIL)) != 0U)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if ((reg[0] & (MX25LM51245G_SECR_PSB | MX25LM51245G_SECR_ESB)) != 0U)
    {
      ret = BSP_ERROR_OSPI_SUSPENDED;
    }
    else if (MX25LM51245G_ReadStatusRegister(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                             Ospi_Nor_Ctx[Instance].TransferRate, reg) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Check the value of the register */
    else if ((reg[0] & MX25LM51245G_SR_WIP) != 0U)
    {
      ret = BSP_ERROR_BUSY;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Return the configuration of the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pInfo     pointer on the configuration structure
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_GetInfo(uint32_t Instance, BSP_OSPI_NOR_Info_t *pInfo)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    (void)MX25LM51245G_GetFlashInfo(pInfo);
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Configure the OSPI in memory-mapped mode
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (Ospi_Nor_Ctx[Instance].TransferRate == BSP_OSPI_NOR_STR_TRANSFER)
    {
      if (MX25LM51245G_EnableMemoryMappedModeSTR(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                                 MX25LM51245G_4BYTES_SIZE) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else /* Update OSPI context if all operations are well done */
      {
        Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_MMP;
      }
    }
    else
    {
      if (MX25LM51245G_EnableMemoryMappedModeDTR(&hospi_nor[Instance],
                                                 Ospi_Nor_Ctx[Instance].InterfaceMode) != MX25LM51245G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else /* Update OSPI context if all operations are well done */
      {
        Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_MMP;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Exit form memory-mapped mode
  *         Only 1 Instance can running MMP mode. And it will lock system at this mode.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_DisableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (Ospi_Nor_Ctx[Instance].IsInitialized != OSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_OSPI_MMP_UNLOCK_FAILURE;
    }/* Abort MMP back to indirect mode */
    else if (HAL_XSPI_Abort(&hospi_nor[Instance]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else /* Update OSPI NOR context if all operations are well done */
    {
      Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Get flash ID 3 Bytes:
  *         Manufacturer ID, Memory type, Memory density
  * @param  Instance  OSPI instance
  * @param  Id Pointer to flash ID bytes
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX25LM51245G_ReadID(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               Ospi_Nor_Ctx[Instance].TransferRate, Id) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Set Flash to desired Interface mode. And this instance becomes current instance.
  *         If current instance running at MMP mode then this function doesn't work.
  *         Indirect -> Indirect
  * @param  Instance  OSPI instance
  * @param  Mode      OSPI mode
  * @param  Rate      OSPI transfer rate
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_ConfigFlash(uint32_t Instance, BSP_OSPI_NOR_Interface_t Mode, BSP_OSPI_NOR_Transfer_t Rate)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if MMP mode locked ************************************************/
    if (Ospi_Nor_Ctx[Instance].IsInitialized == OSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_OSPI_MMP_LOCK_FAILURE;
    }
    else
    {
      /* Setup Flash interface ***************************************************/
      switch (Ospi_Nor_Ctx[Instance].InterfaceMode)
      {
        case BSP_OSPI_NOR_OPI_MODE :  /* 8-8-8 commands */
          if ((Mode != BSP_OSPI_NOR_OPI_MODE) || (Rate != Ospi_Nor_Ctx[Instance].TransferRate))
          {
            /* Exit OPI mode */
            ret = OSPI_NOR_ExitOPIMode(Instance);

            if ((ret == BSP_ERROR_NONE) && (Mode == BSP_OSPI_NOR_OPI_MODE))
            {

              if (Ospi_Nor_Ctx[Instance].TransferRate == BSP_OSPI_NOR_STR_TRANSFER)
              {
                /* Enter DTR OPI mode */
                ret = OSPI_NOR_EnterDOPIMode(Instance);
              }
              else
              {
                /* Enter STR OPI mode */
                ret = OSPI_NOR_EnterSOPIMode(Instance);
              }
            }
          }
          break;

        case BSP_OSPI_NOR_SPI_MODE :  /* 1-1-1 commands, Power on H/W default setting */
        default :
          if (Mode == BSP_OSPI_NOR_OPI_MODE)
          {
            if (Rate == BSP_OSPI_NOR_STR_TRANSFER)
            {
              /* Enter STR OPI mode */
              ret = OSPI_NOR_EnterSOPIMode(Instance);
            }
            else
            {
              /* Enter DTR OPI mode */
              ret = OSPI_NOR_EnterDOPIMode(Instance);
            }
          }
          break;
      }

      /* Update OSPI context if all operations are well done */
      if (ret == BSP_ERROR_NONE)
      {
        /* Update current status parameter *****************************************/
        Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
        Ospi_Nor_Ctx[Instance].InterfaceMode = Mode;
        Ospi_Nor_Ctx[Instance].TransferRate  = Rate;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_SuspendErase(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  /* Check whether the device is busy (erase operation is in progress). */
  else if (BSP_OSPI_NOR_GetStatus(Instance) != BSP_ERROR_BUSY)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_Suspend(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (BSP_OSPI_NOR_GetStatus(Instance) != BSP_ERROR_OSPI_SUSPENDED)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function resumes a paused erase command.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_ResumeErase(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  /* Check whether the device is busy (erase operation is in progress). */
  else if (BSP_OSPI_NOR_GetStatus(Instance) != BSP_ERROR_OSPI_SUSPENDED)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_Resume(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /*
  When this command is executed, the status register write in progress bit is set to 1, and
  the flag status register program erase controller bit is set to 0. This command is ignored
  if the device is not in a suspended state.
  */
  else if (BSP_OSPI_NOR_GetStatus(Instance) != BSP_ERROR_BUSY)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enter the OSPI memory in deep power down mode.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_EnterDeepPowerDown(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX25LM51245G_EnterPowerDown(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                       Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* ---          Memory takes 10us max to enter deep power down          --- */

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function leave the OSPI memory from deep power down mode.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_NOR_LeaveDeepPowerDown(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX25LM51245G_NoOperation(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                    Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* --- A NOP command is sent to the memory, as the nCS should be low for at least 20 ns --- */
  /* ---                  Memory takes 30us min to leave deep power down                  --- */

  /* Return BSP status */
  return ret;
}
/**
  * @}
  */

/** @addtogroup STM32H573I_DK_OSPI_NOR_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the OSPI MSP.
  * @param  hospi OSPI handle
  * @retval None
  */
static void OSPI_NOR_MspInit(XSPI_HandleTypeDef *hospi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  /* Enable the OctoSPI memory interface clock */
  OSPI_NOR_CLK_ENABLE();

  /* Reset the OctoSPI memory interface */
  OSPI_NOR_FORCE_RESET();
  OSPI_NOR_RELEASE_RESET();

  /* Enable GPIO clocks */
  OSPI_NOR_CLK_GPIO_CLK_ENABLE();
  OSPI_NOR_DQS_GPIO_CLK_ENABLE();
  OSPI_NOR_CS_GPIO_CLK_ENABLE();
  OSPI_NOR_D0_GPIO_CLK_ENABLE();
  OSPI_NOR_D1_GPIO_CLK_ENABLE();
  OSPI_NOR_D2_GPIO_CLK_ENABLE();
  OSPI_NOR_D3_GPIO_CLK_ENABLE();
  OSPI_NOR_D4_GPIO_CLK_ENABLE();
  OSPI_NOR_D5_GPIO_CLK_ENABLE();
  OSPI_NOR_D6_GPIO_CLK_ENABLE();
  OSPI_NOR_D7_GPIO_CLK_ENABLE();

  /* Activate HSLV */
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_CS_GPIO_PORT, OSPI_NOR_CS_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_CLK_GPIO_PORT, OSPI_NOR_CLK_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D0_GPIO_PORT, OSPI_NOR_D0_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D1_GPIO_PORT, OSPI_NOR_D1_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D2_GPIO_PORT, OSPI_NOR_D3_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D3_GPIO_PORT, OSPI_NOR_D3_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D4_GPIO_PORT, OSPI_NOR_D4_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D5_GPIO_PORT, OSPI_NOR_D5_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D6_GPIO_PORT, OSPI_NOR_D6_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_D7_GPIO_PORT, OSPI_NOR_D7_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_NOR_DQS_GPIO_PORT, OSPI_NOR_DQS_PIN);

  /* OctoSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = OSPI_NOR_CS_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_CS_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI DQS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_DQS_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_DQS_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_DQS_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = OSPI_NOR_CLK_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D0_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D0_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D0_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D1_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D1_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D1_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D2_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D2_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D2_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D3_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D3_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D3_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D4 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D4_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D4_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D4_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D5 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D5_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D5_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D5_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D6 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D6_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D6_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D6_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D7 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_NOR_D7_PIN;
  GPIO_InitStruct.Alternate = OSPI_NOR_D7_PIN_AF;
  HAL_GPIO_Init(OSPI_NOR_D7_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  De-Initializes the OSPI MSP.
  * @param  hospi OSPI handle
  * @retval None
  */
static void OSPI_NOR_MspDeInit(XSPI_HandleTypeDef *hospi)
{
  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  /* OctoSPI GPIO pins de-configuration  */
  HAL_GPIO_DeInit(OSPI_NOR_CLK_GPIO_PORT, OSPI_NOR_CLK_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_DQS_GPIO_PORT, OSPI_NOR_DQS_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_CS_GPIO_PORT, OSPI_NOR_CS_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D0_GPIO_PORT, OSPI_NOR_D0_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D1_GPIO_PORT, OSPI_NOR_D1_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D2_GPIO_PORT, OSPI_NOR_D2_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D3_GPIO_PORT, OSPI_NOR_D3_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D4_GPIO_PORT, OSPI_NOR_D4_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D5_GPIO_PORT, OSPI_NOR_D5_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D6_GPIO_PORT, OSPI_NOR_D6_PIN);
  HAL_GPIO_DeInit(OSPI_NOR_D7_GPIO_PORT, OSPI_NOR_D7_PIN);

  /* Reset the OctoSPI memory interface */
  OSPI_NOR_FORCE_RESET();
  OSPI_NOR_RELEASE_RESET();

  /* Disable the OctoSPI memory interface clock */
  OSPI_NOR_CLK_DISABLE();
}

/**
  * @brief  This function reset the OSPI memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static int32_t OSPI_NOR_ResetMemory(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (MX25LM51245G_ResetEnable(&hospi_nor[Instance], BSP_OSPI_NOR_SPI_MODE,
                               BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_ResetMemory(&hospi_nor[Instance], BSP_OSPI_NOR_SPI_MODE,
                                    BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_ResetEnable(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                    BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_ResetMemory(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                    BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_ResetEnable(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                    BSP_OSPI_NOR_DTR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX25LM51245G_ResetMemory(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                    BSP_OSPI_NOR_DTR_TRANSFER) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;     /* After reset S/W setting to indirect access  */
    Ospi_Nor_Ctx[Instance].InterfaceMode = BSP_OSPI_NOR_SPI_MODE;    /* After reset H/W back to SPI mode by default */
    Ospi_Nor_Ctx[Instance].TransferRate  = BSP_OSPI_NOR_STR_TRANSFER; /* After reset S/W setting to STR mode        */

    /* Wait SWreset CMD is effective and check that memory is ready */
    if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                         Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enables the octal DTR mode of the memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static int32_t OSPI_NOR_EnterDOPIMode(uint32_t Instance)
{
  int32_t ret;
  uint8_t reg[2];

  /* Enable write operations */
  if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with new dummy cycles) */
  else if (MX25LM51245G_WriteCfg2Register(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                          Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_CR2_REG3_ADDR,
                                          MX25LM51245G_CR2_DC_6_CYCLES) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Enable write operations */
  else if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                    Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  else if (MX25LM51245G_WriteCfg2Register(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                          Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_CR2_REG1_ADDR,
                                          MX25LM51245G_CR2_DOPI) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Reconfigure the memory type of the peripheral */
    hospi_nor[Instance].Init.MemoryType            = HAL_XSPI_MEMTYPE_MACRONIX;
    hospi_nor[Instance].Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
    if (HAL_XSPI_Init(&hospi_nor[Instance]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    /* Check Flash busy ? */
    else if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                              BSP_OSPI_NOR_DTR_TRANSFER) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    /* Check the configuration has been correctly done */
    else if (MX25LM51245G_ReadCfg2Register(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE, BSP_OSPI_NOR_DTR_TRANSFER,
                                           MX25LM51245G_CR2_REG1_ADDR, reg) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (reg[0] != MX25LM51245G_CR2_DOPI)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enables the octal STR mode of the memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static int32_t OSPI_NOR_EnterSOPIMode(uint32_t Instance)
{
  int32_t ret;
  uint8_t reg[2];

  /* Enable write operations */
  if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with new dummy cycles) */
  else if (MX25LM51245G_WriteCfg2Register(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                          Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_CR2_REG3_ADDR,
                                          MX25LM51245G_CR2_DC_6_CYCLES) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Enable write operations */
  else if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                    Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  else if (MX25LM51245G_WriteCfg2Register(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                          Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_CR2_REG1_ADDR,
                                          MX25LM51245G_CR2_SOPI) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Check Flash busy ? */
    if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE,
                                         BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    /* Check the configuration has been correctly done */
    else if (MX25LM51245G_ReadCfg2Register(&hospi_nor[Instance], BSP_OSPI_NOR_OPI_MODE, BSP_OSPI_NOR_STR_TRANSFER,
                                           MX25LM51245G_CR2_REG1_ADDR, reg) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (reg[0] != MX25LM51245G_CR2_SOPI)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function disables the octal DTR or STR mode of the memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static int32_t OSPI_NOR_ExitOPIMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t reg[2];

  /* Enable write operations */
  if (MX25LM51245G_WriteEnable(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                               Ospi_Nor_Ctx[Instance].TransferRate) != MX25LM51245G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Write Configuration register 2 (with SPI protocol) */
    reg[0] = 0;
    reg[1] = 0;
    if (MX25LM51245G_WriteCfg2Register(&hospi_nor[Instance], Ospi_Nor_Ctx[Instance].InterfaceMode,
                                       Ospi_Nor_Ctx[Instance].TransferRate, MX25LM51245G_CR2_REG1_ADDR,
                                       reg[0]) != MX25LM51245G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      if (Ospi_Nor_Ctx[Instance].TransferRate == BSP_OSPI_NOR_DTR_TRANSFER)
      {
        /* Reconfigure the memory type of the peripheral */
        hospi_nor[Instance].Init.MemoryType            = HAL_XSPI_MEMTYPE_MICRON;
        hospi_nor[Instance].Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
        if (HAL_XSPI_Init(&hospi_nor[Instance]) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        /* Check Flash busy ? */
        if (MX25LM51245G_AutoPollingMemReady(&hospi_nor[Instance], BSP_OSPI_NOR_SPI_MODE,
                                             BSP_OSPI_NOR_STR_TRANSFER) != MX25LM51245G_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        /* Check the configuration has been correctly done */
        else if (MX25LM51245G_ReadCfg2Register(&hospi_nor[Instance], BSP_OSPI_NOR_SPI_MODE, BSP_OSPI_NOR_STR_TRANSFER,
                                               MX25LM51245G_CR2_REG1_ADDR, reg) != MX25LM51245G_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else if (reg[0] != 0U)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          /* Nothing to do */
        }
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enables delay block.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static void OSPI1_DLYB_Enable(uint32_t Instance)
{
  HAL_XSPI_DLYB_CfgTypeDef  dlyb_cfg;

  (void)HAL_XSPI_DLYB_GetClockPeriod(&hospi_nor[Instance], &dlyb_cfg);

  /*when DTR, PhaseSel is divided by 4 (emperic value)*/
  dlyb_cfg.PhaseSel /= 4U;

  /*set delay block configuration*/
  (void)HAL_XSPI_DLYB_SetConfig(&hospi_nor[Instance], &dlyb_cfg);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

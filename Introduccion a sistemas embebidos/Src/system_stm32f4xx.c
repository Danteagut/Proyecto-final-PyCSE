/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32F4xx devices.
  ******************************************************************************
  */

#include "stm32f4xx.h"
#define VECT_TAB_OFFSET  0x00U
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    8000000U
#endif
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U
#endif

uint32_t SystemCoreClock = 16000000U;
const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10U * 2U) | (3UL << 11U * 2U));
#endif

  RCC->CR |= (uint32_t)0x00000001U;
  RCC->CFGR = 0x00000000U;
  RCC->CR &= (uint32_t)0xFEF6FFFFU;
  RCC->PLLCFGR = 0x24003010U;
  RCC->CR &= (uint32_t)0xFFFBFFFFU;
  RCC->CIR = 0x00000000U;

#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl();
#endif

#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif
}

void SystemCoreClockUpdate(void)
{
  uint32_t tmp;
  uint32_t pllvco;
  uint32_t pllp;
  uint32_t pllsource;
  uint32_t pllm;

  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:
      SystemCoreClock = HSI_VALUE;
      break;

    case RCC_CFGR_SWS_HSE:
      SystemCoreClock = HSE_VALUE;
      break;

    case RCC_CFGR_SWS_PLL:
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22U;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0U)
      {
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U);
      }
      else
      {
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16U) + 1U) * 2U;
      SystemCoreClock = pllvco / pllp;
      break;

    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }

  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
  SystemCoreClock >>= tmp;
}

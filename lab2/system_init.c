#include <stm32f10x.h>

void HSE_Configuration(void);

int main(void)
{
  
  HSE_Configuration();
  
  
}

void HSE_Configuration(void){
  // HSE Clock Enable
  RCC->CR |= RCC_CR_HSEON;
  
  // PLLXTPRE
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;
  
  // PLLSRC select HSE from PLLXTPRE
  RCC->CFGR |= RCC_CFGR_PLLSRC;
  
  // PLLMUL input clk x4 (0010)
  RCC->CFGR &= ~(RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_2 | RCC_CFGR_PLLMULL_3);
  RCC->CFGR |= RCC_CFGR_PLLMULL_1;
  
  // PLL selected as system clock
  RCC->CFGR &= ~RCC_CFGR_SW_0;
  RCC->CFGR |= RCC_CFGR_SW_1;
  
}
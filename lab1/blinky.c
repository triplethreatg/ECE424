// Default 8 MHz internal clock
 
#include "stm32f10x_conf.h"
#include "system_stm32f10x.c"
 
#define F_CPU           8000000   // 1MHz
#define PRESCALE        64
#define PERIOD          500 // microseconds - 1/2 second delay
#define TCLKS           ((F_CPU/PRESCALE*PERIOD)/1000)
 
volatile uint8_t flag1 = 0;
 
void TIM1_CC_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    if(flag1 == 1)
      flag1 = 0;
    else
      flag1 = 1;
    TIM1->SR = ~TIM_SR_CC1IF;   // clear CC1 int flag (write 0)
  }
}
 
int main(void)
{
  RCC->CFGR = 0;                       // HSI, 8 MHz, 
 
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB
  
  // Configure PB5 *
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // enable Timer1
 
  TIM1->PSC = PRESCALE - 1;
  TIM1->DIER = TIM_DIER_CC1IE; // enable 2 CC interrupt channels
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  NVIC->ISER[0] = (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
 
  while (1)
  {
    if (flag1)
      // toggle green LED
      GPIOB->BRR = 0b00100000;
    else
      GPIOB->BSRR = 0b00100000;
  }
}

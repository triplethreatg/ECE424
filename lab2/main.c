// HSE 72 MHz internal clock
 
#include "stm32f10x_conf.h"
#include "lab2.h"

unsigned char pidControl = 1;
uint8_t count = 0;

MotorSpeeds* p_motorSpeedsPtr;

// Timer 1 Capture and Compare IRQ Handler
void TIM1_CC_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    detectEmergency();
    TIM1->SR &= ~TIM_SR_CC1IF;   // clear CC1 flag (write 0)
  }
}

// Timer 2 IRQ Handler
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    refreshSensorData();
    TIM2->SR &= ~TIM_SR_CC1IF; // clear CC1 flag (write 0)
  }
}

// Timer 3 IRQ Handler
void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    calculateOrientation();
    TIM3->SR &= ~TIM_SR_CC1IF;  // clear CC1 flag (write 0)
  }
}

// Timer 4 IRQ Handler
void TIM4_IRQHandler(void)
{
  if (TIM4->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    pidControl = updatePid(p_motorSpeedsPtr); 
    TIM4->SR &= ~TIM_SR_CC1IF;  // 
  }
}

// Configure System Tick Timer
void SysTick_Configuration(void){

  // Clock source selection (AHB/8)
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;

  // Initialize to 2250000
  SysTick->LOAD = 2250000;

  // SysTick exception request enable
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT;
  
  // Counter enable
  SysTick->CTRL |= SysTick_CTRL_ENABLE;
  
}

// Configure Timer 1
void TIM1_Configuration(void)
{
  // TIM1
  TIM1->PSC = 35;   // Set prescale
  TIM1->CR1 |= TIM_CR1_ARPE;   // Auto reload preload enabled
  TIM1->ARR = 55535;   // Auto reload
  TIM1->DIER |= TIM_DIER_CC1IE;		// enable CC interrupt channel
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  // Set priority to 1
  NVIC_SetPriority (TIM1_CC_IRQn, NVIC_IPR0_PRI_1);
  NVIC->ISER[0] |= (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
}

// Configure Timer 2
void TIM2_Configuration(void)
{
  
  // Set prescale
  TIM2->PSC = 71;
  TIM2->CR1 |= TIM_CR1_ARPE;		// Auto-reload preload enabled
  TIM2->ARR = 15535;	// Auto reload value
  //
  TIM2->DIER |= TIM_DIER_CC1IE;		// enable CC interrupt channel
  TIM2->CCMR1 = 0;                     // chan 1 is output
  TIM2->CR1 = TIM_CR1_CEN;             // enable timer

  // Set priority to 2
  NVIC_SetPriority (TIM2_IRQn, NVIC_IPR0_PRI_2);
  NVIC->ISER[0] |= (1 << TIM2_IRQn); // enable TIM2 int in NVIC
}

// Configure Timer 3
void TIM3_Configuration(void)
{

  // Clear CR1
  TIM3->CR1 = 0x00;
  
  // set prescale value
  TIM3->PSC = 71;
  TIM3->CR1 |= TIM_CR1_ARPE;    // Auto-reload preload enabled
  TIM3->ARR = 15530;		// Auto reload value
  
  // Value determined by CCRx register
  
  /*
  The PWM mode can be selected independently on each channel (one PWM per OCx
  output) by writing 110 (PWM mode 1) in the OCxM bits in the
  TIMx_CCMRx register. enable the corresponding preload register by setting the
  OCxPE bit in the TIMx_CCMRx register
  */
  TIM3->CCMR2 = 0;
  TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  
  TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  
  // Stop motors
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  
  /*
   * As the preload registers are transferred to the shadow registers only when an update event
  occurs, before starting the counter, you have to initialize all the registers by setting the UG
  bit in the TIMx_EGR register.
  */ 
  TIM3->EGR |= TIM_EGR_UG;

  // main output enable
  TIM3->BDTR |= TIM_BDTR_MOE;
  
  /*
   * OCx polarity is software programmable using the CCxP bit in the TIMx_CCER register. It
  can be programmed as active high or active low. OCx output is enabled by the CCxE bit in
  the TIMx_CCER register. Refer to the TIMx_CCERx register description for more details.
   */
  // active high programed
  TIM3->CCER &= ~(TIM_CCER_CC3P |TIM_CCER_CC4P);
  // output enabled
  TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
  
  /*
   * OCx output is enabled by a combination of
the CCxE, CCxNE, MOE, OSSI and OSSR bits (TIMx_CCER and TIMx_BDTR registers).
Refer to the TIMx_CCER register description for more details.
*/
  TIM3->BDTR |= TIM_BDTR_OSSI;
  TIM3->BDTR |= TIM_BDTR_OSSR;
  
  
  /*
   * The timer is able to generate PWM in edge-aligned mode or center-aligned mode
depending on the CMS bits in the TIMx_CR1 register.
   */
  TIM3->CR1 = TIM_CR1_CEN;             // enable timer
  
  // main output enable
  TIM_CtrlPWMOutputs(TIM3, ENABLE);

  // set priority to 3
  NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_3);
  NVIC->ISER[0] |= (1 << TIM3_IRQn); // enable TIM2 int in NVIC
}

// Configure Timer 4
void TIM4_Configuration(void)
{
  // clear CR1 register
  TIM4->CR1 = 0x00;
  // set prescale value
  TIM4->PSC = 1023;
  TIM4->CR1 |= TIM_CR1_ARPE;		// Auto-reload enable
  TIM4->ARR = 30380;			// Set auto reload value
  
  TIM4->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  TIM4->CCMR1 = 0;                     // chan 1 is output
  
  // clear ccmr2
  TIM4->CCMR2 = 0;
  // pwm mode 1
  TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  // output enabled
  TIM4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  
  // stop motors
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;
  
  /*
   * As the preload registers are transferred to the shadow registers only when an update event
  occurs, before starting the counter, you have to initialize all the registers by setting the UG
  bit in the TIMx_EGR register.
  */
  TIM4->EGR |= TIM_EGR_UG;
  // main output enable
  TIM4->BDTR |= TIM_BDTR_MOE;
  
  /*
   * OCx polarity is software programmable using the CCxP bit in the TIMx_CCER register. It
  can be programmed as active high or active low. OCx output is enabled by the CCxE bit in
  the TIMx_CCER register. Refer to the TIMx_CCERx register description for more details.
   */
  TIM4->CCER &= ~(TIM_CCER_CC3P |TIM_CCER_CC4P);
  TIM4->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
  
  TIM4->BDTR |= TIM_BDTR_OSSI;
  TIM4->BDTR |= TIM_BDTR_OSSR;

  TIM4->CR1 = TIM_CR1_CEN;             // enable timer
  
  TIM_CtrlPWMOutputs(TIM4, ENABLE);	// main output enable
  
  // priority 4
  NVIC_SetPriority (TIM4_IRQn, NVIC_IPR1_PRI_4);
  NVIC->ISER[0] |= (1 << TIM4_IRQn); // enable TIM2 int in NVIC
}

// Configure RCC
void RCC_Configuration(void)
{

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // enable Timer1
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;  // enable Timer2 3 4

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
}

// Configure System Clock
void SYSCLK_Configuration(void)
{
  // Stop RCC
  RCC_DeInit();
  
  RCC_HSEConfig(RCC_HSE_ON);	// turn HSE on
  
  // detect status of HSE
  ErrorStatus HSEStartUpStatus;
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if (HSEStartUpStatus = SUCCESS)
  {
 
    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
    
    // Disable the PLL
    RCC_PLLCmd(DISABLE);
    
    // Divide HCLK by 2
    RCC_HCLKConfig(RCC_SYSCLK_Div2);
    
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    RCC_PCLK1Config(RCC_HCLK_Div1);
    
    // DIV2
    //RCC->CFGR |= RCC_CFGR_PLLXTPRE;  
    // 0111: PLL input clock x 9 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);    
    
    // Enable the PLL
    RCC_PLLCmd(ENABLE);
    
    // Poll until the PLL is ready
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    // Select PLLCLK as SYSCLK source
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    // Poll until PLLCLK is slected as SYSCLK
    while(RCC_GetSYSCLKSource() != 0x08);
  }
  else
    for(;;);

}

// Test Motors
void MotorTest(void)
{
  
    while(count < 4)
    {
      while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
      count++;
    }
    count = 0;
    TIM3->CCR3 = 1553;
    TIM3->CCR4 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
    
    while(count < 4)
    {
      while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
      count++;
    }
    count = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 1553;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
    
    while(count < 4)
    {
      while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
      count++;
    }
    count = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM4->CCR3 = 3038;
    TIM4->CCR4 = 0;
    
    while(count < 4)
    {
      while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
      count++;
    }
    count = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 3038;
  
}

// Run motors depending on pidUpdate
void MotorRun(void)
{
   // if pid is 1 run at 10%
   if(pidControl)
   {
	TIM3->CCR3 = 1553;
	TIM3->CCR4 = 1553;
	TIM4->CCR3 = 3038;
	TIM4->CCR4 = 3038;
   }
   // else stop motors
   else
   {
        TIM3->CCR3 = 0;
        TIM3->CCR4 = 0;
        TIM4->CCR3 = 0;
        TIM4->CCR4 = 0;
    }
}

// Run Motors depending on motorSpeed pointer
void updatePidHandler(void)
{
  if(p_motorSpeedsPtr->m1)
    TIM3->CCR3 = 1553;
  else
    TIM3->CCR3 = 0;
  if(p_motorSpeedsPtr->m2)
    TIM3->CCR4 = 1553;
  else
    TIM3->CCR4 = 0;
  if(p_motorSpeedsPtr->m3)
    TIM4->CCR3 = 3038;
  else
    TIM4->CCR3 = 0;
  if(p_motorSpeedsPtr->m4)
    TIM4->CCR3 = 3038;
  else
    TIM4->CCR3 = 0;
}

int main(void)
{
  // setup SYSCLK
  SYSCLK_Configuration();
  
  // setup RCC
  RCC_Configuration();
  
  // Remap JTRST so PB4 (red LED) can be used as 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);
  
  // setup system tick timer
  SysTick_Configuration();

  // Configure PB5 (green LED) and PB4 (red LED)
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Configure Timer3 & 4 to be used as push pull alternative function on pins 0 1 8 9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
  
  // Setup timers
  TIM1_Configuration();
  TIM2_Configuration();
  TIM3_Configuration();
  TIM4_Configuration();
 
  // Loop continuously
  while (1)
  {
    // run motors
    MotorRun();
    // Poll .25 seconds using System Tick Timer
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
    // Set Red LED
    GPIOB->BRR  |= GPIO_BRR_BR4;

    // Poll .25 seconds using System Tick Timer
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
    // Clear Red LED
    GPIOB->BSRR |= GPIO_BSRR_BS4;
    // Toggle Green LED
    GPIOB->ODR ^= GPIO_ODR_ODR5;
    
    logDebugInfo();
    
  }
}

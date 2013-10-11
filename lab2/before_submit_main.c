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
    //TIM1->CCR1 += TCLKS1;        // next interrupt time
    TIM1->SR &= ~TIM_SR_CC1IF;   // clear CC1 int flag (write 0)
  }
}

// Timer 2 IRQ Handler
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_CC1IF){
    refreshSensorData();

    TIM2->SR &= ~TIM_SR_CC1IF;
    //TIM2->CCR1 += TCLKS1;
  }
}

// Timer 3 IRQ Handler
void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC1IF){
    calculateOrientation();

    //TIM3->CCR1 += TCLKS1;        // next interrupt time   
    TIM3->SR &= ~TIM_SR_CC1IF;
  }
}

// Timer 4 IRQ Handler
void TIM4_IRQHandler(void)
{
  if (TIM4->SR & TIM_SR_CC1IF){
    pidControl = updatePid(p_motorSpeedsPtr);
    
    TIM4->SR &= ~TIM_SR_CC1IF;
  }
}

// Configure System Tick Timer
void SysTick_Configuration(void){

  // Clock source selection (AHB/8)
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;

  SysTick->LOAD = 2250000;

  // SysTick exception request enable
  // SysTick->CTRL |= SysTick_CTRL_TICKINT;
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT;
  
  // Counter enable
  SysTick->CTRL |= SysTick_CTRL_ENABLE;
  
}

// Configure Timer 1
void TIM1_Configuration(void)
{
  // TIM1
  TIM1->PSC = 35;
  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->ARR = 55535;
  TIM1->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  NVIC_SetPriority (TIM1_CC_IRQn, NVIC_IPR0_PRI_1);
  NVIC->ISER[0] |= (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
}

// Configure Timer 2
void TIM2_Configuration(void)
{
  
  // APB1 Prescaler /2 - 100: HCLK divided by 2
  //RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1);
  //RCC->CFGR |= RCC_CFGR_PPRE1_2;

  //TIM2->PSC = PRESCALE - 1;
  //
  // Auto-reload preload enabled
  TIM2->PSC = 71;
  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->ARR = 15535;
  //
  TIM2->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM2->CCMR1 = 0;                     // chan 1 is output
  TIM2->CR1 = TIM_CR1_CEN;             // enable timer

  NVIC_SetPriority (TIM2_IRQn, NVIC_IPR0_PRI_2);
  NVIC->ISER[0] |= (1 << TIM2_IRQn); // enable TIM2 int in NVIC
}

// Configure Timer 3
void TIM3_Configuration(void)
{

  TIM3->CR1 = 0x00;
  //TIM3->CR1 |= TIM_CR1_CMS_0;
  //TIM3->CR1 &= ~TIM_CR1_CMS_1;
  
  TIM3->PSC = 71;
  TIM3->CR1 |= TIM_CR1_ARPE;
  TIM3->ARR = 15530;
  
  // Value determined by CCRx register
  
  /*
  The PWM mode can be selected independently on each channel (one PWM per OCx
  output) by writing 110 (PWM mode 1) or ‘111 (PWM mode 2) in the OCxM bits in the
  TIMx_CCMRx register. You must enable the corresponding preload register by setting the
  OCxPE bit in the TIMx_CCMRx register, and eventually the auto-reload preload register (in
  upcounting or center-aligned modes) by setting the ARPE bit in the TIMx_CR1 register.
  */
  TIM3->CCMR2 = 0;
  TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  
  TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  
  //TIM3->CR1 |= TIM_CR1_ARPE;
  
  TIM3->CCR3 = 0;
  //TIM3->CCR3 = 13107;
  TIM3->CCR4 = 0;
  
  /*
   * As the preload registers are transferred to the shadow registers only when an update event
  occurs, before starting the counter, you have to initialize all the registers by setting the UG
  bit in the TIMx_EGR register.
  */
  
  TIM3->EGR |= TIM_EGR_UG;
  
  
  
  TIM3->BDTR |= TIM_BDTR_MOE;
  
  /*
   * OCx polarity is software programmable using the CCxP bit in the TIMx_CCER register. It
  can be programmed as active high or active low. OCx output is enabled by the CCxE bit in
  the TIMx_CCER register. Refer to the TIMx_CCERx register description for more details.
   */
  TIM3->CCER &= ~(TIM_CCER_CC3P |TIM_CCER_CC4P);
  //TIM3->CCER |= TIM_CCER_CC3P;
  // TIM3->CCER &= ~TIM_CCER_CC4P;
  TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
  
  /*
   * OCx output is enabled by a combination of
the CCxE, CCxNE, MOE, OSSI and OSSR bits (TIMx_CCER and TIMx_BDTR registers).
Refer to the TIMx_CCER register description for more details.
*/
  
  //TIM3->BDTR |= TIM_BDTR_MOE;
  TIM3->BDTR |= TIM_BDTR_OSSI;
  TIM3->BDTR |= TIM_BDTR_OSSR;
  
  
  /*
   * The timer is able to generate PWM in edge-aligned mode or center-aligned mode
depending on the CMS bits in the TIMx_CR1 register.
   */
  
  //TIM3->CR1 |= TIM_CR1_OPM;

  
  // TIM3->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  //TIM3->CCMR1 = 0;                     // chan 1 is output
  //TIM3->CR1 |= TIM_CR1_ARPE;
  TIM3->CR1 = TIM_CR1_CEN;             // enable timer
  
  TIM_CtrlPWMOutputs(TIM3, ENABLE);

  NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_3);
  NVIC->ISER[0] |= (1 << TIM3_IRQn); // enable TIM2 int in NVIC
}

// Configure Timer 4
void TIM4_Configuration(void)
{
  TIM4->CR1 = 0x00;
  TIM4->PSC = 1023;
  TIM4->CR1 |= TIM_CR1_ARPE;
  TIM4->ARR = 30380;
  
  TIM4->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  TIM4->CCMR1 = 0;                     // chan 1 is output
  
  /****************************************************/

  TIM4->CCMR2 = 0;
  TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  
  TIM4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  
  //TIM3->CR1 |= TIM_CR1_ARPE;
  
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;
  
  /*
   * As the preload registers are transferred to the shadow registers only when an update event
  occurs, before starting the counter, you have to initialize all the registers by setting the UG
  bit in the TIMx_EGR register.
  */
  
  TIM4->EGR |= TIM_EGR_UG;

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
  
  /***************************************************/

  TIM4->CR1 = TIM_CR1_CEN;             // enable timer
  
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  
  NVIC_SetPriority (TIM4_IRQn, NVIC_IPR1_PRI_4);
  NVIC->ISER[0] |= (1 << TIM4_IRQn); // enable TIM2 int in NVIC
}

// Configure RCC
void RCC_Configuration(void){
   
  // APB1 Prescaler /2 - 100: HCLK divided by 2
  //RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1);
  //RCC->CFGR |= RCC_CFGR_PPRE1_2;
  
  // APB2 Prescaler HCLK not divided
  //RCC->CFGR |= (RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE1_2);
  
  //RCC->CFGR = 0;                       // HSI, 8 MHz, 

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // enable Timer1
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;  // enable Timer2

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
}

// Configure System Clock
void SYSCLK_Configuration(void)
{
  RCC_DeInit();
  
  RCC_HSEConfig(RCC_HSE_ON);
  
  ErrorStatus HSEStartUpStatus;
  
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if (HSEStartUpStatus = SUCCESS)
  {
    
     /* Enable Prefetch Buffer */
    //FLASH->ACR |= FLASH_ACR_PRFTBE;
 
    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
 
    //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    //FLASH_SetLatency(FLASH_Latency_2);
    
    RCC_PLLCmd(DISABLE);
    
    RCC_HCLKConfig(RCC_SYSCLK_Div2);
    
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    RCC_PCLK1Config(RCC_HCLK_Div1);
    
    // DIV2
    RCC->CFGR |= RCC_CFGR_PLLXTPRE;  
    // 0111: PLL input clock x 9
    
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
    //RCC->CFGR &= ~(RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_2 | RCC_CFGR_PLLMULL_3 | RCC_CFGR_PLLMULL_3);
    //RCC->CFGR |= (RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2);
    
    
    RCC_PLLCmd(ENABLE);
    
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
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

   if(pidControl)
   {
	TIM3->CCR3 = 1553;
	TIM3->CCR4 = 1553;
	TIM4->CCR3 = 3038;
	TIM4->CCR4 = 3038;
   }
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
    updatePidHandler();
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

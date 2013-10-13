// HSE 72 MHz internal clock

#include "stm32f10x_conf.h"
#include "lab3.h"

#include "FreeRTOS.h"
#include "StackMacros.h"
#include "semphr.h"
#include "task.h"
#include "portmacro.h"

unsigned char pidControl = 1;
uint8_t count = 0;

// Global semaphore handler
xSemaphoreHandle semaphore;

MotorSpeeds* p_motorSpeedsPtr;

// Configure System Tick Timer
void SysTick_Configuration(void)
{

  // Clock source selection (AHB/8)
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;

  // Initialize to 2250000
  SysTick->LOAD = 2250000;

  // SysTick exception request enable
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT;
  
  // Counter enable
  SysTick->CTRL |= SysTick_CTRL_ENABLE;
  
}

// Configure Timer 3
void TIM3_Configuration(void)
{

  // Clear CR1
  TIM3->CR1 = 0x00;
  
  // set prescale value
  TIM3->PSC = (uint16_t)(36000000/24) - 1;
  TIM3->CR1 |= TIM_CR1_ARPE;    // Auto-reload preload enabled
  //TIM3->ARR = 15530;		// Auto reload value
  TIM3->ARR = 100;
  
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
  TIM3->CCR3 = 24;
  TIM3->CCR4 = 24;
  
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
  // NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_3);
  // NVIC->ISER[0] |= (1 << TIM3_IRQn); // enable TIM2 int in NVIC
}

void setMotors(void)
{
    TIM3->CCR3 = 10 * p_motorSpeedsPtr->m1;
    TIM3->CCR4 = 10 * p_motorSpeedsPtr->m2;
    TIM4->CCR3 = 10 * p_motorSpeedsPtr->m3;
    TIM4->CCR4 = 10 * p_motorSpeedsPtr->m4; 
}

// Configure Timer 4
void TIM4_Configuration(void)
{
  // clear CR1 register
  TIM4->CR1 = 0x00;
  // set prescale value
  TIM4->PSC = (uint16_t)(36000000/24) - 1;
  TIM4->CR1 |= TIM_CR1_ARPE;		// Auto-reload enable
  //TIM4->ARR = 30380;			// Set auto reload value
  TIM4->ARR = 2400;
  
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
  TIM4->CCR3 = 24;
  TIM4->CCR4 = 24;
  
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
  // NVIC_SetPriority (TIM4_IRQn, NVIC_IPR1_PRI_4);
  // NVIC->ISER[0] |= (1 << TIM4_IRQn); // enable TIM2 int in NVIC
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

void task_detectEmergency(void *pvParameters)
{
  // Initialize delay wake time 
  //portTickType wakeTime;
  //portTickType frequency = 10;
  //wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
      xSemaphoreGive(semaphore);
      detectEmergency();
      vTaskDelay(10);
  } 
}

void task_refreshSensorData(void *pvParameters)
{
    while(1){
    // Block until available
    refreshSensorData();
    vTaskDelay(100);
  }
}

void task_calculateOrientation(void *pvParameters)
{
  // Initialize delay wake time 
  //portTickType wakeTime;
  //portTickType frequency = 100;
  //wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
    // Block until available
    while (!(xSemaphoreTake(semaphore,0)==pdTRUE));
    calculateOrientation();
    vTaskDelay(100);
  }
}

void task_updatePid(void *pvParameters)
{
  // Initialize delay wake time 
  // portTickType wakeTime;
  // portTickType frequency = 1000;
  // wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
    updatePid(p_motorSpeedsPtr);
    setMotors();
    vTaskDelay(1000);
  }
}

void task_logDebugInfo(void *pvParameters)
{
  // Initialize delay wake time 
  // portTickType wakeTime;
  // portTickType frequency = 1000;
  // wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
    logDebugInfo();
    vTaskDelay(1000);
  }
}

void task_gLED(void *pvParameters)
{
  // Initialize delay wake time 
  // portTickType wakeTime;
  // portTickType frequency = 500;
  // wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
    GPIOB->ODR ^= GPIO_ODR_ODR5;
    vTaskDelay(500);
  }
}

void task_rLED(void *pvParameters)
{
  // Initialize delay wake time 
  // portTickType wakeTime;
  // portTickType frequency = 250;
  // wakeTime = xTaskGetTickCount();
  
  // Execute & delay
  while(1){
    GPIOB->ODR ^= GPIO_ODR_ODR4;
    vTaskDelay(250);
  }
}

void task_Configuration(void)
{
 
  // Create semaphore
  vSemaphoreCreateBinary(semaphore);
  
  // Semaphore not created
  while(semaphore == NULL);
  
  // Task Creations
  xTaskCreate(task_detectEmergency, NULL, configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 5), NULL);
  xTaskCreate(task_refreshSensorData, NULL, configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 4), NULL);
  xTaskCreate(task_calculateOrientation, NULL, configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3), NULL);
  xTaskCreate(task_updatePid, NULL, configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2), NULL);
  xTaskCreate(task_logDebugInfo, NULL, configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1), NULL);
  xTaskCreate(task_rLED, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(task_gLED, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  
}


void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pxTask;
    ( void ) pcTaskName;
    while(1);
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
  // SysTick_Configuration();
  // set SysTick priority
  NVIC_SetPriority (SysTick_IRQn, NVIC_IPR0_PRI_1);
  //NVIC->ISER[0] |= (1 << SysTick_IRQn); // enable SysTick int in NVIC

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
  TIM3_Configuration();
  TIM4_Configuration();
  
  // FreeRTOS initialization
  task_Configuration();
  
  // start scheduler
  vTaskStartScheduler();
  
  return 0;
}

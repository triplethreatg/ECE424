/********************************************
 * 
 * 	Lab 3 - FreeRTOS
 * 	Authors: Gregory Simpson & Justin Ng
 * 	10/16/2013
 * 
 ********************************************/

#include "stm32f10x.h"
#include "lab3.h"

#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "portmacro.h"

#define PCLK1		72000000

// Priorities used for FreeRTOS tasks
#define PRIORITY	tskIDLE_PRIORITY
#define PRIORITY1	(tskIDLE_PRIORITY+1)
#define PRIORITY2	(tskIDLE_PRIORITY+2)
#define PRIORITY3	(tskIDLE_PRIORITY+3)
#define PRIORITY4	(tskIDLE_PRIORITY+4)
#define PRIORITY5	(tskIDLE_PRIORITY+5)


unsigned char pidControl = 1;
uint8_t count = 0;

// Structures to initialize the motor timers
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

// Global semaphore handler
xSemaphoreHandle semaphore;

MotorSpeeds p_motorSpeedsPtr;

/* Keep linker happy. */
void assert_failed( unsigned portCHAR* pcFile, unsigned portLONG ulLine )
{
	for( ;; )
	{
	}
}

// Configure Timer 3
void TIM3_Configuration(void)
{

  // Clear CR1
  TIM3->CR1 = 0x00;
  
  // set prescale value
  TIM3->PSC = 0;
  TIM3->CR1 |= TIM_CR1_ARPE;    // Auto-reload preload enabled
  TIM3->ARR = 100;		// set auto-reload value
  
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
}

// Configure Timer 4
void TIM4_Configuration(void)
{
  // clear CR1 register
  TIM4->CR1 = 0x00;
  // set prescale value
  TIM4->PSC = 0;
  TIM4->CR1 |= TIM_CR1_ARPE;		// Auto-reload enable
  // Set auto reload value
  TIM4->ARR = 100;
  
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
}

// Configure RCC
void RCC_Configuration(void)
{

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
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
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    
    // Divide PCLK2 by 2
    RCC_PCLK2Config(RCC_HCLK_Div2);
    
    // Divide PCLK1 by 1
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

void vDetectEmergencyTask(void *p)
{

  while(1){
    detectEmergency();
    // Delay 10 ticks
    vTaskDelay(10);
  }
}

void vRefreshSensorDataTask(void *p)
{
  while(1){
    // Take the Semaphore
    xSemaphoreTake(semaphore, 0);
    refreshSensorData();
    // Release the semaphore
    xSemaphoreGive(semaphore);
    // Delay 100 ticks
    vTaskDelay(100);
  }
}

void vCalculateOrientationTask(void *p)
{

  // Execute & delay
  while(1){
    // Block until semaphore available
    if(xSemaphoreTake(semaphore,0)==pdTRUE)
      calculateOrientation();
    // Delay 1000 ticks
    vTaskDelay(1000);
  }
}

void vUpdatePidTask(void *p)
{
  
  // Execute & delay
  while(1){
    // updatePID
    updatePid(&p_motorSpeedsPtr);
    // Set motor speeds
    TIM3->CCR3 = (uint16_t)(20 * p_motorSpeedsPtr.m1);
    TIM3->CCR4 = (uint16_t)(20 * p_motorSpeedsPtr.m2);
    TIM4->CCR3 = (uint16_t)(20 * p_motorSpeedsPtr.m3);
    TIM4->CCR4 = (uint16_t)(20 * p_motorSpeedsPtr.m4);
    // Delay 1000 ticks
    vTaskDelay(1000);
  }
}

// Task for logDebugInfo method
void vLogDebugInfoTask(void *p)
{ 
  // Execute & delay
  while(1){
    // Debug
    logDebugInfo();
    // Delay 1000 ticks
    vTaskDelay(1000);
  }
}

// Task to toggle Green LED
void vGreenLEDTask(void *p)
{
  
  // Execute & delay
  while(1){
    // toggle Green LED
    GPIOB->ODR ^= GPIO_ODR_ODR5;
    // delay task by 500 ticks
    vTaskDelay(500);
  }
}

// Task to toggle Red LED
void vRedLEDTask(void *p)
{
  
  // Execute & delay
  while(1){
    // toggle Red LED
    GPIOB->ODR ^= GPIO_ODR_ODR4;
    // delay task by 250 ticks
    vTaskDelay(250);
  }
}

void task_Configuration(void)
{
  
  // Task Creations
  xTaskCreate(vDetectEmergencyTask, (signed char *) "Emergency", configMINIMAL_STACK_SIZE, NULL, PRIORITY5, NULL);
  xTaskCreate(vRefreshSensorDataTask, (signed char *) "Refresh", configMINIMAL_STACK_SIZE, NULL, PRIORITY4, NULL);
  xTaskCreate(vCalculateOrientationTask, (signed char *) "Orientation", configMINIMAL_STACK_SIZE, NULL, PRIORITY3, NULL);
  xTaskCreate(vUpdatePidTask, (signed char *) "Update", configMINIMAL_STACK_SIZE, NULL, PRIORITY2, NULL);
  xTaskCreate(vLogDebugInfoTask, (signed char *) "Log", configMINIMAL_STACK_SIZE, NULL, PRIORITY1, NULL);
  xTaskCreate(vRedLEDTask, (signed char *) "REDLED", configMINIMAL_STACK_SIZE, NULL, PRIORITY, NULL);
  xTaskCreate(vGreenLEDTask, (signed char *) "GREENLED" , configMINIMAL_STACK_SIZE, NULL, PRIORITY, NULL);
  
  // Create Semaphore
  vSemaphoreCreateBinary(semaphore);
  
}

// needed for FreeRTOS tasks
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
  
  // In case task start scheduler fails
  while(1);
  
  return 0;
}

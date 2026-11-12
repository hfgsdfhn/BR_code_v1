/* USER CODE BEGIN Header */
/* CubeMX RTOS entry point with the user task setup kept in this file. */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Status_Task */
osThreadId_t Status_TaskHandle;
const osThreadAttr_t Status_Task_attributes = {
  .name = "Status_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SuctionCup_Task */
osThreadId_t SuctionCup_TaskHandle;
const osThreadAttr_t SuctionCup_Task_attributes = {
  .name = "SuctionCup_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Arm_Task */
osThreadId_t Arm_TaskHandle;
const osThreadAttr_t Arm_Task_attributes = {
  .name = "Arm_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

extern void Status_Task(void *argument);
extern void SuctionCup_Task(void *argument);
extern void Arm_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
   (void)xTask;
   (void)pcTaskName;

   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  My_RtosInit();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Status_Task */
  Status_TaskHandle = osThreadNew(Status_Task, NULL, &Status_Task_attributes);

  /* creation of SuctionCup_Task */
  SuctionCup_TaskHandle = osThreadNew(SuctionCup_Task, NULL, &SuctionCup_Task_attributes);

  /* creation of Arm_Task */
  Arm_TaskHandle = osThreadNew(Arm_Task, NULL, &Arm_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* All application threads are created above. */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Status_Task */
/**
  * @brief  Function implementing the Status_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Status_Task */
__weak void Status_Task(void *argument)
{
  /* USER CODE BEGIN Status_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Status_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId TestTaskHandle;
osThreadId receive_2g4Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void testTask(void const * argument);
void receive_2g4_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TestTask */
  osThreadDef(TestTask, testTask, osPriorityNormal, 0, 128);
  TestTaskHandle = osThreadCreate(osThread(TestTask), NULL);

  /* definition and creation of receive_2g4 */
  osThreadDef(receive_2g4, receive_2g4_task, osPriorityLow, 0, 128);
  receive_2g4Handle = osThreadCreate(osThread(receive_2g4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_testTask */
/**
  * @brief  Function implementing the TestTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_testTask */
void testTask(void const * argument)
{
  /* USER CODE BEGIN testTask */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   //LED闪烁
      osDelay(100);
  }
  /* USER CODE END testTask */
}

/* USER CODE BEGIN Header_receive_2g4_task */
/**
* @brief Function implementing the receive_2g4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receive_2g4_task */
void receive_2g4_task(void const * argument)
{
  /* USER CODE BEGIN receive_2g4_task */
  uint8_t ret = 0;
  uint16_t sum = 0;
    RF2G4_RX_Mode();	// 接收模式

  /* Infinite loop */
  for(;;)
  {
      ret = RF2G4_Rx_Packet((u8 *)RF2G4_Receive_Data, 14);  //第一位是0或1就是按键，其他则是功能码
      if(sum >= 3)  //返回频率 每3次接收返回1次
      {
          sum = 0;
          RF2G4_TX_Mode_X();	// 发射模式
          RF2G4_Tx_Packet((u8 *)RF2G4_Send_Data,14);		//返回车子当前状态
          RF2G4_RX_Mode_X();	// 接收模式
      }
      else if (ret == 0) //收到了数据
      {
          if(RF2G4_Receive_Data[0] == 251 ||    //接收出错
                RF2G4_Receive_Data[1] == 251 ||
                RF2G4_Receive_Data[2] == 251 ||
                RF2G4_Receive_Data[3] == 251 ||
                RF2G4_Receive_Data[4] == 251 ||
                RF2G4_Receive_Data[5] == 251 ||
                RF2G4_Receive_Data[6] == 251 ||
                RF2G4_Receive_Data[7] == 251 ||
                RF2G4_Receive_Data[8] == 251 ||
                RF2G4_Receive_Data[9] == 251 ||
                RF2G4_Receive_Data[10] == 251 ||
                RF2G4_Receive_Data[11] == 251 ||
                RF2G4_Receive_Data[12] == 251 ||
                RF2G4_Receive_Data[13] == 251 )
          {
              //osDelay(10);
              //continue;
          }
          else if(RF2G4_Receive_Data[0] == 0xAA) //请求连接
          {
              RF2G4_TX_Mode_X();	// 发射模式
              RF2G4_Tx_Packet((u8 *)RF2G4_Receive_Data,14);		//返回握手包
              RF2G4_RX_Mode_X();	// 接收模式
              sum = 0;
          }
          else if(RF2G4_Receive_Data[0] == 0 || RF2G4_Receive_Data[0] == 1) //按键
          {


          }
          sum++;

          //打印所有接收到的信息
          RTT_printf(0, "%d: ",sum);
          for(u32 i = 0; i <14 ; i++)
            RTT_printf(0, "%d,", RF2G4_Receive_Data[i]);
          RTT_printf(0, "\r\n");
      }

      RF2G4_RX_Mode_X();	// 接收模式
      osDelay(10);
  }
  /* USER CODE END receive_2g4_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

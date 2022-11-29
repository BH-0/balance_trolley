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
#include "usart.h"
#include <string.h>
#include <stdio.h>
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
osThreadId pid_processHandle;
osThreadId receive_JY61Handle;
osThreadId receive_EncoderHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void testTask(void const * argument);
void receive_2g4_task(void const * argument);
void pid_process_task(void const * argument);
void receive_JY61_task(void const * argument);
void receive_Encoder_task(void const * argument);

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
  osThreadDef(TestTask, testTask, osPriorityIdle, 0, 128);
  TestTaskHandle = osThreadCreate(osThread(TestTask), NULL);

  /* definition and creation of receive_2g4 */
  osThreadDef(receive_2g4, receive_2g4_task, osPriorityLow, 0, 128);
  receive_2g4Handle = osThreadCreate(osThread(receive_2g4), NULL);

  /* definition and creation of pid_process */
  osThreadDef(pid_process, pid_process_task, osPriorityLow, 0, 128);
  pid_processHandle = osThreadCreate(osThread(pid_process), NULL);

  /* definition and creation of receive_JY61 */
  osThreadDef(receive_JY61, receive_JY61_task, osPriorityIdle, 0, 128);
  receive_JY61Handle = osThreadCreate(osThread(receive_JY61), NULL);

  /* definition and creation of receive_Encoder */
  osThreadDef(receive_Encoder, receive_Encoder_task, osPriorityIdle, 0, 128);
  receive_EncoderHandle = osThreadCreate(osThread(receive_Encoder), NULL);

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
              if(RF2G4_Receive_Data[8] == 1)    //左摇杆控按下制小车开关
              {
                  RF2G4_Send_Data[0] = !RF2G4_Send_Data[0];
              }


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

/* USER CODE BEGIN Header_pid_process_task */
/**
* @brief Function implementing the pid_process thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pid_process_task */
void pid_process_task(void const * argument)
{
  /* USER CODE BEGIN pid_process_task */
  /* Infinite loop */
    int16_t a_buf = 0 , b_buf = 0;
  for(;;)
  {
      if(JY61_res_bit == 1) //读取到数据
      {
          JY61_res_bit = 0;
          if(RF2G4_Send_Data[0] == 1)   //是否使能小车
          {
          a_buf = (RF2G4_Receive_Data[10] - 127)*50/127;    //摇杆直接控制车轮  （测试）
          b_buf = (RF2G4_Receive_Data[13] - 127)*50/127;
          sprintf((char *)TxBuffer2,"A%dB%d\r\n", a_buf, b_buf);

          HAL_UART_Transmit(&huart2, TxBuffer2, strlen((char *) TxBuffer2), 10);


          }else
          {
              a_buf = b_buf = 0;    //失能电机
              sprintf((char *)TxBuffer2,"A%dB%d\r\n", a_buf, b_buf);
              HAL_UART_Transmit(&huart2, TxBuffer2, strlen((char *) TxBuffer2), 10);
          }
      }
    osDelay(5);
  }
  /* USER CODE END pid_process_task */
}

/* USER CODE BEGIN Header_receive_JY61_task */
/**
* @brief Function implementing the receive_JY61 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receive_JY61_task */
void receive_JY61_task(void const * argument)
{
  /* USER CODE BEGIN receive_JY61_task */
  /* Infinite loop */
  for(;;)
  {
      if(__HAL_UART_GET_FLAG (&huart1, UART_FLAG_IDLE) != RESET)    //串口空闲标志
      {
          __HAL_UART_CLEAR_IDLEFLAG(&huart1);       //清除标志
          HAL_UART_DMAStop(&huart1);    //停止DMA接收
          RxLen = UART_RX_BUF_SIZE - huart1.hdmarx->Instance->NDTR;  //获取DMA接收长度
          if(RxLen > 0)  //判断接收非空
          {
              for(int i=0;i<RxLen;i++)
              {
                  //RTT_printf(2, "%d,", RxBuffer[i]);
                  if(RxBuffer[i] == 0x55 && RxBuffer[i+1] == 0x53 && (RxLen-i)>= 11)
                  {
                      //欧拉角数据处理
                      memcpy(&stcAngle,&RxBuffer[i+2],8);

                      RF2G4_Send_Data[1] = (stcAngle.Angle[1]>>8)&0xFF;
                      RF2G4_Send_Data[2] = stcAngle.Angle[1]&0xFF;
                      RF2G4_Send_Data[3] = (stcAngle.Angle[0]>>8)&0xFF;
                      RF2G4_Send_Data[4] = stcAngle.Angle[0]&0xFF;
                      RF2G4_Send_Data[5] = (stcAngle.Angle[2]>>8)&0xFF;
                      RF2G4_Send_Data[6] = stcAngle.Angle[2]&0xFF;

                      pitch = (float)stcAngle.Angle[1]/32768*180;
                      roll = (float)stcAngle.Angle[0]/32768*180;
                      yaw = (float)stcAngle.Angle[2]/32768*180;
                      //输出角度
                      RTT_printf(3,"pitch:%.3f roll:%.3f yaw:%.3f\r\n",pitch,roll,yaw);
                      JY61_res_bit = 1;
                  }
              }
              //RTT_printf(2,"\r\n");

          }
          StartUartRxDMA(&huart1);  //开启DMA

          //          HAL_UART_Receive_IT(huart, &ucData, 1);	// 由于接收1次数据，中断使能会被清除，所以这行代码必须添这句
//          ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
//          if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
//          {
//              ucRxCnt=0;
//              return;
//          }
//          if (ucRxCnt<11) {return;}//数据不满11个，则返回
//          else
//          {
//              switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
//              {
//                  //memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
//                  case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
//                  case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
//                  case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);JY61_res_bit = 1;break;
//              }
//              ucRxCnt=0;//清空缓存区
//          }
      }
    osDelay(5);
  }
  /* USER CODE END receive_JY61_task */
}

/* USER CODE BEGIN Header_receive_Encoder_task */
/**
* @brief Function implementing the receive_Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receive_Encoder_task */
void receive_Encoder_task(void const * argument)
{
  /* USER CODE BEGIN receive_Encoder_task */
  /* Infinite loop */
  for(;;)
  {
      if(__HAL_UART_GET_FLAG (&huart2, UART_FLAG_IDLE) != RESET)    //串口空闲标志
      {
          __HAL_UART_CLEAR_IDLEFLAG(&huart2);       //清除标志
          HAL_UART_DMAStop(&huart2);    //停止DMA接收
          RxLen2 = UART_RX_BUF_SIZE - huart2.hdmarx->Instance->NDTR;  //获取DMA接收长度
          if(RxLen2 > 0)  //判断接收非空
          {
              for(int i = 0; i <RxLen2; i++)
              {
                  if(RxBuffer2[i] == 'L')
                  {
                      Encoder_Left = atof((const char *)&RxBuffer2[i+1]);
                  }
                  if(RxBuffer2[i] == 'R')
                  {
                      Encoder_Right = atof((const char *)&RxBuffer2[i+1]);
                  }
              }
              RTT_printf(4,"%f,%f\r\n",Encoder_Left, Encoder_Right);

          }
          StartUartRxDMA(&huart2);  //开启DMA

      }
    osDelay(10);
  }
  /* USER CODE END receive_Encoder_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

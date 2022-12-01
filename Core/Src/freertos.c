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
#include "pid.h"
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
  osThreadDef(pid_process, pid_process_task, osPriorityBelowNormal, 0, 128);
  pid_processHandle = osThreadCreate(osThread(pid_process), NULL);

  /* definition and creation of receive_JY61 */
  osThreadDef(receive_JY61, receive_JY61_task, osPriorityLow, 0, 128);
  receive_JY61Handle = osThreadCreate(osThread(receive_JY61), NULL);

  /* definition and creation of receive_Encoder */
  osThreadDef(receive_Encoder, receive_Encoder_task, osPriorityLow, 0, 128);
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
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   //LED��˸
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
  uint16_t sum = 0; //���մ�������
  uint16_t count = 0;   //��������
  uint16_t i = 0;
    RF2G4_RX_Mode();	// ����ģʽ

  /* Infinite loop */
  for(;;)
  {
      ret = RF2G4_Rx_Packet((u8 *)RF2G4_Receive_Data, 14);  //��һλ��0��1���ǰ������������ǹ�����
      if(sum >= 3)  //����Ƶ�� ÿ3�ν��շ���1��
      {
          sum = 0;
          RF2G4_TX_Mode_X();	// ����ģʽ
          RF2G4_Tx_Packet((u8 *)RF2G4_Send_Data,14);		//���س��ӵ�ǰ״̬
          RF2G4_RX_Mode_X();	// ����ģʽ
      }
      else if (ret == 0) //�յ�������
      {
          if(RF2G4_Receive_Data[0] == 251 ||    //���ճ���
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
          else if(RF2G4_Receive_Data[0] == 0xAA) //��������
          {
              RF2G4_TX_Mode_X();	// ����ģʽ
              RF2G4_Tx_Packet((u8 *)RF2G4_Receive_Data,14);		//�������ְ�
              RF2G4_RX_Mode_X();	// ����ģʽ
              sum = 0;
          }
          else if((RF2G4_Receive_Data[0] == 0 || RF2G4_Receive_Data[0] == 1)) //��������
          {
              if(RF2G4_Receive_Data[8] == 1 && foc_ready == 1 && count> 20)    //��ҡ�˿ذ�����С������
              {
                  count = 0;
                  RF2G4_Send_Data[0] = !RF2G4_Send_Data[0];
              }

              if(RF2G4_Receive_Data[5] == 1 && count> 10)    //A+
              {
                  count = 0;
                  pid.Kp += 0.01f;
                  RF2G4_Send_Data[5] = pid.Kp*100;
              }
              else if(RF2G4_Receive_Data[4] == 1 && count> 10)    //A
              {
                  count = 0;
                  pid.Kp -= 0.01f;
                  RF2G4_Send_Data[5] = pid.Kp*100;
              }

              if(RF2G4_Receive_Data[7] == 1 && count> 10)    //B+
              {
                  count = 0;
                  pid.Kd += 0.001f;
                  RF2G4_Send_Data[6] = pid.Kd*1000;
              }
              else if(RF2G4_Receive_Data[6] == 1 && count> 10)    //B
              {
                  count = 0;
                  pid.Kd -= 0.001f;
                  RF2G4_Send_Data[6] = pid.Kd*1000;
              }

              if(RF2G4_Receive_Data[0] == 1 && count> 10)    //UP
              {
                  count = 0;
                  pid.Kp_speed += 1;
                  pid.Ki_speed = pid.Kp_speed/200;
                  RF2G4_Send_Data[7] = pid.Kp_speed;
              }
              else if(RF2G4_Receive_Data[1] == 1 && count> 10)    //DOWN
              {
                  count = 0;
                  pid.Kp_speed -= 1;
                  pid.Ki_speed = pid.Kp_speed/200;
                  RF2G4_Send_Data[7] = pid.Kp_speed;
              }

              if(RF2G4_Receive_Data[2] == 1 && count> 10)    //LEFT
              {
                  count = 0;
                  pid.Kd_turn += 0.01f;
                  RF2G4_Send_Data[9] = pid.Kd_turn*-100;
              }
              else if(RF2G4_Receive_Data[3] == 1 && count> 10)    //RIGHT
              {
                  count = 0;
                  pid.Kd_turn -= 0.01f;
                  RF2G4_Send_Data[9] = pid.Kd_turn*-100;
              }

              //�ٶȿ���
              if(RF2G4_Receive_Data[10]!=0 && (RF2G4_Receive_Data[10]<125 || RF2G4_Receive_Data[10]>145))
              {
                  pid.Set_Speed = (float)RF2G4_Receive_Data[10]*-0.5f/255+0.25f;
              }
              else
              {
                  pid.Set_Speed = 0;
              }

              //�������
              if(RF2G4_Receive_Data[12]!=0 && (RF2G4_Receive_Data[12]<120 || RF2G4_Receive_Data[12]>145))
              {
//                  pid.Angle_turn += (float)(RF2G4_Receive_Data[12]-127)/10;    //��Сϵ��
//                  if(pid.Angle_turn >= 180)
//                      pid.Angle_turn = -179.80f;
//                  else if(pid.Angle_turn <= -180)
//                      pid.Angle_turn = 179.80f;
                  //RTT_printf(1,":%f\r\n",pid.Angle_turn);
                  if(RF2G4_Receive_Data[12] <127)
                  {
                      pid.Flag_Left = 0;
                      pid.Flag_Right = 1;
                      pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[12]-127)/-30;
                  }
                  else
                  {
                      pid.Flag_Left = 1;
                      pid.Flag_Right = 0;
                      pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[12]-127)/30;
                  }
              } //С�������
              else if(RF2G4_Receive_Data[11]!=0 && (RF2G4_Receive_Data[11]<110 || RF2G4_Receive_Data[11]>145) &&
                      (RF2G4_Receive_Data[10]<110 || RF2G4_Receive_Data[10]>150))
              {
//                  pid.Angle_turn += (float)(RF2G4_Receive_Data[11]-127)/20;    //��Сϵ��
//                  if(pid.Angle_turn >= 180)
//                      pid.Angle_turn = -179.80f;
//                  else if(pid.Angle_turn <= -180)
//                      pid.Angle_turn = 179.80f;
                  //RTT_printf(1,":%f\r\n",pid.Angle_turn);
                  if(RF2G4_Receive_Data[11] <127)
                  {
                      pid.Flag_Left = 0;
                      pid.Flag_Right = 1;
                      pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[11]-127)/-50;
                  }
                  else
                  {
                      pid.Flag_Left = 1;
                      pid.Flag_Right = 0;
                      pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[11]-127)/50;
                  }
              }
              else
              {
                  pid.Flag_Left = pid.Flag_Right = 0;
              }

          }
          sum++;

          //��ӡ���н��յ�����Ϣ
          RTT_printf(0, "%d: ",sum);
          for(i = 0; i <14 ; i++)
            RTT_printf(0, "%d,", RF2G4_Receive_Data[i]);
          RTT_printf(0, "\r\n");
      }
      count++;
      RF2G4_RX_Mode_X();	// ����ģʽ
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
    int Balance_f = 0, Velocity_f = 0, Turn_f = 0;
    int Moto_Left, Moto_Right;  //���������������ٷֱ� ���100����С-100
    PID_Init();	//��ʼ��pidֵ
    for(;;)
    {
        if(JY61_res_bit == 1) //��ȡ������
        {
            JY61_res_bit = 0;
            if(RF2G4_Send_Data[0] == 1)   //�Ƿ�ʹ��С��
            {
                //          Moto_Left = (RF2G4_Receive_Data[10] - 127)*50/127;    //ҡ��ֱ�ӿ��Ƴ���  �����ԣ�
                //          Moto_Right = (RF2G4_Receive_Data[13] - 127)*50/127;
                pid.Pv = pitch; //�Ƕ�*10��
                Balance_f = balance(pid.Pv); //ƽ�⻷
                Velocity_f = velocity(Encoder_Left, Encoder_Right); //�ٶȻ�
                Turn_f = turn_x(Encoder_Left, Encoder_Right, yaw);    //ת��

                Moto_Left = Balance_f + Velocity_f - Turn_f;    //�ϳ����
                Moto_Right = Balance_f + Velocity_f + Turn_f;    //�ϳ����
                Xianfu_Pwm(&Moto_Left, &Moto_Right);    //�޷�
                sprintf((char *)TxBuffer2,"A%dB%d\r\n", Moto_Left, Moto_Right);
                HAL_UART_Transmit(&huart2, TxBuffer2, strlen((char *) TxBuffer2), 10);  //���
            }else
            {
                Moto_Left = Moto_Right = 0;    //ʧ�ܵ��
                pid.EK_speed = pid.SEK_speed = 0;
                pid.Angle_turn = yaw;   //ֹͣʱȡ��Ĭ�ϳ���
                sprintf((char *)TxBuffer2,"A%dB%d\r\n", Moto_Left, Moto_Right);
                HAL_UART_Transmit(&huart2, TxBuffer2, strlen((char *) TxBuffer2), 10);
            }
        }
    osDelay(3);    //10Ч���Ϻ�
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
      if(__HAL_UART_GET_FLAG (&huart1, UART_FLAG_IDLE) != RESET)    //���ڿ��б�־
      {
          __HAL_UART_CLEAR_IDLEFLAG(&huart1);       //�����־
          HAL_UART_DMAStop(&huart1);    //ֹͣDMA����
          RxLen = UART_RX_BUF_SIZE - huart1.hdmarx->Instance->NDTR;  //��ȡDMA���ճ���
          if(RxLen > 0)  //�жϽ��շǿ�
          {
              for(int i=0;i<RxLen;i++)
              {
                  //RTT_printf(2, "%d,", RxBuffer[i]);
                  if(RxBuffer[i] == 0x55 && RxBuffer[i+1] == 0x52)
                  {
                      //�����ǽ��ٶ����ݴ���
                      memcpy(&stcGyro,&RxBuffer[i+2],8);

                      gyrox = (float)stcGyro.w[0]/32768*2000;
                      gyroy = (float)stcGyro.w[1]/32768*2000;
                      gyroz = (float)stcGyro.w[2]/32768*2000;
                      //����������
                      RTT_printf(5,":%.3f :%.3f :%.3f\r\n",gyrox,gyroy,gyroz);
                  }
                  if(RxBuffer[i] == 0x55 && RxBuffer[i+1] == 0x53 && (RxLen-i)>= 11)
                  {
                      //ŷ�������ݴ���
                      memcpy(&stcAngle,&RxBuffer[i+2],8);

                      RF2G4_Send_Data[1] = (stcAngle.Angle[1]>>8)&0xFF;
                      RF2G4_Send_Data[2] = stcAngle.Angle[1]&0xFF;
                      RF2G4_Send_Data[3] = (stcAngle.Angle[2]>>8)&0xFF;
                      RF2G4_Send_Data[4] = stcAngle.Angle[2]&0xFF;

                      pitch = (float)stcAngle.Angle[1]/32768*180;
                      roll = (float)stcAngle.Angle[0]/32768*180;
                      yaw = (float)stcAngle.Angle[2]/32768*180;

                      if(pitch>60 || pitch<-60 ||
                              roll>60 || roll<-60)
                          RF2G4_Send_Data[0] = 0;   //����ʧ��

                      //����Ƕ�
                      RTT_printf(3,"pitch:%.3f roll:%.3f yaw:%.3f\r\n",pitch,roll,yaw);
                      JY61_res_bit = 1;
                  }
              }
              //RTT_printf(2,"\r\n");

          }
          StartUartRxDMA(&huart1);  //����DMA
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
      if(__HAL_UART_GET_FLAG (&huart2, UART_FLAG_IDLE) != RESET)    //���ڿ��б�־
      {
          __HAL_UART_CLEAR_IDLEFLAG(&huart2);       //�����־
          HAL_UART_DMAStop(&huart2);    //ֹͣDMA����
          RxLen2 = UART_RX_BUF_SIZE - huart2.hdmarx->Instance->NDTR;  //��ȡDMA���ճ���
          if(RxLen2 > 0)  //�жϽ��շǿ�
          {
              for(int i = 0; i <RxLen2; i++)
              {
                  if(RxBuffer2[i] == 'L')
                  {
                      Encoder_Left = atof((const char *)&RxBuffer2[i+1]);
                      foc_ready = 1;
                  }
                  if(RxBuffer2[i] == 'R')
                  {
                      Encoder_Right = atof((const char *)&RxBuffer2[i+1]);
                      foc_ready = 1;
                  }
                  if(Encoder_Left>15 || Encoder_Left<-15 || Encoder_Right>15 || Encoder_Right<-15)
                      RF2G4_Send_Data[0] = 0;   //����ʧ��
              }
              RTT_printf(4,"%f,%f\r\n",Encoder_Left, Encoder_Right);

          }
          StartUartRxDMA(&huart2);  //����DMA

      }
    osDelay(10);
  }
  /* USER CODE END receive_Encoder_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

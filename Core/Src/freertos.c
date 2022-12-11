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
#include "Adafruit_NeoPixel.h"
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
uint16_t Tof_Value = 0; //����
/* USER CODE END Variables */
osThreadId LED_RGBHandle;
osThreadId receive_2g4Handle;
osThreadId pid_processHandle;
osThreadId receive_JY61Handle;
osThreadId receive_EncoderHandle;
osThreadId receive_vl53l0xHandle;
osThreadId testTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint16_t VL53L0X_GetValue(void);
/* USER CODE END FunctionPrototypes */

void LED_RGB_task(void const * argument);
void receive_2g4_task(void const * argument);
void pid_process_task(void const * argument);
void receive_JY61_task(void const * argument);
void receive_Encoder_task(void const * argument);
void receive_vl53l0x_task(void const * argument);
void test_Task(void const * argument);

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
  /* definition and creation of LED_RGB */
  osThreadDef(LED_RGB, LED_RGB_task, osPriorityNormal, 0, 128);
  LED_RGBHandle = osThreadCreate(osThread(LED_RGB), NULL);

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

  /* definition and creation of receive_vl53l0x */
  osThreadDef(receive_vl53l0x, receive_vl53l0x_task, osPriorityLow, 0, 128);
  receive_vl53l0xHandle = osThreadCreate(osThread(receive_vl53l0x), NULL);

  /* definition and creation of testTask */
  osThreadDef(testTask, test_Task, osPriorityIdle, 0, 128);
  testTaskHandle = osThreadCreate(osThread(testTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LED_RGB_task */
/**
  * @brief  Function implementing the LED_RGB thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LED_RGB_task */
void LED_RGB_task(void const * argument)
{
  /* USER CODE BEGIN LED_RGB_task */
  /* Infinite loop */
  uint16_t sum = 0;
  uint16_t Brightness = 0;
  WS2812B_Init();

    //�ȴ���ʼ��
    while (foc_ready == 0)
    {
        if((RGB_mode_bit&0x80) != 0 && RGB_cmd_bit != 0)
          double_rotating(SKY, YELLOW, 100);
    }

    //��ʼ����ɣ���˸
    if((RGB_mode_bit&0x80) != 0 && RGB_cmd_bit != 0)
    {
        colorWipe(0x44ff00,1);
        osDelay(100);
        colorWipe(0,1);
        osDelay(100);
        colorWipe(0x44ff00,1);
        osDelay(100);
        colorWipe(0,1);
        osDelay(500);
    }

  for(;;)
  {
      if((RGB_mode_bit&0x80)!= 0 && RGB_cmd_bit != 0)
      {
          if(RF2G4_Send_Data[13] == 0)  //����
          {
              if(pid.brake == 1) //ɲ��״̬
              {
                if(Brightness+1 >= index_wave_num)
                    Brightness = 0;
                else
                    Brightness += 1;
                  rainbow((float)index_wave[Brightness]/255, 2);    //�����ʺ�
              }
              else
                  rainbowCycle(2);  //��ת�ʺ�
          }
          else if(RF2G4_Send_Data[13] == 1) //����
          {
              if(Tof_Value>80 && Tof_Value <= 400)    //��Ӧ��Χ
              {
                //240-400
                    float Red_Value = 0;
                    float Green_Value = 0;
                    if(Tof_Value>240)
                        Red_Value = (float)(400-Tof_Value)/160;
                    else
                        Red_Value = 1;
                  if(Tof_Value>240)
                      Green_Value = (float)(Tof_Value-240)/160;
                  else
                      Green_Value = 0;

                  colorWipe(Color(0xFF*Red_Value, 0xFF*Green_Value, 0),1);  //Խ��Խ��
              }
              else if(Tof_Value>400 && Tof_Value!= 0xFFFF)
              {
                  colorWipe(0x00ff00,1);    //������Ӧ��Χȫ��
              }
          }
          else if(RF2G4_Send_Data[13] == 2) //����
          {
              if(Tof_Value>80 && Tof_Value <= 400)    //��Ӧ��Χ
              {
                  if(Tof_Value>240)
                  {
                      progress_bar(9*(float)(400-Tof_Value)/160,BLUE,PINK,2);   //˫�������
                  }
                  else if(Tof_Value<=240)
                  {
                      progress_bar(9*(float)(240-Tof_Value)/160+8,BLUE,PINK,2);
                  }
              }
              else  //Ѱ��Ŀ��
              {
                  SetPixelColor(0, 0);
                  SetPixelColor(1, 0);
                  SetPixelColor(2, Color(ORANGE.R, ORANGE.G, ORANGE.B));
                  SetPixelColor(3, Color(ORANGE.R, ORANGE.G, ORANGE.B));
                  SetPixelColor(4, 0);
                  SetPixelColor(5, 0);
                  SetPixelColor(6, 0);
                  SetPixelColor(7, 0);
                  osDelay(200);
                  SetPixelColor(0, 0);
                  SetPixelColor(1, 0);
                  SetPixelColor(2, 0);
                  SetPixelColor(3, 0);
                  SetPixelColor(4, 0);
                  SetPixelColor(5, 0);
                  SetPixelColor(6, Color(ORANGE.R, ORANGE.G, ORANGE.B));
                  SetPixelColor(7, Color(ORANGE.R, ORANGE.G, ORANGE.B));
                  osDelay(200);
              }
          }

      }
      else if(RGB_cmd_bit != 0)//ң�ص�
      {
            if(RGB_mode_bit == 1)    //Զ��
            {
                colorWipe(0xFFFFFF, 10);
            }
            else if(RGB_mode_bit == 2)   //����
            {
                double_rotating(RED, BLUE, 80);
            }
      }
      else
          colorWipe(0,10);
      //colorWipe(Color(255,255,0),2);

      //rainbowCycle(2);
      if(sum >= 11)
      {

          sum = 0;
      }
      sum++;

      osDelay(10);
  }
  /* USER CODE END LED_RGB_task */
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
  uint8_t RGB_mode_bit_last = 0;    //�ϴ�ģʽ
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
                  if(RF2G4_Send_Data[0] != 1)
                      RF2G4_Send_Data[0] = 1;
                  else if(RF2G4_Send_Data[0] != 0)
                      RF2G4_Send_Data[0] = 0;
              }
              if(RF2G4_Receive_Data[9] == 1 && foc_ready == 1 && count> 20)    //��ҡ�˿ذ���ģʽ�л�
              {
                  count = 0;
                  if(RF2G4_Send_Data[13] >= 2)
                      RF2G4_Send_Data[13] = 0;
                  else
                      RF2G4_Send_Data[13] ++;
              }

#define PID_SET 1
#if PID_SET
              if(RF2G4_Receive_Data[5] == 1 && count> 10)    //A+
              {
                  count = 0;
                  pid.Kp += 0.1f;
                  RF2G4_Send_Data[5] = pid.Kp*10;
              }
              else if(RF2G4_Receive_Data[4] == 1 && count> 10)    //A
              {
                  count = 0;
                  pid.Kp -= 0.1f;
                  RF2G4_Send_Data[5] = pid.Kp*10;
              }

              if(RF2G4_Receive_Data[7] == 1 && count> 10)    //B+
              {
                  count = 0;
                  pid.Kd += 0.01f;
                  RF2G4_Send_Data[6] = pid.Kd*100;
              }
              else if(RF2G4_Receive_Data[6] == 1 && count> 10)    //B
              {
                  count = 0;
                  pid.Kd -= 0.01f;
                  RF2G4_Send_Data[6] = pid.Kd*100;
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
#else
              if(RF2G4_Receive_Data[0] == 1 && count> 10)    //UP
              {
                  count = 0;
                  //RGB����
                  if(RGB_cmd_bit*2 >= 1)
                      RGB_cmd_bit = 1;
                  else if(RGB_cmd_bit == 0)
                      RGB_cmd_bit = 0.025f;
                  else
                      RGB_cmd_bit*=2;
              }
              else if(RF2G4_Receive_Data[1] == 1 && count> 10)    //DOWN
              {
                  count = 0;
                  //RGB����
                  if(RGB_cmd_bit/2 <= 0.02f)
                      RGB_cmd_bit = 0;
                  else
                      RGB_cmd_bit/=2;
              }

              if(RF2G4_Receive_Data[2] == 1 && count> 10)    //LEFT
              {
                  count = 0;

              }
              else if(RF2G4_Receive_Data[3] == 1 && count> 10)    //RIGHT
              {
                  count = 0;

              }

              if(RF2G4_Receive_Data[5] == 1 && count> 10)    //A+
              {
                  count = 0;
                  //Զ��
                    if((RGB_mode_bit&0x80)!= 0)
                    {
                        RGB_mode_bit_last = RGB_mode_bit;
                        RGB_mode_bit = 1;
                    }
                    else if(RGB_mode_bit == 1)
                    {
                        RGB_mode_bit = RGB_mode_bit_last;
                    }
                    else
                    {
                        RGB_mode_bit = 1;
                    }

              }
              else if(RF2G4_Receive_Data[4] == 1 && count> 10)    //A
              {
                  count = 0;
                  //����
                  if((RGB_mode_bit&0x80)!= 0)
                  {
                      RGB_mode_bit_last = RGB_mode_bit;
                      RGB_mode_bit = 2;
                  }
                  else if(RGB_mode_bit == 2)
                  {
                      RGB_mode_bit = RGB_mode_bit_last;
                  }
                  else
                  {
                      RGB_mode_bit = 2;
                  }
              }

              if(RF2G4_Receive_Data[7] == 1 && count> 10)    //B+
              {
                  count = 0;

              }
              else if(RF2G4_Receive_Data[6] == 1 && count> 10)    //B
              {
                  count = 0;

              }

#endif /*PID_SET*/

              //�ٶȿ���
              if(RF2G4_Receive_Data[10]!=0 && (RF2G4_Receive_Data[10]<125 || RF2G4_Receive_Data[10]>145))
              {
                  if(RF2G4_Send_Data[13] != 2)  //���ڸ���ģʽ��
                    pid.Set_Speed = (float)RF2G4_Receive_Data[10]*-0.5f/255+0.25f;
                  pid.brake = 0;//ɲ����־
              }
              else
              {
                  pid.Set_Speed = 0;
                  if(pid.brake == 0)
                      pid.SEK_speed *= 0.4f;    //�ۼ��ٶȲ������ɲ������ 0.6
                  pid.brake = 1;//ɲ����־
              }

              if(RF2G4_Send_Data[13] != 2)  //���ڸ���ģʽ��
              {
                  //�������
                  if(RF2G4_Receive_Data[12]!=0 && (RF2G4_Receive_Data[12]<120 || RF2G4_Receive_Data[12]>145))
                  {
                      pid.Angle_turn += (float)(RF2G4_Receive_Data[12]-127)/10;    //��Сϵ��
                      if(pid.Angle_turn >= 180)
                          pid.Angle_turn = -179.80f;
                      else if(pid.Angle_turn <= -180)
                          pid.Angle_turn = 179.80f;
                      //RTT_printf(1,":%f\r\n",pid.Angle_turn);
//                      if(RF2G4_Receive_Data[12] <127)
//                      {
//                          pid.Flag_Left = 0;
//                          pid.Flag_Right = 1;
//                          pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[12]-127)/-30;
//                      }
//                      else
//                      {
//                          pid.Flag_Left = 1;
//                          pid.Flag_Right = 0;
//                          pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[12]-127)/30;
//                      }
                  } //С�������
                  else if(RF2G4_Receive_Data[11]!=0 && (RF2G4_Receive_Data[11]<110 || RF2G4_Receive_Data[11]>145) &&
                          (RF2G4_Receive_Data[10]<110 || RF2G4_Receive_Data[10]>150))
                  {
                      pid.Angle_turn += (float)(RF2G4_Receive_Data[11]-127)/20;    //��Сϵ��
                      if(pid.Angle_turn >= 180)
                          pid.Angle_turn = -179.80f;
                      else if(pid.Angle_turn <= -180)
                          pid.Angle_turn = 179.80f;
                      //RTT_printf(1,":%f\r\n",pid.Angle_turn);
//                      if(RF2G4_Receive_Data[11] <127)
//                      {
//                          pid.Flag_Left = 0;
//                          pid.Flag_Right = 1;
//                          pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[11]-127)/-50;
//                      }
//                      else
//                      {
//                          pid.Flag_Left = 1;
//                          pid.Flag_Right = 0;
//                          pid.Turn_Amplitude = (float)(RF2G4_Receive_Data[11]-127)/50;
//                      }
                  }
                  else
                  {
                      pid.Flag_Left = pid.Flag_Right = 0;
                  }
              }
          }
          sum++;


#if 0   //��ӡ���н��յ�����Ϣ
          RTT_printf(0, "%d: ",sum);
          for(i = 0; i <14 ; i++)
            RTT_printf(0, "%d,", RF2G4_Receive_Data[i]);
          RTT_printf(0, "\r\n");
#endif
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
                //Turn_f = turn_x(Encoder_Left, Encoder_Right, yaw);    //ת��
                Turn_f = turn(yaw);    //ת��

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
  uint8_t sum = 0;
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
                  if(RxBuffer[i] == 0x55 && RxBuffer[i+1] == 0x51)
                  {
                      //�����Ǽ��ٶ����ݴ���
                      memcpy(&stcAcc,&RxBuffer[i+2],8);

                      aacx = (float)stcAcc.a[0]/32768*16;
                      aacy = (float)stcAcc.a[1]/32768*16;
                      aacz = (float)stcAcc.a[2]/32768*16;
                      //����������
                      //RTT_printf(3,"Acc:%.3f :%.3f :%.3f\r\n",aacx,aacy,aacz);
                  }
                  if(RxBuffer[i] == 0x55 && RxBuffer[i+1] == 0x52)
                  {
                      //�����ǽ��ٶ����ݴ���
                      memcpy(&stcGyro,&RxBuffer[i+2],8);

                      gyrox = (float)stcGyro.w[0]/32768*2000;
                      gyroy = (float)stcGyro.w[1]/32768*2000;
                      gyroz = (float)stcGyro.w[2]/32768*2000;
                      //����������
                      //RTT_printf(5,":%.3f :%.3f :%.3f\r\n",gyrox,gyroy,gyroz);
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

                      if((pitch>60 || pitch<-70 ||
                              roll>60 || roll<-60 )&&
                              RF2G4_Send_Data[0] == 1)
                      {
                          sum++;
                          if(sum>=3)    //��ֹ����
                          {
                              RF2G4_Send_Data[0] = 0x40;   //����ʧ��
                              sum = 0;
                          }
                      }
                      else
                      {
                          sum = 0;
                      }
                      //����Ƕ�
                      //RTT_printf(3,"pitch:%.3f roll:%.3f yaw:%.3f\r\n",pitch,roll,yaw);
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
  uint8_t sum = 0;
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
                  if((Encoder_Left>10 || Encoder_Left<-10 || Encoder_Right>10 || Encoder_Right<-10) && RF2G4_Send_Data[0] == 1)
                  {
                      sum++;
                      if(sum >= 3)
                      {
                          RF2G4_Send_Data[0] = 0x80;   //����ʧ��
                          sum = 0;
                      }
                  }
                  else
                  {
                      sum = 0;
                  }
              }
              //RTT_printf(4,"%f,%f\r\n",Encoder_Left, Encoder_Right);  //��ӡ������

          }
          StartUartRxDMA(&huart2);  //����DMA

      }
    osDelay(10);
  }
  /* USER CODE END receive_Encoder_task */
}

/* USER CODE BEGIN Header_receive_vl53l0x_task */
/**
* @brief Function implementing the receive_vl53l0x thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receive_vl53l0x_task */
void receive_vl53l0x_task(void const * argument)
{
  /* USER CODE BEGIN receive_vl53l0x_task */
  /* Infinite loop */
  for(;;)
  {
      uint16_t Value = 0;
      Value = VL53L0X_GetValue();

      if(Value!=20 && Value!=8190 && Value!= 8191)
      {
          Tof_Value = Value;
          RTT_printf(2,"d=%d\r\n",Tof_Value);
      }
      else
      {
          Tof_Value = 0xFFFF;
      }

      osDelay(10);
  }
  /* USER CODE END receive_vl53l0x_task */
}

/* USER CODE BEGIN Header_test_Task */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test_Task */
void test_Task(void const * argument)
{
  /* USER CODE BEGIN test_Task */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   //LED��˸
      osDelay(100);
  }
  /* USER CODE END test_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

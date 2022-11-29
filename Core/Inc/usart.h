/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "JY61.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define UART_RX_BUF_SIZE 1024  //缓存数据长度
#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif


extern uint8_t JY61_res_bit;   //读取到数据
extern struct SAcc 		stcAcc; //加速度传感器原始数据
extern struct SGyro 		stcGyro;    //陀螺仪原始数据
extern struct SAngle 	stcAngle;   //欧拉角
extern float pitch, roll, yaw;			 //欧拉角
extern float Encoder_Left, Encoder_Right; //左右编码器的转速，单位：弧度/100毫秒

extern unsigned char RxBuffer[UART_RX_BUF_SIZE];
extern unsigned char TxBuffer[UART_RX_BUF_SIZE];
extern char sendCompleteSign;  //发送完成标志
extern unsigned char TxLen; //发送长度
extern unsigned int RxLen;//接收每段数据的字节长度

extern unsigned char RxBuffer2[UART_RX_BUF_SIZE];
extern unsigned char TxBuffer2[UART_RX_BUF_SIZE];
extern char sendCompleteSign2;  //发送完成标志
extern unsigned char TxLen2; //发送长度
extern unsigned int RxLen2;//接收每段数据的字节长度

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
//用串口1给JY模块发送指令
void sendcmd(char cmd[]);


char StartUartTxDMA(UART_HandleTypeDef *huart,uint8_t Len);
char StartUartRxDMA(UART_HandleTypeDef *huart);
void ProcessData(UART_HandleTypeDef *huart);
void HAL_UART_IdleCallback(UART_HandleTypeDef *huart);

char UartTxData(UART_HandleTypeDef *huart, unsigned char *buf, const unsigned int len);//DMA发送
char UartRxData(UART_HandleTypeDef *huart, unsigned char *buf, const unsigned int len);//DMA接收
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


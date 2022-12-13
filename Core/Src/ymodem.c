#include "sys.h"
#include "usart.h"
#include "ymodem.h"
#include "w25qxx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//uint8_t RxTestBuf[10240] = {0};

extern osMutexId SPI1_MutexHandle;

/**
 * @bieaf CRC-16 校验
 *
 * @param addr 开始地址
 * @param num   长度
 * @param num   CRC
 * @return crc  返回CRC的值
 */
#define POLY        0x1021  
uint16_t crc16(uint8_t *addr, int32_t num, uint16_t crc)  
{  
    int32_t i;  
    for (; num > 0; num--)					/* Step through bytes in memory */  
    {  
        crc = crc ^ (*addr++ << 8);			/* Fetch byte from memory, XOR into CRC top byte*/  
        for (i = 0; i < 8; i++)				/* Prepare to rotate 8 bits */  
        {
            if (crc & 0x8000)				/* b15 is set... */  
                crc = (crc << 1) ^ POLY;  	/* rotate and XOR with polynomic */  
            else                          	/* b15 is clear... */  
                crc <<= 1;					/* just rotate */  
        }									/* Loop for 8 bits */  
        crc &= 0xFFFF;						/* Ensure CRC remains 16-bit value */  
    }										/* Loop until num=0 */  
    return(crc);							/* Return updated CRC */  
}



/* 设置升级的步骤 */
/*static */enum UPDATE_STATE update_state = TO_START;

void ymodem_set_state(enum UPDATE_STATE state)
{
	update_state = state;
}


/* 查询升级的步骤 */
uint8_t ymodem_get_state(void)
{
	return update_state;
}




/* 发送指令 */
void ymodem_send_cmd(uint8_t command)
{
	portDISABLE_INTERRUPTS();   //关中断
	HAL_UART_Transmit(&huart2, &command, 1, 10);  //输出
	portENABLE_INTERRUPTS();    //开中断
	
	//osDelay(10);
}

/* 标记升级完成 */
void update_set_down(void)
{
	uint32_t update_flag = 0xAAAAAAAA;				// 对应bootloader的启动步骤
	
	//flash_program((APPLICATION_2_ADDR + APPLICATION_2_SIZE - 4), &update_flag,1 );
	portDISABLE_INTERRUPTS();   //关中断
	W25QXX_Write((uint8_t *)&update_flag, 64/8*1024*1024 - 4, 4);
	portENABLE_INTERRUPTS();    //开中断
}

uint8_t _4k_buf[4096] = {0};
/**
 * @bieaf ymodem下载
 *
 * @param none
 * @return none
 */
void ymodem_download(void)
{
	uint16_t crc = 0;

static 
	uint8_t data_state = 0;
	static uint8_t data_state_sum = 0;
	static uint8_t block_state = 0;
	// if(ymodem_get_state()==TO_START)
	// {
	// 	ymodem_send_cmd(CCC);
		
	// 	delay_ms(1000);
	// }
	
	/* 串口1接收完一个数据包 */
	if(RxLen2 > 0)//g_usart1_rx_end)    	
	{
		
		/* 清空接收完成标志位、接收计数值 */
		// g_usart1_rx_end=0;
		// g_usart1_rx_cnt=0;
		
		switch(RxBuffer2[0])
		{
			case SOH://数据包开始
			{
					crc = 0;
				
					/* 计算crc16 */
					crc = crc16((uint8_t *)&RxBuffer2[3], BUF_SIZE, crc);
						
					if(crc != (RxBuffer2[BUF_SIZE+3]<<8|RxBuffer2[BUF_SIZE+4]))
					{
						RTT_printf(1,"CRC Error\r\n");
						return;
					}
						
					
					if((ymodem_get_state()==TO_START)&&(RxBuffer2[1] == 0x00)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// 开始
					{

						ymodem_set_state(TO_RECEIVE_DATA);
						
						/* 若ymodem_send_cmd执行在sector_erase之前，则导致串口数据丢包，因为擦除会关闭所有中断 */
						/* 擦除应用程序2的扇区 */
						//sector_erase(APPLICATION_2_SECTOR);		

						/*擦除2个块,共擦除128kB*/
						osMutexWait(SPI1_MutexHandle, portMAX_DELAY);   //等待互斥量
						W25QXX_Erase_Block(0);	
						W25QXX_Erase_Block(64*1024);	
						osMutexRelease(SPI1_MutexHandle); //释放互斥量 

						osDelay(10);			
						
						data_state = 0x01;						
						ymodem_send_cmd(ACK);
						ymodem_send_cmd(CCC);

	
					}
					else if((ymodem_get_state()==TO_RECEIVE_END)&&(RxBuffer2[1] == 0x00)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// 结束
					{
						update_set_down();						
						ymodem_set_state(TO_START);
						ymodem_send_cmd(ACK);
						
						/* 嘀一声示，表示下载完成 */
						//beep_on();delay_ms(80);beep_off();
						if(data_state_sum != 0)
						{
							data_state_sum = 0;
							//memcpy(RxTestBuf+block_state*4096, _4k_buf, 4096);	//4096写入块 测试
                            portDISABLE_INTERRUPTS();   //关中断
							W25QXX_Write(_4k_buf, block_state*4096, 4096);
                            portENABLE_INTERRUPTS();    //开中断
							block_state++;
						}
						
						/* 复位 */
						//NVIC_SystemReset();

                        //连续发送。测试
                        ymodem_set_state(TO_START);
					}					
					else if((ymodem_get_state()==TO_RECEIVE_DATA)&&(RxBuffer2[1] == data_state)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// 接收数据
					{

						/* 烧录程序 */
						//flash_program((APPLICATION_2_ADDR + (data_state-1) * 128), (uint32_t *)(&RxBuffer2[3]), 32);
						if(data_state_sum<31)
						{
							memcpy(_4k_buf+data_state_sum*BUF_SIZE, (uint32_t *)(&RxBuffer2[3]), BUF_SIZE); //复制至缓存
							data_state_sum++;
						}
						else
						{
							memcpy(_4k_buf+data_state_sum*BUF_SIZE, (uint32_t *)(&RxBuffer2[3]), BUF_SIZE); //复制至缓存
                            data_state_sum = 0;
							//memcpy(RxTestBuf+block_state*4096, _4k_buf, 4096);	//4096写入块 测试
                            portDISABLE_INTERRUPTS();   //关中断
							W25QXX_Write(_4k_buf, block_state*4096, 4096);
                            portENABLE_INTERRUPTS();    //开中断
							block_state++;
						}



						data_state++;
						ymodem_send_cmd(ACK);		
					}
			}break;
			
			case EOT://数据包传输结束
			{
				if(ymodem_get_state()==TO_RECEIVE_DATA)
				{
					ymodem_set_state(TO_RECEIVE_EOT2);					
					ymodem_send_cmd(NACK);
				}
				else if(ymodem_get_state()==TO_RECEIVE_EOT2)
				{					
					ymodem_set_state(TO_RECEIVE_END);					
					ymodem_send_cmd(ACK);
					ymodem_send_cmd(CCC);
				}

                
	
			}break;	
			
			default:break;
		}

	}
}




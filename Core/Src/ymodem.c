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
 * @bieaf CRC-16 У��
 *
 * @param addr ��ʼ��ַ
 * @param num   ����
 * @param num   CRC
 * @return crc  ����CRC��ֵ
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



/* ���������Ĳ��� */
/*static */enum UPDATE_STATE update_state = TO_START;

void ymodem_set_state(enum UPDATE_STATE state)
{
	update_state = state;
}


/* ��ѯ�����Ĳ��� */
uint8_t ymodem_get_state(void)
{
	return update_state;
}




/* ����ָ�� */
void ymodem_send_cmd(uint8_t command)
{
	portDISABLE_INTERRUPTS();   //���ж�
	HAL_UART_Transmit(&huart2, &command, 1, 10);  //���
	portENABLE_INTERRUPTS();    //���ж�
	
	//osDelay(10);
}

/* ���������� */
void update_set_down(void)
{
	uint32_t update_flag = 0xAAAAAAAA;				// ��Ӧbootloader����������
	
	//flash_program((APPLICATION_2_ADDR + APPLICATION_2_SIZE - 4), &update_flag,1 );
	portDISABLE_INTERRUPTS();   //���ж�
	W25QXX_Write((uint8_t *)&update_flag, 64/8*1024*1024 - 4, 4);
	portENABLE_INTERRUPTS();    //���ж�
}

uint8_t _4k_buf[4096] = {0};
/**
 * @bieaf ymodem����
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
	
	/* ����1������һ�����ݰ� */
	if(RxLen2 > 0)//g_usart1_rx_end)    	
	{
		
		/* ��ս�����ɱ�־λ�����ռ���ֵ */
		// g_usart1_rx_end=0;
		// g_usart1_rx_cnt=0;
		
		switch(RxBuffer2[0])
		{
			case SOH://���ݰ���ʼ
			{
					crc = 0;
				
					/* ����crc16 */
					crc = crc16((uint8_t *)&RxBuffer2[3], BUF_SIZE, crc);
						
					if(crc != (RxBuffer2[BUF_SIZE+3]<<8|RxBuffer2[BUF_SIZE+4]))
					{
						RTT_printf(1,"CRC Error\r\n");
						return;
					}
						
					
					if((ymodem_get_state()==TO_START)&&(RxBuffer2[1] == 0x00)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// ��ʼ
					{

						ymodem_set_state(TO_RECEIVE_DATA);
						
						/* ��ymodem_send_cmdִ����sector_erase֮ǰ�����´������ݶ�������Ϊ������ر������ж� */
						/* ����Ӧ�ó���2������ */
						//sector_erase(APPLICATION_2_SECTOR);		

						/*����2����,������128kB*/
						osMutexWait(SPI1_MutexHandle, portMAX_DELAY);   //�ȴ�������
						W25QXX_Erase_Block(0);	
						W25QXX_Erase_Block(64*1024);	
						osMutexRelease(SPI1_MutexHandle); //�ͷŻ����� 

						osDelay(10);			
						
						data_state = 0x01;						
						ymodem_send_cmd(ACK);
						ymodem_send_cmd(CCC);

	
					}
					else if((ymodem_get_state()==TO_RECEIVE_END)&&(RxBuffer2[1] == 0x00)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// ����
					{
						update_set_down();						
						ymodem_set_state(TO_START);
						ymodem_send_cmd(ACK);
						
						/* ��һ��ʾ����ʾ������� */
						//beep_on();delay_ms(80);beep_off();
						if(data_state_sum != 0)
						{
							data_state_sum = 0;
							//memcpy(RxTestBuf+block_state*4096, _4k_buf, 4096);	//4096д��� ����
                            portDISABLE_INTERRUPTS();   //���ж�
							W25QXX_Write(_4k_buf, block_state*4096, 4096);
                            portENABLE_INTERRUPTS();    //���ж�
							block_state++;
						}
						
						/* ��λ */
						//NVIC_SystemReset();

                        //�������͡�����
                        ymodem_set_state(TO_START);
					}					
					else if((ymodem_get_state()==TO_RECEIVE_DATA)&&(RxBuffer2[1] == data_state)&&(RxBuffer2[2] == (uint8_t)(~RxBuffer2[1])))// ��������
					{

						/* ��¼���� */
						//flash_program((APPLICATION_2_ADDR + (data_state-1) * 128), (uint32_t *)(&RxBuffer2[3]), 32);
						if(data_state_sum<31)
						{
							memcpy(_4k_buf+data_state_sum*BUF_SIZE, (uint32_t *)(&RxBuffer2[3]), BUF_SIZE); //����������
							data_state_sum++;
						}
						else
						{
							memcpy(_4k_buf+data_state_sum*BUF_SIZE, (uint32_t *)(&RxBuffer2[3]), BUF_SIZE); //����������
                            data_state_sum = 0;
							//memcpy(RxTestBuf+block_state*4096, _4k_buf, 4096);	//4096д��� ����
                            portDISABLE_INTERRUPTS();   //���ж�
							W25QXX_Write(_4k_buf, block_state*4096, 4096);
                            portENABLE_INTERRUPTS();    //���ж�
							block_state++;
						}



						data_state++;
						ymodem_send_cmd(ACK);		
					}
			}break;
			
			case EOT://���ݰ��������
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




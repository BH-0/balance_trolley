/****************************************************************
*名    称:基于TCP服务器通过ymodem协议实现固件更新/文件传输
*作    者:温子祺
*创建日期:2021/05/28
*知 识 点:
	1.TCP服务器
	2.ymodem协议
	3.本地文件访问与传输
*****************************************************************/
#include <stdio.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

//ymodem用到的信号
#define SOH		0x01
#define STX		0x02
#define ACK		0x06
#define NACK	0x15
#define EOT		0x04
#define CCC		0x43


//状态机
#define YMODEM_STA_IDLE		0
#define YMODEM_STA_START	1
#define YMODEM_STA_DATA		2
#define YMODEM_STA_EOT1		3
#define YMODEM_STA_EOT2		4
#define YMODEM_STA_END		5

uint16_t crc16(uint8_t *addr, int32_t num, uint16_t crc);

unsigned long file_size_get(const char *pfile_path);

int main (int argc,char **argv)
{
	int fd_socket;
	int fd_client;
	
	int rt;
	int len;
	int crc;
	int m=0,n=0;
	
	char buf_rx[133];
	char buf_tx[133];
	
	unsigned int tx_seq=0;
	
	char file_name[128]={0};
	int  file_size=0;	
	int buf_tx_offset=0;
	
	
	int ymodem_sta = YMODEM_STA_IDLE;
	
	/* 检查输入的参数 */
	if(argv[1] ==NULL)
	{
		printf("please input format:./ymodem filename\r\n");
		return -1;
		
	}
	
	printf("This is ymodem server by Teacher.Wen\r\n");
	
	/* 获取文件名 */
	strcpy(file_name,argv[1]);
	
	/* 创建套接字，协议为IPv4，类型为TCP */
	fd_socket = socket(AF_INET,SOCK_STREAM,0);
	
	if(fd_socket<0)
	{
		perror("create socket fail:");
		
		return -1;
	}
	
	//30秒超时
	struct timeval timeout={30,0};	

	
	//设置发送超时
	rt = setsockopt(fd_socket,SOL_SOCKET,SO_SNDTIMEO,(const char *)&timeout,sizeof timeout);
	
	if(rt < 0)
	{
		perror("setsockopt SO_RCVTIMEO:");
		return -1;
	}
	
	//设置接收超时
	rt = setsockopt(fd_socket,SOL_SOCKET,SO_RCVTIMEO,(const char *)&timeout,sizeof timeout);
	
	if(rt < 0)
	{
		perror("setsockopt SO_RCVTIMEO:");
		return -1;
	}
	
	int on =1;
	/* 设置socket允许重复使用地址与端口 */
	setsockopt(fd_socket,SOL_SOCKET,SO_REUSEADDR,&on,sizeof on);
	
	struct sockaddr_in	 local_addr;
	
	local_addr.sin_family 		= AF_INET;						//IPv4
	local_addr.sin_port   		= htons(8888);					//本机端口为8888
	local_addr.sin_addr.s_addr	= htonl(INADDR_ANY);
	
	rt=bind(fd_socket,(struct sockaddr *)&local_addr,sizeof local_addr);
	
	if(rt < 0)
	{
		perror("bind");
		return -1;
	}
	
	/*	设置连接数目为1个 */
	rt = listen(fd_socket,5);
	
	if(rt < 0)
	{
		perror("listen fail:");
		return -1;		
	}
	
	struct sockaddr_in	 dest_addr;
	socklen_t dest_addr_len =sizeof dest_addr;
	
	printf("waiting tcp client...\r\n");
	
	/* 等待客户端连接 */
	fd_client = accept(fd_socket,(struct sockaddr *)&dest_addr, &dest_addr_len);
	
	if(fd_client < 0)
	{
		perror("accept fail:");
		return -1;	
	}
	
	char *p = inet_ntoa(dest_addr.sin_addr);
	
	printf("client ip:%s,port%d\n",p,ntohs(dest_addr.sin_port));
	

	while(1)
	{

		switch(ymodem_sta)
		{
			case YMODEM_STA_IDLE:
			{
				printf("[YMODEM_STA_IDLE]\r\n");
				
				bzero(buf_rx,sizeof buf_rx);
				
				len = recv(fd_client, buf_rx, sizeof buf_rx, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_IDLE]recv");
					printf("[YMODEM_STA_IDLE]please try again\r\n");
					break;
				}
					
				if(buf_rx[0]=='C')
				{
					ymodem_sta = YMODEM_STA_START;
				
				}
			}break;
			
			case YMODEM_STA_START:
			{
				printf("[YMODEM_STA_START]\r\n");
				memset(buf_tx,0,sizeof buf_tx);
				
				tx_seq=0;
				buf_tx_offset=0;
				file_size=0;				
				
				buf_tx[buf_tx_offset++]=(unsigned char )SOH;		//符号
				buf_tx[buf_tx_offset++]=(unsigned char )tx_seq;		//序号
				buf_tx[buf_tx_offset++]=(unsigned char )~tx_seq;	//序号反码
				
				/* 文件名 */
				memcpy(&buf_tx[buf_tx_offset],file_name,strlen(file_name));
				buf_tx_offset+=strlen(file_name);

				/* 文件名与文件大小要相隔一个空字符 */
				buf_tx_offset++;
				
				/* 文件大小 */				
				file_size = file_size_get(file_name);
				printf("file_size:%d\n",file_size);
				buf_tx[buf_tx_offset++]=(unsigned char)((file_size>>24)&0xFF);
				buf_tx[buf_tx_offset++]=(unsigned char)((file_size>>16)&0xFF);
				buf_tx[buf_tx_offset++]=(unsigned char)((file_size>>8)&0xFF);
				buf_tx[buf_tx_offset++]=(unsigned char)((file_size>>0)&0xFF);
				
				/* 计算crc16 */
				crc=0;
				crc=crc16(&buf_tx[3],128,0);
				buf_tx[131]=(unsigned char)((crc>>8)&0xFF);
				buf_tx[132]=(unsigned char)((crc>>0)&0xFF);
				
				/* 发送 */
				len = send(fd_client,buf_tx,133,0);
				if(len <= 0)
				{
					perror("[YMODEM_STA_START]send");
					break;
				}	
				
				/* 等待应答 */
				memset(buf_rx,0,sizeof buf_rx);
				len = recv(fd_client, buf_rx, 1, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_START]recv ack");
					break;
				}
				
				if(buf_rx[0] != ACK)
				{
					printf("[YMODEM_STA_START]please try again\r\n");
					
					ymodem_sta = YMODEM_STA_IDLE;
					break;
				}
				
				/* 等待大写字母C */
				memset(buf_rx,0,sizeof buf_rx);
				len = recv(fd_client, buf_rx, 1, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_START]recv ack");
					break;
				}	

				if(buf_rx[0] != CCC)
				{
					printf("[YMODEM_STA_START]please try again\r\n");
					ymodem_sta = YMODEM_STA_IDLE;
					break;
				}
				
				ymodem_sta = YMODEM_STA_DATA;

			}break;

		
			case YMODEM_STA_DATA:
			{	
				printf("YMODEM_STA_DATA\r\n");
				
				int fd = open(file_name,O_RDONLY);
			
				if(fd < 0)
				{
					perror("[YMODEM_STA_DATA]open");
					break;
				}
				
				while(1)
				{
					memset(buf_tx,0,sizeof buf_tx);
					
					len = read(fd,&buf_tx[3],128);//读取128字节数据
					
					if(len == 0)
					{
						ymodem_sta = YMODEM_STA_EOT1;
						break;
					}
					
					buf_tx[0]=(unsigned char )SOH;	//符号
					
					tx_seq++;
					buf_tx[1]=(unsigned char )tx_seq;	//序号
					buf_tx[2]=(unsigned char )~tx_seq;//序号反码
					
					
					//printf("YMODEM_STA_DATA read() len is %d\r\n",len);	
					
					/* 计算crc16 */
					crc=0;
					crc=crc16(&buf_tx[3],128,0);
					buf_tx[131]=(unsigned char)((crc>>8)&0xFF);
					buf_tx[132]=(unsigned char)((crc>>0)&0xFF);
					
					/* 发送 */
					len = send(fd_client,buf_tx,133,0);
					if(len <= 0)
					{
						perror("[YMODEM_STA_DATA]send");
						break;
					}	
					printf(":%d/%d\r\n", tx_seq, file_size/128+1);
					/* 等待应答 */
					memset(buf_rx,0,sizeof buf_rx);
					len = recv(fd_client, buf_rx, 1, 0);
					
					if(len <= 0)
					{
						perror("[YMODEM_STA_DATA]recv ack");
						break;
					}
					
					if(buf_rx[0] != ACK)
					{
						printf("[YMODEM_STA_DATA]please try again\r\n");
						
						ymodem_sta = YMODEM_STA_IDLE;
						break;
					}
					
					//printf(".");////////////////////////////////////////////////////////////////////////
				}

				close(fd);
				
				printf("\r\n");
				
			}break;
			
			case YMODEM_STA_EOT1:
			{
					printf("[YMODEM_STA_EOT1]\r\n");
					buf_tx[0]=EOT;
					
					/* 发送 */
					len = send(fd_client,buf_tx,1,0);
					if(len <= 0)
					{
						perror("[YMODEM_STA_EOT1]send");
						break;
					}	
					
					/* 等待应答 */
					memset(buf_rx,0,sizeof buf_rx);
					len = recv(fd_client, buf_rx, 1, 0);
					if(len <= 0)
					{
						perror("[YMODEM_STA_EOT1]recv nak");
						break;
					}
					
					/* 等待NAK */
					if(buf_rx[0] != NACK)
					{
						printf("[YMODEM_STA_EOT1]please try again\r\n");
						
						ymodem_sta = YMODEM_STA_IDLE;
						
						break;
					}

					ymodem_sta = YMODEM_STA_EOT2;					
			}break;
			
			case YMODEM_STA_EOT2:
			{
				printf("[YMODEM_STA_EOT2]\r\n");
				buf_tx[0]=EOT;
				
				/* 发送 */
				len = send(fd_client,buf_tx,1,0);
				if(len <= 0)
				{
					perror("[YMODEM_STA_EOT2]send");
					break;
				}	
				
				/* 等待应答 */
				memset(buf_rx,0,sizeof buf_rx);
				len = recv(fd_client, buf_rx, 1, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_EOT2]recv ack");
					break;
				}
				
				if(buf_rx[0] != ACK)
				{
					printf("[YMODEM_STA_EOT2]please try again\r\n");
					
					ymodem_sta = YMODEM_STA_IDLE;
					break;
				}
				
				/* 等待大写字母C */
				memset(buf_rx,0,sizeof buf_rx);
				len = recv(fd_client, buf_rx, 1, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_EOT2]recv ack");
					break;
				}	

				if(buf_rx[0] != CCC)
				{
					printf("[YMODEM_STA_EOT2]please try again\r\n");
					
					ymodem_sta = YMODEM_STA_IDLE;
					break;
				}
				
				ymodem_sta = YMODEM_STA_END;

			}break;		
		
			case YMODEM_STA_END:
			{
				printf("[YMODEM_STA_END]\r\n");
				buf_tx[0]=SOH;		
				buf_tx[1]=0x00;
				buf_tx[2]=0xFF;
				memset(&buf_tx[3],0,128);
				
				/* 计算crc16 */
				crc=0;
				crc=crc16(&buf_tx[3],128,0);
				buf_tx[131]=(unsigned char)((crc>>8)&0xFF);
				buf_tx[132]=(unsigned char)((crc>>0)&0xFF);
				
				/* 发送 */
				len = send(fd_client,buf_tx,133,0);
				if(len <= 0)
				{
					perror("[YMODEM_STA_END]send");
					break;
				}	
				
				/* 等待应答 */
				memset(buf_rx,0,sizeof buf_rx);
				len = recv(fd_client, buf_rx, 1, 0);
				
				if(len <= 0)
				{
					perror("[YMODEM_STA_END]recv ack");
					break;
				}
				
				if(buf_rx[0] != ACK)
				{
					printf("[YMODEM_STA_END]please try again\r\n");
					
					ymodem_sta = YMODEM_STA_IDLE;
					break;
				}			
				
				printf("ymodem download success\r\n");
				close(fd_socket);
				exit(0);
			}break;
		
			default:break;
		}
	
	}
	
	return 0;
}


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

/****************************************************
 *函数名称:file_size_get
 *输入参数:pfile_path	-文件路径
 *返 回 值:-1		-失败
		   其他值	-文件大小
 *说	明:获取文件大小
 ****************************************************/
unsigned long file_size_get(const char *pfile_path)
{
	unsigned long filesize = -1;	
	struct stat statbuff;
	
	if(stat(pfile_path, &statbuff) < 0)
	{
		return filesize;
	}
	else
	{
		filesize = statbuff.st_size;
	}
	
	return filesize;
}

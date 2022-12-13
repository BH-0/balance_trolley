#ifndef __YMODEM_H__
#define __YMODEM_H__
#include "sys.h"


#define SOH		0x01   
#define STX		0x02
#define ACK		0x06
#define NACK	0x15
#define EOT		0x04
#define CCC		0x43

#define BUF_SIZE 128  //单次传输数据包大小 128或1024 

/* 升级的步骤 */
enum UPDATE_STATE
{
	TO_START = 0x01,
	TO_RECEIVE_DATA = 0x02,
	TO_RECEIVE_EOT1 = 0x03,
	TO_RECEIVE_EOT2 = 0x04,
	TO_RECEIVE_END = 0x05
};



extern void ymodem_download(void);
uint8_t ymodem_get_state(void);
void ymodem_send_cmd(uint8_t command);

#endif


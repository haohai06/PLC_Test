#ifndef __LOCK_APP_UART_H__

#define __LOCK_APP_UART_H__

#include "stdint.h"
#define UART_CMD_REQ           0
#define UART_CMD_ACK           1

#define UART_CMD_HELLO          0x1
#define UART_CMD_LOCK           0x2 
#define UART_CMD_REQ_LOCK       0x3 
#define UART_CMD_PLAY_DOORBELL  0x4
#define UART_CMD_SETTING        0x5
#define UART_CMD_REPORT         0x6 
#define UART_CMD_QUERY          0x7
#define UART_CMD_REVERSE_LOCK   0x8
#define UART_CMD_REQ_ADMIN_MENU 0x9
#define UART_CMD_NOTFIY_ADMIN_PWD 0x0A
#define UART_CMD_SET_TIME         0x10
#define UART_CMD_SET_PARAMS       0x11
#define UART_CMD_GET_PARAMS       0x12
//#define UART_CMD_REPORT_VER     0x9
#define UART_CMD_REPORT_VER     0xA9

#define UART_CMD_TRANSPARENT    0x5A

#define UART_CMD_BYE            0x7e


#define UART_INIT_STATE   0 
#define UART_RECV_STATE   1

int uart_rx_normal_io_init(void);
int  app_uart_init(void);  

int lock_app_uart_cmd_handler(uint8_t *data,uint16_t len);
uint16_t  lock_app_send_cmd(uint8_t cmd,uint8_t type,uint8_t req,uint8_t *payload,uint16_t payloadlen);
uint16_t lock_app_send_hello(void);
uint16_t lock_app_send_bye(void);
uint16_t lock_app_send_req_lock(uint8_t lock);
uint16_t lock_app_send_report_lock_state(uint8_t lock_state);
uint16_t lock_app_send_reverse_lock(void);
uint16_t lock_app_report_version(void);
uint16_t lock_app_send_req_admin_menu(void);

uint8_t get_uart_tx_queue_active(void);

void set_uart_recv_state( int state);
void lock_uart_send_frame(void);

void  app_uart_rx_poll(void);
void uart_task_event_init(uint8_t task_id);
uint16_t uart_task_process_event( uint8_t task_id, uint16_t events);
uint8_t get_uart_task_event_active(void);
void uart_task_enter_sleep(void);
void uart_interrupt_handler(void);
	
#endif /*__LOCK_APP_UART_H__*/

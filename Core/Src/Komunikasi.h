/*
 * Komunikasi.h
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_KOMUNIKASI_H_
#define SRC_KOMUNIKASI_H_

#include "main.h"
#include <stdbool.h>

typedef enum{
	MOVE_NULL = 0x00U,
	MOVE_X = 0x01U,
	MOVE_Y = 0x02U,
	MOVE_TRANS_A = 0x03U,
	MOVE_TRANS_B = 0x04U,
}mode_jalan_t;

typedef struct{
	bool ping;
	bool standby;
	bool move;
	bool req;
	bool send;
}feedback_t;

typedef enum{
	PING 		= 0x01U,
	STEADY 	= 0x02U,
	MOVE 		= 0x03U,
	REQ			= 0x04U,
	SEND		= 0x05U
}msg_type_t;

typedef struct{
	int32_t encA;
	int32_t encB;
	int32_t encC;
	int32_t encD;
}encoder_msg_t;

typedef struct{
	int32_t 			walkpoint;
	int8_t 				speed;
	encoder_msg_t enc;
	mode_jalan_t	move_type;
	msg_type_t		msg_type;
}com_get_t;

void komunikasi_init(UART_HandleTypeDef* uart_handler);
static uint8_t checksum_generator(uint8_t* arr, uint8_t size);
bool tx_ping(void);
bool tx_steady(void);
bool tx_move(int8_t speed, mode_jalan_t mode, uint32_t walkpoint);
bool tx_req(void);
bool tx_send(int32_t enc_val);
bool tx_send_all(int16_t enc_a_val, int16_t enc_b_val, int16_t enc_c_val, int16_t enc_d_val);
void rx_start(void);
void rx_feedback(feedback_t* fed);
void rx_start_get(void);
void rx_get(com_get_t* get);

#endif /* SRC_KOMUNIKASI_H_ */

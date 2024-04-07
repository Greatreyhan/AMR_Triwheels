/*
 * Komunikasi.c
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */
#include "Komunikasi.h"

#define TIMEOUT 10

static UART_HandleTypeDef* huart;
static uint8_t rxbuf[3];
static uint8_t rxbuf_get[16];

void komunikasi_init(UART_HandleTypeDef* uart_handler){
	huart = uart_handler;
}

static uint8_t checksum_generator(uint8_t* arr, uint8_t size){
	uint8_t chksm = 0;
	for(uint8_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFF);
}

bool tx_ping(void){
	uint8_t ping[16] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[15] = checksum_generator(ping, 16);

	if(HAL_UART_Transmit(huart, ping, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_steady(void){
	uint8_t steady[16] = {0xA5, 0x5A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	steady[15] = checksum_generator(steady, 16);

	if(HAL_UART_Transmit(huart, steady, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_move(int8_t speed, mode_jalan_t mode, uint32_t walkpoint){
	uint8_t move[16] = {0xA5, 0x5A, 0x03, speed, mode, ((walkpoint >> 24) & 0xFF), ((walkpoint >> 16) & 0xFF), ((walkpoint >> 8) & 0xFF), (walkpoint & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	move[15] = checksum_generator(move, 16);

	if(HAL_UART_Transmit(huart, move, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_req(void){
	uint8_t req[16] = {0xA5, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	req[15] = checksum_generator(req, 16);

	if(HAL_UART_Transmit(huart, req, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_send(int32_t enc_val){
	uint8_t send[16] = {0xA5, 0x5A, 0x05, ((enc_val >> 24) & 0xFF), ((enc_val >> 16) & 0xFF), ((enc_val >> 8) & 0xFF), (enc_val & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	send[15] = checksum_generator(send, 16);

	if(HAL_UART_Transmit(huart, send, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_send_all(int16_t enc_a_val, int16_t enc_b_val, int16_t enc_c_val, int16_t enc_d_val){
	uint8_t send[16] = {0xA5, 0x5A, 0x06, ((enc_a_val >> 8) & 0xFF), ((enc_a_val) & 0xFF),((enc_b_val >> 8) & 0xFF), ((enc_b_val) & 0xFF), ((enc_c_val >> 8) & 0xFF), ((enc_c_val) & 0xFF), ((enc_d_val >> 8) & 0xFF), ((enc_d_val) & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00};
	send[15] = checksum_generator(send, 16);

	if(HAL_UART_Transmit(huart, send, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

void rx_start(void){
	HAL_UART_Receive_DMA(huart,rxbuf, 3);
}

void rx_start_get(void){
	HAL_UART_Receive_DMA(huart,rxbuf_get, 16);
}

void rx_feedback(feedback_t* fed){
	if(rxbuf[0] == 0xA5 && rxbuf[1]  == 0x5A){
		if(rxbuf[2] == 0x01) fed->ping = true;
		else if(rxbuf[2] == 0x02) fed->standby = true;
		else if(rxbuf[2] == 0x03) fed->move = true;
		else if(rxbuf[2] == 0x04) fed->req = true;
		else if(rxbuf[2] == 0x05) fed->send = true;
	}
	HAL_UART_Receive_DMA(huart,rxbuf, 3);
}

void rx_get(com_get_t* get){
	for(int i = 0; i < 16; i++){
		if((rxbuf_get[i] == 0xA5) && (rxbuf_get[i+1] == 0x5A)){

			// Check for Ping
			if(rxbuf_get[i+2] == 0x01){
				get->msg_type = PING;
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x01};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
			}

			// Check for Steady
			else if(rxbuf_get[i+2] == 0x02){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x02};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->msg_type = STEADY;
			}

			// Check for Move
			else if(rxbuf_get[i+2] == 0x03){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x03};

				// Check Speed Value
				if((rxbuf_get[i+3] & 0x80)) get->speed = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->speed = (rxbuf_get[i+3] << 8) | rxbuf_get[i+4];

				// Check Move Mode
				get->move_type = rxbuf_get[i+5];

				// Check Walkpoint value
				if((rxbuf_get[i+6] & 0x80)) get->walkpoint = ((rxbuf_get[i+6] << 24) | rxbuf_get[i+7] << 16 | rxbuf_get[i+8] << 8 | rxbuf_get[i+9])-(4294967296);
				else get->walkpoint = ((rxbuf_get[i+6] << 24) | rxbuf_get[i+7] << 16 | rxbuf_get[i+8] << 8 | rxbuf_get[i+9]);
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->msg_type = MOVE;

			}

			// Check for Request
			else if(rxbuf_get[i+2] == 0x04){
					get->msg_type = REQ;
					uint8_t txbuf[3] = {0xA5, 0x5A, 0x04};
					HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
			}

			// Check for Send
			else if(rxbuf_get[i+2] == 0x05){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x05};

				// Check Walkpoint value
				if((rxbuf_get[i+3] & 0x80)) get->walkpoint = ((rxbuf_get[i+3] << 24) | rxbuf_get[i+4] << 16 | rxbuf_get[i+5] << 8 | rxbuf_get[i+6])-(4294967296);
				else get->walkpoint = ((rxbuf_get[i+3] << 24) | rxbuf_get[i+4] << 16 | rxbuf_get[i+5] << 8 | rxbuf_get[i+6]);
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->msg_type = SEND;

			}

			// Check for Send All
			else if(rxbuf_get[i+2] == 0x06){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x05};

				// Check Walkpoint value
				if((rxbuf_get[i+3] & 0x80)) get->walkpoint = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->walkpoint = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4]);
				if((rxbuf_get[i+5] & 0x80)) get->walkpoint = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->walkpoint = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				if((rxbuf_get[i+7] & 0x80)) get->walkpoint = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->walkpoint = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8]);
				if((rxbuf_get[i+9] & 0x80)) get->walkpoint = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10])-(65536);
				else get->walkpoint = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10]);
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->msg_type = SEND;

			}
		}
	}
	HAL_UART_Receive_DMA(huart, rxbuf_get, 16);
}

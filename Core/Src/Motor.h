/*
 * Motor.h
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "main.h"
#include <stdbool.h>

typedef struct{
	TIM_HandleTypeDef* 	tim;
	TIM_TypeDef* 				tim_number;
	uint32_t						counter;
	int16_t							counts;
	int16_t							position;
	int16_t							old_position;
	int									speed;
}encoder_t;

typedef struct{
	TIM_HandleTypeDef* 	tim_R;
	TIM_HandleTypeDef* 	tim_L;
	TIM_TypeDef* 				tim_number_R;
	TIM_TypeDef* 				tim_number_L;
	uint8_t 						channel_R;
	uint8_t 						channel_L;
	int16_t 						speed_R;
	int16_t 						speed_L;
	GPIO_TypeDef*				EN_PORT_R;
	GPIO_TypeDef*				EN_PORT_L;
	uint16_t						EN_PIN_R;
	uint16_t						EN_PIN_L;
	encoder_t						ENC;
}motor_t;

/*
	||****** Run Omnidirection Wheel *****||
	-	motor_t motor
	- int16_t speed 		: 0-1000
	- uint8_t direction : 1 -> clockwise
												2 -> counterclockwise
*/
void agv_run_motor(motor_t motor, int16_t speed);
void agv_stop(motor_t motor);
void agv_stop_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD);

// Free move
void agv_play_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);

// Move Based on Encoder
void agv_move_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);

/*
	||****** Encoder Init *****||
	-	encoder_t encoder
	- TIM_HandleTypeDef* tim 	: timer used
	- TIM_TypeDef*						: TIM Number
*/
void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim, TIM_TypeDef* tim_number);

#endif /* SRC_MOTOR_H_ */

/*
 * Motor.c
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#include "Motor.h"

void agv_run_motor(motor_t motor, int16_t speed){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_SET);
	if(speed > 0){
		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_4);
		}
		if(motor.channel_L == 1){
			motor.tim_number_L->CCR1 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
	else if(speed < 0){
		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_4);
		}
		if(motor.channel_L == 1){
			motor.tim_number_L->CCR1 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
}

void agv_play_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed){
	agv_run_motor(motorA, speed);
	agv_run_motor(motorB, speed);
	agv_run_motor(motorC, speed);
	agv_run_motor(motorD, speed);
}

void agv_play_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed){
	agv_run_motor(motorA, speed);
	agv_run_motor(motorB, speed);
	agv_run_motor(motorC, -speed);
	agv_run_motor(motorD, -speed);
}

void agv_play_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed){
	agv_run_motor(motorA, -speed);
	agv_run_motor(motorB, speed);
	agv_run_motor(motorC, -speed);
	agv_run_motor(motorD, speed);
}

void agv_play_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed){
	agv_stop(motorA);
	agv_stop(motorC);
	agv_stop(motorB);
	agv_stop(motorD);
	agv_run_motor(motorA, speed);
	agv_run_motor(motorC, -speed);
}

void agv_play_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed){
	if(speed > 0){
	agv_stop(motorA);
	agv_stop(motorC);
	agv_run_motor(motorB, speed);
	agv_run_motor(motorD, -speed);
	}
	else{
	agv_stop(motorA);
	agv_stop(motorC);
	agv_run_motor(motorB, -(speed*-1));
	agv_run_motor(motorD, (speed*-1));
	}
}

void agv_stop(motor_t motor){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_RESET);
}

void agv_stop_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD){
	agv_stop(motorA);
	agv_stop(motorB);
	agv_stop(motorC);
	agv_stop(motorD);
}

void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim,TIM_TypeDef* tim_number){
	encoder.tim = tim;
	encoder.tim_number = tim_number;
	HAL_TIM_Encoder_Start_IT(tim, TIM_CHANNEL_ALL);
}

void agv_move_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist){
	if(speed > 0 && dist > 0){
		while(motorA.ENC.position < dist && motorB.ENC.position < dist && motorC.ENC.position < dist && motorD.ENC.position < dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorC, speed);
			agv_run_motor(motorD, speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
	else if(speed < 0 && dist < 0){
		while(motorA.ENC.position > dist && motorB.ENC.position > dist && motorC.ENC.position > dist && motorD.ENC.position > dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorC, speed);
			agv_run_motor(motorD, speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
}

void agv_move_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist){
	if(speed > 0 && dist > 0){
		while(motorA.ENC.position < dist && motorB.ENC.position < dist && (-motorC.ENC.position) < dist && (-motorD.ENC.position) < dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorC, -speed);
			agv_run_motor(motorD, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
	else if(speed < 0 && dist < 0){
		while(motorA.ENC.position > dist && motorB.ENC.position > dist && (-motorC.ENC.position) > dist && (-motorD.ENC.position) > dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorC, -speed);
			agv_run_motor(motorD, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
}

void agv_move_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist){
	if(speed > 0 && dist > 0){
		while(motorA.ENC.position < dist && (-motorB.ENC.position) < dist && (-motorC.ENC.position) < dist && motorD.ENC.position < dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, -speed);
			agv_run_motor(motorC, -speed);
			agv_run_motor(motorD, speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
	else if(speed < 0 && dist < 0){
		while(motorA.ENC.position > dist && (-motorB.ENC.position) > dist && (-motorC.ENC.position) > dist && motorD.ENC.position > dist){
			agv_run_motor(motorA, speed);
			agv_run_motor(motorB, -speed);
			agv_run_motor(motorC, -speed);
			agv_run_motor(motorD, speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
}

void agv_move_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist){
	if(speed > 0 && dist > 0){
		while(motorA.ENC.position < dist  && (-motorC.ENC.position) < dist){
			agv_stop(motorB);
			agv_stop(motorD);
			agv_run_motor(motorA, speed);
			agv_run_motor(motorC, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
	else if(speed < 0 && dist < 0){
		while(motorA.ENC.position > dist && (-motorC.ENC.position) > dist){
			agv_stop(motorB);
			agv_stop(motorD);
			agv_run_motor(motorA, speed);
			agv_run_motor(motorC, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
}

void agv_move_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist){
	if(speed > 0 && dist > 0){
		while(motorB.ENC.position < dist && (-motorD.ENC.position) < dist){
			agv_stop(motorA);
			agv_stop(motorC);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorD, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
	else if(speed < 0 && dist < 0){
		while(motorB.ENC.position > dist && (-motorD.ENC.position > dist)){
			agv_stop(motorA);
			agv_stop(motorC);
			agv_run_motor(motorB, speed);
			agv_run_motor(motorD, -speed);
		}
		agv_stop_all(motorA,motorB,motorC,motorD);
	}
}

/*
 * Motor.c
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#include "Motor.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>

void agv_run_motor(motor_t motor, int16_t speed){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_SET);

	if(speed >= 0){
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
			motor.tim_number_L->CCR1 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
	else if(speed < 0){

		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = -speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = -speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = -speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = -speed;
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
}


void agv_stop(motor_t motor){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_RESET);
}

void agv_stop_all(motor_t motorA, motor_t motorB, motor_t motorC){
	agv_stop(motorA);
	agv_stop(motorB);
	agv_stop(motorC);
}

void agv_reset_all(motor_t motorA, motor_t motorB, motor_t motorC){
	agv_run_motor(motorA,0);
	agv_run_motor(motorB,0);
	agv_run_motor(motorC,0);
}

void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim,TIM_TypeDef* tim_number){
	encoder.tim = tim;
	encoder.tim_number = tim_number;
	HAL_TIM_Encoder_Start_IT(tim, TIM_CHANNEL_ALL);
}

// Kinematics
double agv_kinematic_Sy(int pos_A, int pos_B, int pos_C){
	double sy = (0.866*pos_A) + (-0.866*pos_B) + (0*pos_C);
	return sy;
}
double agv_kinematic_Sx(int pos_A, int pos_B, int pos_C){
	double sx = (-0.5*pos_A) + (-0.5*pos_B) + (1*pos_C);
	return sx;
}
double agv_kinematic_St(int pos_A, int pos_B, int pos_C){
	double st = (1*pos_A) + (1*pos_B) + (1*pos_C);
	return st;
}
void agv_forward_kinematic(encoder_t enc_A, encoder_t enc_B, encoder_t enc_C, kinematic_t kinematic){
	kinematic.Sx = (0.86*enc_A.position) + (-0.866*enc_B.position) + (0*enc_C.position);
	kinematic.Sy = (-0.5*enc_A.position) + (-0.5*enc_B.position) + (1*enc_C.position);
	kinematic.St = (1*enc_A.position) + (1*enc_B.position) + (1*enc_C.position);
}

void agv_inverse_kinematic(double vx, double vy, double vt, motor_t motorA, motor_t motorB, motor_t motorC){
	double V1 = (-0.33*vx) + (0.58*vy) + (0.33*vt);
	double V2 = (-0.33*vx) + (-0.58*vy) + (0.33*vt);
	double V3 = (0.67*vx) + (0*vy) + (0.33*vt);
	agv_speed_to_pwm(motorA, V1);
	agv_speed_to_pwm(motorB, V2);
	agv_speed_to_pwm(motorC, V3);
}

double agv_calculate_encoder(encoder_t encoder){
	double k_wheel = 2*3.14*30;
	return encoder.position*(7/k_wheel);
}

void agv_calculate_distance(kinematic_t agv){
	agv.Sy = agv.S1*(0.866)-agv.S2*(0.866);
	agv.Sx = agv.S3-agv.S1*(0.5)-agv.S2*(0.5);
	agv.St = (agv.S1/15) + (agv.S2/15) + (agv.S3/15);
}

void agv_calculate_rotational_matrix(kinematic_t agv){
	agv.Yp = cos(agv.St) - sin(agv.St) + 0;
	agv.Xp = sin(agv.St) + cos(agv.St) + 0;
	agv.Tp = 0 + 0 + agv.St;
}

void agv_speed_to_pwm(motor_t motor, double speed){
	// Maximum 628,2 mm/s
	if(speed < 628.2){
		agv_run_motor(motor, (speed*1.592));
	}
	else{
		agv_run_motor(motor, (628.2*1.592));
	}
}

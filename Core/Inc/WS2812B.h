/*
 * WS2812B.h
 *
 *  Created on: Feb 6, 2024
 *      Author: colte
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "main.h"

#define LED_MAX 60
#define USE_BRIGHTNESS 1
#define PI 3.14159265

extern uint8_t LED_Data[LED_MAX][4];
extern uint8_t LED_Mod[LED_MAX][4];

extern int datasentflag;

extern uint16_t pwmData[(24*LED_MAX)+50];

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void Set_LED(int LEDnum, int r, int g, int b);

void Set_Brightness(int brightness);

void WS2812B_Send(TIM_HandleTypeDef *htim1);

#endif /* INC_WS2812B_H_ */

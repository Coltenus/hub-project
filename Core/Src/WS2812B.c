/*
 * WS2812B.c
 *
 *  Created on: Feb 6, 2024
 *      Author: colte
 */
#include "WS2812B.h"
#include <math.h>

void Set_LED(int LEDnum, int r, int g, int b) {
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = g;
	LED_Data[LEDnum][2] = r;
	LED_Data[LEDnum][3] = b;
}

void Set_Brightness(int brightness) {
#if USE_BRIGHTNESS
	if(brightness > 45) brightness = 45;
	for(int i = 0; i<LED_MAX; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for(int j = 1; j<4; j++) {
			float angle = 90-brightness;
			angle = angle*PI/180;
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}
#endif
}

void WS2812B_Send(TIM_HandleTypeDef *htim1) {
	uint32_t indx = 0, color;

	for(int i = 0; i<LED_MAX; i++) {
		color = (LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | LED_Mod[i][3];
		for (int i = 23; i>=0; i--) {
			if(color&(1<<i)) pwmData[indx] = 60;
			else pwmData[indx] = 30;
			indx++;
		}
	}

	for(int i = 0; i<50; i++) {
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while(!datasentflag);
	datasentflag = 0;
}

/*
 * ESP8266_HAL.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */

#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_


void ESP_Init_Server (char *SSID, char *PASSWD);
void ESP_Init_SoftAP (char *SSID, char *PASSWD);

void Server_Start (void);
void SoftAP_Start (void);


#endif /* INC_ESP8266_HAL_H_ */

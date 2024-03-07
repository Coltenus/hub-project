/*
 * ESP8266_HAL.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */


#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"

#include <stdbool.h>

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "yuarel.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define wifi_uart &huart1
#define pc_uart &huart2

extern void textToQrCode(uint8_t x, uint8_t y, const char* text, uint8_t multiplier, uint8_t border);
extern bool led;

char buffer[20];

char *base_start = R"rawliteral(<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>STM32 Web page</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet" integrity="sha384-T3c6CoIi6uLrA9TneNEoa7RxnatzjcDSCmG1MXxSR1GAsXEV/Dwwykc2MPK8M2HN" crossorigin="anonymous">
</head>
<body>)rawliteral";
char *base_end = R"rawliteral(<script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/js/bootstrap.bundle.min.js" integrity="sha384-C6RzsynM9kWDrMNeT87bh95OGNyZPhcTNXj1NW7RuBCsyN/o0jlpcV8Qyq46cDfL" crossorigin="anonymous"></script>
</body>
</html>)rawliteral";

char *controller = R"rawliteral(<div class="container">
        <div class="input-group">
                    <input type="text" class="form-control" name="url" onchange="document.getElementById('bt-1').href = '/qrcode?text=' + this.value;">
                    <a id="bt-1" href="/" class="btn btn-primary">Set</a>
                </div>
		<a href="/led" class="btn btn-primary">Toogle led</a>
    </div>)rawliteral";

/*****************************************************************************************************************************************/

void ESP_Init_Server (char *SSID, char *PASSWD)
{
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n", wifi_uart);
	HAL_Delay(300);

	/********* AT **********/
	Uart_sendstring("AT\r\n", wifi_uart);
	while(!(Wait_for("AT\r\r\n\r\nOK\r\n", wifi_uart)));


	/********* AT+CWMODE=1 **********/
	Uart_sendstring("AT+CWMODE=1\r\n", wifi_uart);
	while (!(Wait_for("AT+CWMODE=1\r\r\n\r\nOK\r\n", wifi_uart)));


	/********* AT+CWJAP="SSID","PASSWD" **********/
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("WIFI GOT IP\r\n\r\nOK\r\n", wifi_uart)));
	sprintf (data, "Connected to %s\n\r", SSID);
	Uart_sendstring(data,pc_uart);


	/********* AT+CIFSR **********/
	Uart_sendstring("AT+CIFSR\r\n", wifi_uart);
	while (!(Wait_for("CIFSR:STAIP,\"", wifi_uart)));
	while (!(Copy_upto("\"",buffer, wifi_uart)));
	while (!(Wait_for("OK\r\n", wifi_uart)));
	int len = strlen (buffer);
	buffer[len-1] = '\0';
	sprintf (data, "IP: http://%s/\n\r", buffer);
	Uart_sendstring(data, pc_uart);


	Uart_sendstring("AT+CIPMUX=1\r\n", wifi_uart);
	while (!(Wait_for("AT+CIPMUX=1\r\r\n\r\nOK\r\n", wifi_uart)));

	Uart_sendstring("AT+CIPSERVER=1,80\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));

	Uart_sendstring("IP is accessable\n\r", pc_uart);
}

void ESP_Init_SoftAP (char *SSID, char *PASSWD)
{
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n", wifi_uart);
	HAL_Delay(300);

	/********* AT **********/
	Uart_sendstring("AT\r\n", wifi_uart);
	while(!(Wait_for("AT\r\r\n\r\nOK\r\n", wifi_uart)));


	/********* AT+CWMODE=1 **********/
	Uart_sendstring("AT+CWMODE=2\r\n", wifi_uart);
	while (!(Wait_for("AT+CWMODE=2\r\r\n\r\nOK\r\n", wifi_uart)));


	/********* AT+CWJAP="SSID","PASSWD" **********/
	sprintf (data, "AT+CWSAP=\"%s\",\"%s\",5,3\r\n", SSID, PASSWD);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	sprintf (data, "Started sharing %s\n\r", SSID);
	Uart_sendstring(data,pc_uart);
}

int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for(">", wifi_uart)));
	Uart_sendstring (str, wifi_uart);
	while (!(Wait_for("SEND OK", wifi_uart)));
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	return 1;
}

bool CheckParam(char* url_str, const char* key, const char* val, uint8_t pcount) {
	char buf[200] = {0};
	struct yuarel url;
	struct yuarel_param params[pcount];
	uint8_t p;
	if(yuarel_parse(&url, url_str) != -1) {
		p = yuarel_parse_query(url.query, '&', params, pcount);
		for(int i = 0; i<p; i++) {
			if(!strcmp(params[i].key, key)) {
				if(!strcmp(params[i].val, val)) return true;
				return false;
			}
		}
	}
	return false;
}

const char* GetParam(char* url_str, const char* key, uint8_t pcount) {
	struct yuarel url;
	struct yuarel_param params[pcount];
	uint8_t p;
	if(yuarel_parse(&url, url_str) != -1) {
		p = yuarel_parse_query(url.query, '&', params, pcount);
		for(int i = 0; i<p; i++) {
			if(!strcmp(params[i].key, key)) {
				return params[i].val;
			}
		}
	}
	return "";
}

void Server_Handle (char *str, int Link_ID)
{
	char datatosend[1024] = {0};
	sprintf (datatosend, base_start);
	if(!strncmp(str, "/check", 6)) {
		char buf[20] = {0};
		if(strlen(str) == 6) snprintf(buf, 200, "<h2>Check</h2>");
		else {
			snprintf(buf, 20, "%s", GetParam(str, "header", 2));
			if(buf[0] != '\0') snprintf(buf, 200, "<h2>Check</h2><h4>%s</h4>", buf);
			else snprintf(buf, 200, "<h2>Check</h2>");
		}
		strcat (datatosend, buf);
	}
	else {
		strcat (datatosend, controller);
	}
	strcat (datatosend, base_end);
	Server_Send(datatosend, Link_ID);

}

void GetAddress(char *str, uint8_t shift, char* result) {
	if(!result) return;
	int i = 0;
	do {
		result[i] = str[i+shift];
		i++;
	} while (str[i+shift] != ' ' && str[i+shift] != '\0');
	result[i] = '\0';
}

char* ReplaceSymbolSeq(char *str) {
	int len = strlen(str);
	for(int i = 0; i+2<len; i++) {
		if(str[i] == '%' && str[i+1] == '2' && str[i+2] == '0') {
			for(int j = i+1; j<len-1; j++) {
				str[j] = str[j+2];
			}
			len -= 2;
			str[i] = ' ';
		}
	}
	return str;
}

void Server_Start (void)
{
	char buftocopyinto[64] = {0};
	char Link_ID;
	while (!(Get_after("+IPD,", 1, &Link_ID, wifi_uart)));
	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftocopyinto, wifi_uart)));
	if (Look_for("/check", buftocopyinto) == 1) {
		char buf[64];
		GetAddress(buftocopyinto, 9, buf);
		Uart_sendstring(buf, pc_uart);
		Uart_sendstring("\n\r", pc_uart);
		HAL_Delay(50);
		Server_Handle(buf, Link_ID);
	}
	if (Look_for("/qrcode", buftocopyinto) == 1) {
		char buf[64];
		GetAddress(buftocopyinto, 9, buf);
		Uart_sendstring(buf, pc_uart);
		Uart_sendstring("\n\r", pc_uart);
		snprintf(buf, 64, "%s", GetParam(buf, "text", 1));
		textToQrCode(0, 0, ReplaceSymbolSeq(buf), 4, 1);
		Server_Handle("/", Link_ID);
	}
	if (Look_for("/led", buftocopyinto) == 1) {
		char buf[64];
		GetAddress(buftocopyinto, 9, buf);
		Uart_sendstring(buf, pc_uart);
		Uart_sendstring("\n\r", pc_uart);
		snprintf(buf, 64, "%s", GetParam(buf, "state", 1));
		if(!strcmp(buf, "on")) {
			led = true;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else if(!strcmp(buf, "off")) {
			led = false;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		else {
			led = !led;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		Server_Handle("/", Link_ID);
	}
	else Server_Handle("/", Link_ID);
}

void SoftAP_Start (void) {
	int i = 0;
	char c, data[200];
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CWLIF\r\n", wifi_uart);
	while (!Wait_for("AT+CWLIF\r", wifi_uart));
	while (!Copy_upto("\r\n", data, wifi_uart));
	data[strlen(data)-2] = '\0';
	if(Look_for("192.168.", data) == 1) {
		snprintf (data, 200, "%s\n\r", data);
		Uart_sendstring(data, pc_uart);
		i++;
	}
	do {
		while (!Copy_upto("\r\n", data, wifi_uart));
		data[strlen(data)-2] = '\0';
		if(Look_for("192.168.", data) == 1) {
			snprintf (data, 200, "%s\n\r", data);
			Uart_sendstring(data, pc_uart);
			i++;
		}
	} while (Look_for("OK", data) == -1);
	if(i > 0) Uart_sendstring("\n\r", pc_uart);
}

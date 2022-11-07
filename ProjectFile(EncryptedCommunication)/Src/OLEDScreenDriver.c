/*
 * OLEDScreenDriver.c
 *
 *  Created on: Nov 7, 2022
 *      Author: m
 */

#include "OLEDScreenDriver.h"
#include "ssd1306.h"
#define width 128
#define length 64;

int currentLinePix;

void clearScreen(){
	currentLinePix = 0;
	SSD1306_Clear();
}
//switch line each time call this method;
void printToScreen(char *buffer){
	if(currentLinePix>=60){
		clearScreen();
	}
	SSD1306_GotoXY (0, currentLinePix);
	SSD1306_Puts (buffer, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	currentLinePix += 10;
}

void InitScreen(){
	currentLinePix = 0;
	SSD1306_Init();
	SSD1306_Fill(0);
	clearScreen();
}


/*
 * testsdd1306.c
 *
 *  Created on: Nov 7, 2022
 *      Author: m
 */
#include <testFunctionalities.h>

void Testssd1306Driver(){
	  SSD1306_Init();
	  char snum[5];

	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("NIZAR", &Font_11x18, 1);
	  SSD1306_GotoXY (0, 30);
	  SSD1306_Puts ("MOHIDEEN", &Font_11x18, 1);
	  SSD1306_UpdateScreen();
	  HAL_Delay (1000);

	  SSD1306_ScrollRight(0,7);
	  HAL_Delay(3000);
	  SSD1306_ScrollLeft(0,7);
	  HAL_Delay(3000);
	  SSD1306_Stopscroll();
	  SSD1306_Clear();
	  SSD1306_GotoXY (35,0);
	  SSD1306_Puts ("SCORE", &Font_11x18, 1);


	  while (1)
	  {
		for ( int x = 1; x <= 10000 ; x++ )
		{
			itoa(x, snum, 10);
			SSD1306_GotoXY (0, 30);
			SSD1306_Puts ("             ", &Font_16x26, 1);
			SSD1306_UpdateScreen();
			if(x < 10) {
				SSD1306_GotoXY (53, 30);  // 1 DIGIT
			}
			else if (x < 100 ) {
				SSD1306_GotoXY (45, 30);  // 2 DIGITS
			}
			else if (x < 1000 ) {
				SSD1306_GotoXY (37, 30);  // 3 DIGITS
			}
			else {
				SSD1306_GotoXY (30, 30);  // 4 DIGIS
			}
			SSD1306_Puts (snum, &Font_16x26, 1);
			SSD1306_UpdateScreen();
			HAL_Delay (500);
		 }
	  }
}

void testOLEDScreenDriverPrint(){
	  InitScreen();
	  printToScreen("hello world");
	  printToScreen("welcome to the club");
	  printToScreen("it is not a joke");
	  printToScreen("it is not a joke");
	  printToScreen("it is not a joke");
	  printToScreen("it is not a joke");
	  HAL_Delay(5000);
	  printToScreen("it is a big joke");
	  printToScreen(" ");
	  printToScreen("it is a big joke2");
	  HAL_Delay(5000);
	  clearScreen();
}

void testKeypadDriver(){
	InitScreen();
	while(1){
		char buf[1];
		char result = getOneCharFromKeypad();
		buf[0] = result;
		//InitScreen();
		printToScreen(buf);
		HAL_Delay(500);
		clearScreen();
	}
}

/*
 * keypadDriver.c
 *
 *  Created on: Nov 9, 2022
 *      Author: m
 */
#include "keypadDriver.h"

int keypadActive = 0;
char *keyPadResult;
const char keyPadLayout[4][4] = {	{'1','2','3','A'},
							{'4','5','6','B'},
							{'7','8','9','C'},
							{'#','0','*','D'}};

void IniteKeypad(){
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
	keypadActive = 0;
	keyPadResult = NULL;

}

void ClearKeypad(){
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
}

//public api function
char getOneCharFromKeypad(){
	IniteKeypad();
	keypadActive = 1;
	while(keyPadResult == NULL){}
	keypadActive = 0;
	ClearKeypad();
	return *keyPadResult;

}

char detectKey(uint16_t GPIO_Pin){
	int R = 0;
	int C = 0;
	if(GPIO_Pin == R1_Pin){
		R = 0;
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET);
			C = 3;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);
			C = 2;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
			C = 1;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
			C = 0;
			return keyPadLayout[R][C];
		}
	}else if(GPIO_Pin == R2_Pin){
		R = 1;
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET);
			C = 3;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);
			C = 2;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
			C = 1;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
			C = 0;
			return keyPadLayout[R][C];
		}
	}else if(GPIO_Pin == R3_Pin){
		R = 2;
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET);
			C = 3;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);
			C = 2;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
			C = 1;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
			C = 0;
			return keyPadLayout[R][C];
		}
	}else if(GPIO_Pin == R4_Pin){
		R = 3;
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET);
			C = 3;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);
			C = 2;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
			C = 1;
			return keyPadLayout[R][C];
		}

		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
			C = 0;
			return keyPadLayout[R][C];
		}
	}
	return ' ';
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Interrupt handler for Button functionality
	if(GPIO_Pin == Button_Pin){
		play_ringtone_1_array();
	}
	if(keypadActive  != 0){
		char result = detectKey(GPIO_Pin);
		keyPadResult = &result;
		HAL_Delay(20);//deBounce
	}
}

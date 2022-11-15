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

}

void ClearKeypad(){
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
}

char getOneCharFromKeypad(){
	keyPadResult = NULL;
	int cal = -1;
	int row = -1;
	IniteKeypad();
	while(1){
		int set = HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_SET){
			row = 0;
			break;
		}
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_SET){
			row = 1;
			break;
		}
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, R3_Pin) == GPIO_PIN_SET){
			row = 2;
			break;
		}
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, R4_Pin) == GPIO_PIN_SET){
			row = 3;
			break;
		}
	}


	if(row == 0){
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_RESET){
			cal = 3;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_RESET){
			cal = 2;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_RESET){
			cal = 1;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_RESET){
			cal = 0;
			return keyPadLayout[row][cal];
		}
		return 'n';

	}else if(row = 1){
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_RESET){
			cal = 3;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_RESET){
			cal = 2;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_RESET){
			cal = 1;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_RESET){
			cal = 0;
			return keyPadLayout[row][cal];
		}
		return 'n';

	}else if(row = 2){
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, R3_Pin) == GPIO_PIN_RESET){
			cal = 3;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, R3_Pin) == GPIO_PIN_RESET){
			cal = 2;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, R3_Pin) == GPIO_PIN_RESET){
			cal = 1;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R3_GPIO_Port, R3_Pin) == GPIO_PIN_RESET){
			cal = 0;
			return keyPadLayout[row][cal];
		}
		return 'n';

	}else if(row = 3){
		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, R4_Pin) == GPIO_PIN_RESET){
			cal = 3;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, R4_Pin) == GPIO_PIN_RESET){
			cal = 2;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, R4_Pin) == GPIO_PIN_RESET){
			cal = 1;
			return keyPadLayout[row][cal];
		}
		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(R4_GPIO_Port, R4_Pin) == GPIO_PIN_RESET){
			cal = 0;
			return keyPadLayout[row][cal];
		}
		return 'n';
	}



}

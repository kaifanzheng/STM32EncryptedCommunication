/*
 * cryptosystem.h
 *
 *  Created on: Nov 10, 2022
 *      Author: m
 */

#ifndef INC_CRYPTOSYSTEM_H_
#define INC_CRYPTOSYSTEM_H_
#include "main.h"

//typedef struct _PublicMessege{
//	int publicMod;
//	int publicPower;
//}PublicMessege;


//return false if it's not coprime

int Encrypt();

int Decrypt();

//return 1 if inite sucessfully, return 0 if not
int Inite(uint32_t p, uint32_t q);


#endif /* INC_CRYPTOSYSTEM_H_ */

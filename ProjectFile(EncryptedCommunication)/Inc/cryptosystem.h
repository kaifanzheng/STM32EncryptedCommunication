/*
 * cryptosystem.h
 *
 *  Created on: Nov 15, 2022
 *      Author: nsaiy
 */

#ifndef INC_CRYPTOSYSTEM_H_
#define INC_CRYPTOSYSTEM_H_



#include "main.h"

//uint32_t p;
//uint32_t q;
//uint32_t phiN;
//uint32_t N;
//uint32_t e;
//uint32_t k;
//uint32_t privateKey;

//typedef struct _PublicMessege{
//	int publicMod;
//	int publicPower;
//}PublicMessege;


//return false if it's not coprime
uint32_t encode(uint32_t publicMode,uint32_t publicPower, uint32_t num);

uint32_t decode(uint32_t publicMode,uint32_t privatePower, uint32_t num);

//return 1 if inite sucessfully, return 0 if not
uint8_t IniteCrypto(uint32_t p, uint32_t q);

uint32_t getPublicMod();

uint32_t getPublicKey();

uint32_t getPrivateKey();

#endif /* INC_CRYPTOSYSTEM_H_ */

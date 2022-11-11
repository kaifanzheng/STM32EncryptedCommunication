/*
 * cryptosystem.c
 *
 *  Created on: Nov 10, 2022
 *      Author: m
 */

#include "cryptosystem.h"
uint32_t p;
uint32_t q;
uint32_t phi;
uint32_t e;
uint32_t k;
uint32_t privateKey;


//
bool isCoprime(uint32_t a, uint32_t b){
	for(uint32_t i = 0;i<a+1;i++){
		if(a%i == 0 && b%i == 0){
			return false;
		}
	}
	return true;
}

bool isPrime(uint32_t a){
	for(uint32_t i=2; i<a;i++){
		if(a%i == 0){
			return false;
		}
	}
	return true;
}

int Inite(uint32_t userInputP, uint32_t userInputQ){
	if(isPrime(userInputP) == false || isPrime(userInputQ) == false){
		return 0;//cannot inite
	}
	p = userInputP;
	q = userInputQ;
}

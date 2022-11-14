/*
 * cryptosystem.c
 *
 *  Created on: Nov 10, 2022
 *      Author: m
 */

#include "cryptosystem.h"
uint32_t p;
uint32_t q;
uint32_t phiN;
uint32_t N;
uint32_t e;
uint32_t k;
uint32_t privateKey;


//
int isCoprime(uint32_t a, uint32_t b){
	for(uint32_t i = 0;i<a+1;i++){
		if(a%i == 0 && b%i == 0){
			return 0;
		}
	}
	return 1;
}

int isPrime(uint32_t a){
	for(uint32_t i=2; i<a;i++){
		if(a%i == 0){
			return 0;
		}
	}
	return 1;
}

int Inite(uint32_t userInputP, uint32_t userInputQ){
	if(isPrime(userInputP) == 0 || isPrime(userInputQ) == 0){
		return 0;//cannot inite
	}
	p = userInputP;
	q = userInputQ;
	N = p*q;
	phiN = (p-1)*(q-1);
	for(uint32_t i =0;i<phiN;i++){
		if(isCoprime(i,N) && isCoprime(i,phiN)){
			e = i;
		}
	}


}

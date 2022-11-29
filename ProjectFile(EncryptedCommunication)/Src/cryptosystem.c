/*
 * cryptosystem.c
 *
 *  Created on: Nov 15, 2022
 *      Author: nsaiy
 */
#include "cryptosystem.h"
#include "asm.h"
uint32_t p;
uint32_t q;
uint32_t phiN;
uint32_t N;
uint32_t e;
uint32_t k;
uint32_t privateKey;

//
int isCoprime(uint32_t a, uint32_t b){
	for(uint32_t i = 2;i<a+1;i++){
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

uint32_t emod(uint32_t a, uint32_t  b, uint32_t  c){
    if(b == 0){
        return 1;
    }else if(b%2 == 0){
    	uint32_t  d = emod(a,b/2,c);
        return (d*d)%c;
    }else{
        return ((a%c)*emod(a,b-1,c))%c;
    }
}

uint8_t IniteCrypto(uint32_t userInputP, uint32_t userInputQ){
	if(asmisPrime(userInputP) == 0 || asmisPrime(userInputQ) == 0){
		return 0;//cannot inite
	}
	p = userInputP;
	q = userInputQ;
	N = p*q;
	phiN = (p-1)*(q-1);
	for(uint32_t i =2;i<phiN;i++){
		if(asmisCoprime(i,N) && asmisCoprime(i,phiN)){
			e = i;
			break;
		}
	}
	int findK = -1;
	for(uint32_t i = 1;i<phiN;i++){
		int find = ((i*phiN)+1)%e;
		if(find == 0){
			findK = i;
			break;
		}
	}
	if(findK == -1){
		return 0;
	}

	privateKey = ((findK*phiN)+1)/e;
	return 1;
}


uint32_t encode(uint32_t publicMode,uint32_t publicPower, uint32_t num){
	uint32_t result=  asmemod(num,publicPower,publicMode);
	return result;
}

uint32_t decode(uint32_t publicMode,uint32_t privatePower, uint32_t num){
	uint32_t result =  asmemod(num,privatePower,publicMode);
	return result;
}

uint32_t getPublicMod(){
	return p*q;
}

uint32_t getPublicKey(){
	return e;
}

uint32_t getPrivateKey(){
	return privateKey;
}

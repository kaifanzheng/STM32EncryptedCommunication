/*
 * asmisCoprime.s
 *
 *  Created on: Nov 29, 2022
 *      Author: m
 */
 //unified indicates that we're using a mix of different ARM instructions,
 //eg., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
 .syntax unified

 //.global exports the label asmMax, which is expected by lab1math.h
 .global asmisCoprime

 // .section marks a new section in assembly. .text identifies it as source code;
 // .rodata marks it as read-only , setting it to go in FLASH, not SRAM
 .section .text.rodata



/**
*int asmisCoprime(uint32_t a, uint32_t b);
*
* R0 = a
* R1 = b
*/

asmisCoprime:
MOV R2, #2 //R2=i = 2

loop:
ADD R3, R0, #1
CMP R2, R3
BGE ret1

udiv    r3, r0, R2
mul     r3, r3, R2
sub     r3, r0, r3//R3 = a%i

CMP R3, #0
BEQ check2

ADD R2, R2, #1
B loop

check2:
udiv    r3, r1, R2
mul     r3, r3, R2
sub     r3, r1, r3//R3 = b%i

CMP R3, #0
BEQ ret0

ADD R2, R2, #1
B loop

ret0:
MOV R0, #0
BX LR

ret1:
MOV R0, #1
BX LR

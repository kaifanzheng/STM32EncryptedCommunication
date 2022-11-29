/*
 * asmisPrime.s
 *
 *  Created on: Nov 29, 2022
 *      Author: m
 */
 //unified indicates that we're using a mix of different ARM instructions,
 //eg., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
 .syntax unified

 //.global exports the label asmMax, which is expected by lab1math.h
 .global asmisPrime

 // .section marks a new section in assembly. .text identifies it as source code;
 // .rodata marks it as read-only , setting it to go in FLASH, not SRAM
 .section .text.rodata



/**
* int asmisPrime(uint32_t a);
*
* R0 = a
*/

asmisPrime:
MOV R1, #2 //R1=i = 2

loop:
CMP R1, R0
BGE ret1

udiv    r3, r0, R1
mul     r3, r3, R1
sub     r3, r0, r3//R3 = a%i

CMP R3, #0
BEQ ret0

ADD R1, R1, #1
B loop


ret0:
MOV R0, #0
BX LR

ret1:
MOV R0, #1
BX LR

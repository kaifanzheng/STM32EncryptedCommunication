/*
 * asmemod.s
 *
 *  Created on: Nov 29, 2022
 *      Author: m
 */

  //unified indicates that we're using a mix of different ARM instructions,
 //eg., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
 .syntax unified

 //.global exports the label asmstd, which is expected by lab1math.h
 .global asmemod

 // .section marks a new section in assembly. .text identifies it as source code;
 // .rodata marks it as read-only , setting it to go in FLASH, not SRAM
 .section .text.rodata


 /**
* void asmemod(uint32_t a, uint32_t  b, uint32_t  c);
*
* R0 = a
* R1 = b
* R2 = c
*/


asmemod:
PUSH {R4-R7}

MOV R3, R0 //save a in R3

CMP R1, #0
BEQ ret1

MOV R4, #2 //store 2 in R4 for mod operation
udiv    r5, r1, R4
mul     r5, r5, R4
sub     r5, r1, r5//R5 is used to store b%2

CMP R5, #0
BEQ ret2

B ret3


ret1:
POP {R4-R7}
MOV R0, #1
BX LR

ret2:
MOV R4, #2
SDIV R1, R1, R4
push {LR}
BL asmemod
mul R5, R0, R0 //R3 has emod = d and r5 = d*d

udiv    r0, r5, R2
mul     r0, r0, R2
sub     r0, r5, r0//R3 = (d*d)%c

POP {LR}
POP {R4-R7}
BX LR

ret3:
SUB R1, R1, #1 //b-1
push {LR}
BL asmemod

udiv    r4, r3, R2
mul     r4, r4, R2
sub     r4, r3, r4//R4 = a%c
MUL r4, r4, r0 // (a%c)*emod

udiv    r0, r4, R2
mul     r0, r0, R2
sub     r0, r4, r0//R3 = ((a%c)*emod)%c

POP {LR}
POP {R4-R7}
BX LR










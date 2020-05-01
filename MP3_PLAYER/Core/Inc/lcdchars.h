/*
 * lcdchars.h
 *
 *  Created on: 15 kwi 2020
 *      Author: rella
 */

#ifndef INC_LCDCHARS_H_
#define INC_LCDCHARS_H_

//INVERTED CHARACTERS
//UPPERCASE
extern unsigned char A1[];
extern unsigned char B1[];
extern unsigned char C1[];
extern unsigned char D1[];
extern unsigned char E1[];
extern unsigned char F1[];
extern unsigned char G1[];
extern unsigned char H1[];
extern unsigned char I1[];
extern unsigned char J1[];
extern unsigned char K1[];
extern unsigned char L1[];
extern unsigned char M1[];
extern unsigned char N1[];
extern unsigned char O1[];
extern unsigned char P1[];
extern unsigned char R1[];
extern unsigned char S1[];
extern unsigned char T1[];
extern unsigned char U1[];
extern unsigned char W1[];
extern unsigned char X1[];
extern unsigned char Y1[];
extern unsigned char Z1[];
//LOWCASE
extern unsigned char A2[];
extern unsigned char B2[];
extern unsigned char C2[];
extern unsigned char D2[];
extern unsigned char E2[];
extern unsigned char F2[];
extern unsigned char G2[];
extern unsigned char H2[];
extern unsigned char I2[];
extern unsigned char J2[];
extern unsigned char K2[];
extern unsigned char L2[];
extern unsigned char M2[];
extern unsigned char N2[];
extern unsigned char O2[];
extern unsigned char P2[];
extern unsigned char R2[];
extern unsigned char S2[];
extern unsigned char T2[];
extern unsigned char U2[];
extern unsigned char W2[];
extern unsigned char X2[];
extern unsigned char Y2[];
extern unsigned char Z2[];
extern unsigned char downArrow[];
extern unsigned char upArrow[];
void crInvWo(char Loc, unsigned char Symbol[], char memoryLoc, int v);
void createInvertedWord(char Word[], char Loc, int Length, char mLoc);
#endif /* INC_LCDCHARS_H_ */

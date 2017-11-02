/* Copyright (c) 2015 Takafumi Naka. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef LEDSPATTERNS_H__
#define LEDSPATTERNS_H__

#include	"ws2812b_drive.h"
#include 	"stdlib.h"

#define OFFLEDS '0'
#define ONLEDS '1'
#define FADE '2'
#define CYCLON '3'
#define FLASH '4'
#define FLASHFADE '5'
#define WIPE '6'
#define RING '7'
 
 
#define DELAYFADE 1
#define DELAYCYCLON 80
#define DELAYFLASH 10
#define DELAYFLASHFADE 1
#define DELAYWIPE 5
#define DELAYBOUNCE 10
#define DELAYRING 250
#define DELAYRINGBOUNCE 350



void OffLeds(void);
void SetAll(uint8_t red, uint8_t green, uint8_t blue);
void SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
uint8_t	FadeInOut(void);
uint8_t Cyclon(void);
uint8_t Flash(void);
uint8_t FlashFadeInOut(void);
uint16_t Wipe(void);
uint16_t Ring(void);

unsigned short random_rand(void);
void random_init(unsigned short seed);

#endif // LEDSPATTERNS_H


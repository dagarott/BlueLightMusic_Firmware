/* Copyright (c) 2016 Takafumi Naka. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */


#include	"LedPatterns.h"
#include	"ws2812b_drive.h"
#include	"project.h"

#define RANDOM_BUFF_SIZE 16                                                           /**< Random numbers buffer size. */


rgb_led_t led_array[NUM_LEDS];
//uint16_t i=0;
//uint16_t j=0;
//uint16_t k=255;

void SetAll(uint8_t red, uint8_t green, uint8_t blue)
{
	    led_array[0].red=red;
        led_array[0].green=green;
        led_array[0].blue=blue;
        led_array[1].red=red;
        led_array[1].green=green;
        led_array[1].blue=blue;
        led_array[2].red=red;
        led_array[2].green=green;
        led_array[2].blue=blue;


}

void SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue)
{

	 	led_array[pixel].red=red;
        led_array[pixel].green=green;
        led_array[pixel].blue=blue;
}

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff                               Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length                               Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
uint8_t FadeInOut(void)
{
	//static uint16_t i=0,j=0, k=255;
	static uint16_t index_led=0;
	static uint16_t up_value=0;
	static uint16_t down_value=255;

	if(index_led<3)
	{
		
		if(up_value<256)
		{
			
			switch(index_led)
			{
				case 0: SetAll((uint8_t)up_value,0,0); break;
                case 1: SetAll(0,(uint8_t)up_value,0); break;
				case 2: SetAll(0,0,(uint8_t)up_value); break;
			}
			up_value++;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(1);

		}
		else if(down_value>0)
		{
			switch(index_led)
			{
				case 0: SetAll((uint8_t)down_value,0,0); break;
                case 1: SetAll(0,(uint8_t)down_value,0); break;
				case 2: SetAll(0,0,(uint8_t)down_value); break;
			}
			down_value--;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(1);
		}
		down_value=255;
		up_value=0;
		index_led++;

	}
	else index_led=0;
}

uint8_t Cyclon(void)
{
	
	static uint16_t index_led0=0,index_led1=2;
	const uint8_t red=127;
	const uint8_t blue=127;
	const uint8_t green=127;

	SetAll(0,0,0); 

	if(index_led0<3)
	{
		SetPixel(index_led0, red, blue, 0);
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		index_led0++;
		return(80);
	}
	else if (index_led1>0)
	{
		
		SetPixel(index_led1, red, 0, green);
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		index_led1--;
		return(80);
	}
	else
	{		
		index_led0=0;
		index_led1=2;
	}
}
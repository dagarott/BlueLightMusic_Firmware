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
#include <stdio.h>
#include <stdlib.h>

#define RANDOM_BUFF_SIZE 16                                                           /**< Random numbers buffer size. */


rgb_led_t led_array[NUM_LEDS];


uint8_t random(void)
{
	uint32_t err_code;
	//uint8_t * random_number;
	uint8_t random_number;
	uint8_t num_rand_bytes_available;
	
	//random_number=random_vector;
	
	err_code = sd_rand_application_bytes_available_get(&num_rand_bytes_available);
	APP_ERROR_CHECK(err_code);
	//if there is a randon number available
	if(num_rand_bytes_available > 0)
	{
          err_code = sd_rand_application_vector_get(&random_number, 1);
          APP_ERROR_CHECK(err_code);
					return(random_number);
	
	}
}
	 
	 
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


uint8_t Flash(void)
{
	
	static uint16_t index_led=0,num_flashes=0;
	const uint8_t first_red=10;
	const uint8_t first_blue=25;
	const uint8_t first_green=127;
	const uint8_t second_red=0;
	const uint8_t second_blue=255;
	const uint8_t second_green=255;
	static uint8_t tmp_red=first_red;
	static uint8_t tmp_blue=first_blue;
	static uint8_t tmp_green=first_green;
	
	
	SetAll(0,0,0); 

	if(index_led<3)
	{
		SetPixel(index_led, tmp_red,tmp_blue, tmp_green );
		
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		
		tmp_red=first_red+second_red-tmp_red;
		tmp_blue=first_blue+second_blue-tmp_blue;
		tmp_green=first_green+second_green-tmp_green;
		
		if(num_flashes>3)
		{		
			index_led++;
			num_flashes=0;
		}
		else
			num_flashes++;
			
		return(10);
	}
	else 
		index_led=0;
		//index_led=random()%3;
	
}

uint8_t Randome(void)
{
	
	static uint16_t index_led=0;
	const uint8_t first_red=10;
	const uint8_t first_blue=25;
	const uint8_t first_green=127;
//	const uint8_t second_red=0;
//	const uint8_t second_blue=255;
//	const uint8_t second_green=255;
	static uint8_t tmp_red=first_red;
	static uint8_t tmp_blue=first_blue;
	static uint8_t tmp_green=first_green;
	
//	static uint16_t tmp=0;


	SetAll(0,0,0); 

	if(index_led<3)
	{
		SetPixel(index_led, tmp_red,tmp_blue, tmp_green );
		
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		
	//	tmp_red=first_red+second_red-tmp_red;
	//	tmp_blue=first_blue+second_blue-tmp_blue;
	//	tmp_green=first_green+second_green-tmp_green;
		
	//	if(num_flashes>3)
	//	{		
	//		index_led++;
	//		num_flashes=0;
	//	}
	//	else
	//		num_flashes++;
			
		return(5);
	}
	else 
		index_led=random()%3;
	
}


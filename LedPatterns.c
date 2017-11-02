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
#include "arm_math.h"
#include <nrf_drv_rng.h>
#include "app_error.h"

#define RANDOM_BUFF_SIZE 1                                                          /**< Random numbers buffer size. */ 


rgb_led_t led_array[NUM_LEDS];

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff                               Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length                               Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
unsigned short random_vector_generate(void)
{
  unsigned short value = 42;
  uint8_t available;
  ret_code_t err_code;

  do {
    nrf_drv_rng_bytes_available(&available);
  } while (available < sizeof(value));

  err_code = nrf_drv_rng_rand((uint8_t *)&value, sizeof(value));
  APP_ERROR_CHECK(err_code);

  return value;
}


static volatile bool rng_done;
static volatile uint8_t *rng_ptr;
static volatile uint32_t rng_len;

void RNG_IRQHandler(void){
	NRF_RNG->EVENTS_VALRDY = 0;	
	(void) NRF_RNG->EVENTS_VALRDY;
	if (rng_len){
		*rng_ptr = NRF_RNG->VALUE;
		rng_ptr++;
		rng_len--;
	}else{
		rng_done = true;
	}		
}

void rng_get_bytes(void){
/* 	rng_done = false;
	rng_ptr = rng_buf_ptr;
	rng_len = rng_buf_len;

	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled;
	NRF_RNG->INTENSET = (RNG_INTENSET_VALRDY_Set << RNG_INTENSET_VALRDY_Pos);
	NRF_RNG->EVENTS_VALRDY = 0;

	NVIC_SetPriority(RNG_IRQn, PRIORITY_RNG_IRQn);
	NVIC_EnableIRQ(RNG_IRQn);	

	NRF_RNG->TASKS_START = 1;
	while (false == rng_done){
		__WFI();
	}
	NRF_RNG->TASKS_STOP = 1;

	NRF_RNG->INTENCLR = (RNG_INTENCLR_VALRDY_Clear << RNG_INTENCLR_VALRDY_Pos);
	NVIC_DisableIRQ(RNG_IRQn);	 */
	
	
	
	uint8_t key[16];
	uint8_t random_values_length, rand_values, generated;
	
	
	//try to generate the key randomly
	sd_rand_application_pool_capacity_get(&random_values_length);
	//we need only 16 bytes
	if(random_values_length>16)
		random_values_length=16;
	//wait until values are generated
	do
		sd_rand_application_bytes_available_get(&generated);
	while(generated<random_values_length);
	//get the random data
	sd_rand_application_vector_get(key, random_values_length);
	//if random values are less than 16 add static data to fill the buffer
	if(random_values_length<16)
		for(int i=random_values_length; i<16; i++)
			key[i]=i;
}






/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the nRF52 random number generator.
 * \param seed Ignored. It's here because the function prototype is in core.
 *
 */
void
random_init(unsigned short seed)
{
  (void)seed;
  ret_code_t err_code = nrf_drv_rng_init(NULL);
  APP_ERROR_CHECK(err_code);
}

void OffLeds(void)
{
	SetAll(0,0,0);
	i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
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
	static uint16_t index_color=0;
	static uint16_t up_value=0;
	static uint16_t down_value=255;

	if(index_color<3)
	{

		if(up_value<256)
		{

			switch(index_color)
			{
			case 0:
				SetAll((uint8_t)up_value,0,0);
				break;
			case 1:
				SetAll(0,(uint8_t)up_value,0);
				break;
			case 2:
				SetAll(0,0,(uint8_t)up_value);
				break;
			}
			up_value++;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYFADE);

		}
		else if(down_value>0)
		{
			switch(index_color)
			{
			case 0:
				SetAll((uint8_t)down_value,0,0);
				break;
			case 1:
				SetAll(0,(uint8_t)down_value,0);
				break;
			case 2:
				SetAll(0,0,(uint8_t)down_value);
				break;
			}
			down_value--;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYFADE);
		}
		down_value=255;
		up_value=0;
		index_color++;

	}
	else index_color=0;
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
		return(DELAYCYCLON);
	}
	else if (index_led1>0)
	{

		SetPixel(index_led1, red, 0, green);
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		index_led1--;
		return(DELAYCYCLON);
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

		return(DELAYFLASH);
	}
	else
		index_led=0;
}

uint8_t FlashFadeInOut(void)
{

	static uint16_t index_led=0;
	static uint16_t up_value=0;
	static uint16_t down_value=255;
	const uint8_t first_red=255;
	const uint8_t first_blue=0;
	const uint8_t first_green=0;
	const uint8_t second_red=0;
	const uint8_t second_blue=0;
	const uint8_t second_green=255;
	static uint8_t tmp_red=first_red;
	static uint8_t tmp_blue=first_blue;
	static uint8_t tmp_green=first_green;


	if(index_led<3)
	{

		if(up_value<256)
		{
			switch(index_led)
			{
			case 0:
			{
				SetPixel((index_led%3),(uint8_t)up_value,0, 0);
				SetPixel(((index_led+1)%3),(uint8_t)up_value,0,0);
				SetPixel(((index_led+2)%3),tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}
			case 1:
			{
				SetPixel((index_led%3),0,(uint8_t)up_value,0);
				SetPixel(((index_led+1)%3),0,(uint8_t)up_value,0);
				SetPixel(((index_led+2)%3),tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}
			case 2:
			{
				SetPixel((index_led%3),0,0,(uint8_t)up_value);
				SetPixel(((index_led+1)%3),0,0,(uint8_t)up_value);
				SetPixel(((index_led+2)%3),tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}
			}
			up_value++;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYFLASHFADE);

		}
		else if(down_value>0)
		{
			switch(index_led)
			{
			case 0:
			{
				SetPixel((index_led%3),(uint8_t)down_value,0, 0);
				SetPixel(((index_led+1)%3),(uint8_t)down_value,0,0);
				SetPixel(((index_led+2)%3),tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}
			case 1:
			{

				SetPixel((index_led%3),0, (uint8_t)down_value,0);
				SetPixel(((index_led+1)%3),0,(uint8_t)down_value,0);
				SetPixel(((index_led+2)%3),tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}
			case 2:
			{
				SetPixel((index_led%3),0,0,(uint8_t)down_value);
				SetPixel(((index_led+1)%3),0,0,(uint8_t)down_value);
				SetPixel(((index_led+2)%3), tmp_red,tmp_blue, tmp_green );
				//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				tmp_red=first_red+second_red-tmp_red;
				tmp_blue=first_blue+second_blue-tmp_blue;
				tmp_green=first_green+second_green-tmp_green;
				break;
			}

			}
			down_value--;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYFLASHFADE);


		}
		down_value=255;
		up_value=0;
		index_led++;
	}
	else index_led=0;
}

uint16_t Wipe(void)
{

	//static uint16_t i=0,j=0, k=255;
	static uint16_t index=0;
	static uint16_t up_value=0;
	static uint16_t down_value=256;
	static uint16_t count_debounce=0;
	

	

		if((up_value<256) && (down_value>0))
		{

			switch(index)
			{
			case 0:
				SetPixel(0,up_value,0,0);
				SetPixel(1,0,down_value,0);
				SetPixel(2,0,0,up_value);
				break;
			case 1:
				SetPixel(0,0,down_value,0);
				SetPixel(1,0,0,up_value);
				SetPixel(2,down_value,0,0);		
				break;
			case 2:
				SetPixel(0,0,0,up_value);
				SetPixel(1,down_value,0,0);
				SetPixel(2,0,down_value,0);	
				break;
			}
			up_value++;
			down_value--;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYWIPE);

		}
		else
		{
			count_debounce++;
			if(count_debounce==DELAYBOUNCE)
			{
				count_debounce=0;
				down_value=256;
				up_value=0;
				index++;
				if(index==3)
					index=0;
			}
			
		}
}

uint16_t Ring(void)
{
	static uint16_t index0=0, index1=3,index2=0;
	const uint8_t first_red=255;
	const uint8_t first_blue=0;
	const uint8_t first_green=0;
	const uint8_t second_red=0;
	const uint8_t second_blue=0;
	const uint8_t second_green=255;
		
		
	if(index0<3)	
	{
		switch(index0)
			{
			case 0:
				SetPixel(0,0,first_red,0);
				SetPixel(1,0,0,0);
				SetPixel(2,0,0,0);
				break;
			case 1:
				SetPixel(0,0,first_red,0);
				SetPixel(1,0,first_red,0);
				SetPixel(2,0,0,0);
				break;
			case 2:
				SetPixel(0,0,first_red,0);
				SetPixel(1,0,first_red,0);
				SetPixel(2,0, first_red,0);
				break;
			}
			index0++;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYRING);		
	}
	else if(index1>0)	
	{
		switch(index1)
			{
			case 1:
				SetPixel(0,0,0,first_red);
				SetPixel(1,0,0,0);
				SetPixel(2,0,0,0);
				break;
			case 2:
				SetPixel(0,0,0,first_red);
				SetPixel(1,0,0,first_red);
				SetPixel(2,0,0,0);
				break;
			case 3:
				SetPixel(0,0,0,first_red);
				SetPixel(1,0,0,first_red);
				SetPixel(2,0, 0,first_red);
				break;
			}
			index1--;
			i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
			return(DELAYRING);		
	}
	else if (index2<3)
	{
		switch(index2)
				{
				case 0:
					SetPixel(0,first_red,0,0);
					SetPixel(1,0,0,0);
					SetPixel(2,0,0,0);
					break;
				case 1:
					SetPixel(0,first_red,0,0);
					SetPixel(1,first_red,0,0);
					SetPixel(2,0,0,0);
					break;
				case 2:
					SetPixel(0,first_red,0,0);
					SetPixel(1,first_red,0,0);
					SetPixel(2,first_red,0,0);
					break;
				}
				index2++;
				i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
				return(DELAYRING);	
	}			
	else 
	{
		index0=0;
		index1=3;
		index2=0;
		SetAll(127,127,127);
		i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);
		return(DELAYRINGBOUNCE);	
	}
	
}

/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup	ble_sdk_app_nus_eval
 * @brief		UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

//debug

#include "arm_math.h"
#include "nrf_delay.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "app_pwm.h"
#include "stdlib.h"
#include "project.h"
#include "ws2812b_drive.h"
#include "i2s_ws2812b_drive.h"
#include "drv_speaker.h"
#include "sounds.h "
#include "notes.h "
#include "LedPatterns.h "
#include "softdevice_handler.h"
//debug

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0																					 /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT							0																					 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT					 1																					 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME										 "Nordic_UART"															 /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE					 BLE_UUID_TYPE_VENDOR_BEGIN									/**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL								64																					/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS			180																				 /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER						 0																					 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE				 4																					 /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL							 MSEC_TO_UNITS(20, UNIT_1_25_MS)						 /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL							 MSEC_TO_UNITS(75, UNIT_1_25_MS)						 /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY									 0																					 /**< Slave latency. */
#define CONN_SUP_TIMEOUT								MSEC_TO_UNITS(4000, UNIT_10_MS)						 /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)	/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY	 APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT		3																					 /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF											 0xDEADBEEF																	/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE								256																				 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE								256																				 /**< UART RX buffer size. */

static ble_nus_t												m_nus;																			/**< Structure to identify the Nordic UART Service. */
static uint16_t												 m_conn_handle = BLE_CONN_HANDLE_INVALID;		/**< Handle of the current connection. */

static ble_uuid_t											 m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};	/**< Universally unique service identifier. */


//debug
#define DEFAULT_VOLUME 100

APP_TIMER_DEF(systick_timer_id);
#define SYSTICK_INTERVAL         APP_TIMER_TICKS(1, APP_TIMER_PRESCALER) /**< SYSTICK interval (ticks). This value corresponds to 1 seconds. */


uint8_t ws2812bPattern=RING;
uint16_t DelayWS2812b=0;
uint8_t DelayBuzzer=0;
uint8_t flag_off_leds=false;




//APP_PWM_INSTANCE(PWM1,1);									 // Create the instance "PWM1" using TIMER1.

//static volatile bool ready_flag;						// A flag indicating PWM status.
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_pwm_values_common_t m_seq_values;

void update_pwm(int16_t duty_cycle);
//debug

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *					how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num		Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *					the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
	uint32_t								err_code;
	ble_gap_conn_params_t	 gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
	                                      (const uint8_t *) DEVICE_NAME,
	                                      strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency		 = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout	= CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *					it to the UART module.
 *
 * @param[in] p_nus		Nordic UART Service structure.
 * @param[in] p_data	 Data to be send to UART module.
 * @param[in] length	 Length of the data.
 */
/**@snippet [Handling the data received over BLE] */


static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{

//		for (uint32_t i = 0; i < length; i++)
//		{
//				while(app_uart_put(p_data[i]) != NRF_SUCCESS);
//		}
//		while(app_uart_put('\n') != NRF_SUCCESS);

	switch(p_data[0])
	{
	case FADE:
		ws2812bPattern=FADE;
		break;
	case CYCLON:
		ws2812bPattern=CYCLON;
		break;
	case FLASH:
		ws2812bPattern=FLASH;
		break;
	case FLASHFADE:
		ws2812bPattern=FLASHFADE;
		break;
	case WIPE:
		ws2812bPattern=WIPE;
		break;
	case RING:
		ws2812bPattern=RING;
		break;
	case OFFLEDS:
		flag_off_leds=true;
		break;
	case ONLEDS:
		flag_off_leds=false;
		break;

	}
	//Update Leds
	//i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, I2S_STDO_PIN);

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t			 err_code;
	ble_nus_init_t nus_init;

	memset(&nus_init, 0, sizeof(nus_init));

	nus_init.data_handler = nus_data_handler;

	err_code = ble_nus_init(&m_nus, &nus_init);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *					which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *			 the disconnect_on_fail config parameter, but instead we use the event handler
 *			 mechanism to demonstrate its use.
 *
 * @param[in] p_evt	Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	uint32_t err_code;

	if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error	Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	uint32_t							 err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params									= NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay	= NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count	 = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle		= BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail						 = false;
	cp_init.evt_handler										= on_conn_params_evt;
	cp_init.error_handler									= conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
	APP_ERROR_CHECK(err_code);

	// Prepare wakeup buttons.
	err_code = bsp_btn_ble_sleep_mode_prepare();
	APP_ERROR_CHECK(err_code);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt	Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	uint32_t err_code;

	switch (ble_adv_evt)
	{
	case BLE_ADV_EVT_FAST:
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
		break;
	case BLE_ADV_EVT_IDLE:
		sleep_mode_enter();
		break;
	default:
		break;
	}
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t												 err_code;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		err_code = bsp_indication_set(BSP_INDICATE_IDLE);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// Pairing not supported
		err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		// No implementation needed.
		break;
	}
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *				event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *					SoftDevice event has been received.
 *
 * @param[in] p_ble_evt	SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	ble_conn_params_on_ble_evt(p_ble_evt);
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_advertising_on_ble_evt(p_ble_evt);
	bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	uint32_t err_code;

	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	// Initialize SoftDevice.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
	           PERIPHERAL_LINK_COUNT,
	           &ble_enable_params);
	APP_ERROR_CHECK(err_code);

	//Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
	// Enable BLE stack.
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Subscribe for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]	 event	 Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
	uint32_t err_code;
	switch (event)
	{
	case BSP_EVENT_SLEEP:
		sleep_mode_enter();
		break;

	case BSP_EVENT_DISCONNECT:
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BSP_EVENT_WHITELIST_OFF:
		err_code = ble_advertising_restart_without_whitelist();
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
		break;

	default:
		break;
	}
}


/**@brief	 Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *					a string. The string will be be sent over BLE when the last character received was a
 *					'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of
 *					@ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
	static uint8_t index = 0;
	uint32_t			 err_code;

	switch (p_event->evt_type)
	{
	case APP_UART_DATA_READY:
		UNUSED_VARIABLE(app_uart_get(&data_array[index]));
		index++;

		if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
		{
			err_code = ble_nus_string_send(&m_nus, data_array, index);
			if (err_code != NRF_ERROR_INVALID_STATE)
			{
				APP_ERROR_CHECK(err_code);
			}

			index = 0;
		}
		break;

	case APP_UART_COMMUNICATION_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_communication);
		break;

	case APP_UART_FIFO_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_code);
		break;

	default:
		break;
	}
}
/**@snippet [Handling the data received over UART] */


/**@brief	Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
	uint32_t										 err_code;
	const app_uart_comm_params_t comm_params =
	{
		RX_PIN_NUMBER,
		TX_PIN_NUMBER,
		RTS_PIN_NUMBER,
		CTS_PIN_NUMBER,
		APP_UART_FLOW_CONTROL_ENABLED,
		false,
		UART_BAUDRATE_BAUDRATE_Baud115200
	};

	APP_UART_FIFO_INIT( &comm_params,
	                    UART_RX_BUF_SIZE,
	                    UART_TX_BUF_SIZE,
	                    uart_event_handle,
	                    APP_IRQ_PRIORITY_LOW,
	                    err_code);
	APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t			err_code;
	ble_advdata_t advdata;
	ble_advdata_t scanrsp;

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type					= BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance = false;
	advdata.flags							= BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids	= m_adv_uuids;

	ble_adv_modes_config_t options = {0};
	options.ble_adv_fast_enabled	= BLE_ADV_FAST_ENABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout	= APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

//debug
void pwm_ready_callback(uint32_t pwm_id)		// PWM callback function
{
	//ready_flag = true;
}

nrf_pwm_sequence_t const m_LED_channel_seq =
{
	//.values.p_individual = &m_LED_seq_values,
	.values.p_common = &m_seq_values,
	//.length					= NRF_PWM_VALUES_LENGTH(m_LED_seq_values),
	.length					= NRF_PWM_VALUES_LENGTH(m_seq_values),
	.repeats				 = 0,
	.end_delay			 = 0
};
/*
static void pwm_simple(void)
{
	uint32_t err_code;
	nrf_drv_pwm_config_t const config0 =
	{
			.output_pins =
			{
					2	, // channel 0
					NRF_DRV_PWM_PIN_NOT_USED,						 // channel 1
					NRF_DRV_PWM_PIN_NOT_USED,						 // channel 2
					NRF_DRV_PWM_PIN_NOT_USED,						 // channel 3
			},
			.irq_priority = APP_IRQ_PRIORITY_LOW,
			.base_clock	 = NRF_PWM_CLK_16MHz,
			//.base_clock	 = NRF_PWM_CLK_125kHz,
			.count_mode	 = NRF_PWM_MODE_UP,
			//.top_value		= 1000,
			//.top_value		= 800,	//20KHz
			.top_value		= 32000,	//4KHz
			.load_mode		= NRF_PWM_LOAD_COMMON,
			//.load_mode	= NRF_PWM_LOAD_INDIVIDUAL,
			.step_mode		= NRF_PWM_STEP_AUTO
	};

	err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
	if (err_code != NRF_SUCCESS)
	{
			// Initialization failed. Take recovery action.
	}

}*/

static void pwm_simple(uint16_t freq)
{
	uint32_t err_code;
	nrf_drv_pwm_config_t const config0 =
	{
		.output_pins =
		{
			2	, // channel 0
			NRF_DRV_PWM_PIN_NOT_USED,						 // channel 1
			NRF_DRV_PWM_PIN_NOT_USED,						 // channel 2
			NRF_DRV_PWM_PIN_NOT_USED,						 // channel 3
		},
		.irq_priority = APP_IRQ_PRIORITY_LOW,
		.base_clock	 = NRF_PWM_CLK_1MHz,
		//.base_clock	 = NRF_PWM_CLK_125kHz,
		.count_mode	 = NRF_PWM_MODE_UP,
		//.top_value		= 1000,
		//.top_value		= 800,	//20KHz
		.top_value		= freq,	//4KHz
		.load_mode		= NRF_PWM_LOAD_COMMON,
		//.load_mode	= NRF_PWM_LOAD_INDIVIDUAL,
		.step_mode		= NRF_PWM_STEP_AUTO
	};

	err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
	if (err_code != NRF_SUCCESS)
	{
		// Initialization failed. Take recovery action.
	}

}

void update_pwm(int16_t duty_cycle)
{
	m_seq_values = duty_cycle;
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_LED_channel_seq, 1, 0);
}

static void drv_speaker_evt_handler(drv_speaker_evt_t evt)
{
	switch(evt)
	{
	case DRV_SPEAKER_EVT_FINISHED:
	{
		//DEBUG_PRINTF(0, "drv_speaker_evt_handler: DRV_SPEAKER_EVT_FINISHED\r\n");
		//(void)ble_tss_spkr_stat_set(&m_tss, BLE_TSS_SPKR_STAT_FINISHED);
	}
	break;
	//
	case DRV_SPEAKER_EVT_BUFFER_WARNING:
	{
		//DEBUG_PRINTF(0, "drv_speaker_evt_handler: DRV_SPEAKER_EVT_BUFFER_WARNING\r\n");
		//(void)ble_tss_spkr_stat_set(&m_tss, BLE_TSS_SPKR_STAT_BUFFER_WARNING);
	}
	break;
	//
	case DRV_SPEAKER_EVT_BUFFER_READY:
	{
		//DEBUG_PRINTF(0, "drv_speaker_evt_handler: DRV_SPEAKER_EVT_BUFFER_READY\r\n");
		//(void)ble_tss_spkr_stat_set(&m_tss, BLE_TSS_SPKR_STAT_BUFFER_READY);
	}
	break;
	//
	default:
		APP_ERROR_CHECK_BOOL(false);
		break;
	}
}

/**@brief Function for handling the systick timer timeout.
 *
 * @details This function will be called each time the systick timer expires.
 *          This function will start update_leds(), update_buzzer(), update_motor(),
 *					get_pin_status() and update_power_state() functions.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void systick_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
//    static uint8_t DelayWS2812b=0;
//    static uint8_t DelayBuzzer=0;
	if(!flag_off_leds)
	{
		if(DelayWS2812b==0)
		{

			switch(ws2812bPattern)
			{

			case FADE:
			{
				DelayWS2812b=FadeInOut();
				break;
			}
			case CYCLON:
			{
				DelayWS2812b=Cyclon();
				break;
			}
			case FLASH:
			{
				DelayWS2812b=Flash();
				break;
			}
			case FLASHFADE:
			{
				DelayWS2812b=FlashFadeInOut();
				break;
			}
			case WIPE:
			{
				DelayWS2812b=Wipe();
				break;
			}
			
			case RING:
			{
				DelayWS2812b=Ring();
				break;
			}

			default:
				break;
			}
		}
		else
			DelayWS2812b--;
	}
	else if(flag_off_leds)
		OffLeds();

}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timer_init(void)
{
	uint32_t err_code;

	// Initialize timer module.
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

	// Create battery timer.
	err_code = app_timer_create(&systick_timer_id,
	                            APP_TIMER_MODE_REPEATED,
	                            systick_timeout_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer starting.
 *
 * @details Start the timer module. This starts application timers.
 */
static void timer_start(void)
{
	uint32_t err_code;
	// Start battery timer
	err_code = app_timer_start(systick_timer_id, SYSTICK_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}
//debug

/**@brief Application main function.
 */
int main(void)
{
	uint32_t err_code;
	drv_speaker_init_t speaker_init;
	speaker_init.evt_handler = drv_speaker_evt_handler;
	int note=1;

	// Initialize.
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	uart_init();
	ble_stack_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();
	//drv_speaker_init(&speaker_init);
	timer_init();


	printf("\r\nUART Start!\r\n");
	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);


	//debug
//	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
//  SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
//	err_code = nrf_drv_rng_init(NULL);
//    APP_ERROR_CHECK(err_code);
	timer_start();
	//debug


	// Enter main loop.
	for (;;)
	{
		//power_manage();
		//for (int j=1000; j<4000; (j=j+500)) {

		//drv_speaker_sample_play(j);
		//note=starwars[j];
		//drv_speaker_tone_start(j,350 , 50);
		//drv_speaker_tone_start(2000, 500 , 100);
		//nrf_delay_ms(100);
	}

}
/**
 * @}
 */

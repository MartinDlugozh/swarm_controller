/**
 * Параметры, хранимые в постоянной памяти EEPROM.
 * Порядок заполнения:
 * 1. Значение ONBOARD_PARAM_COUNT увеличить в сответствии с количеством
 * добавляемых параметров;
 * 2. Дописать имена параметров в массив const char *param_names[ONBOARD_PARAM_COUNT];
 * 3. Дописать в массив uint8_t	param_values[ONBOARD_PARAM_COUNT] значение инициализации
 * для добавляемых параметров (рекомендуется заполнять нулями);
 * 4. Для добавляемых параметров добавить дефайны для связи имен и значений (для удобочитаемости)
 * #define name_n param_values[n]
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <EEPROM.h>

#define ONBOARD_PARAM_COUNT 9

/**
 * Parameter names
 */
const char 		*param_names[ONBOARD_PARAM_COUNT] = {
				"OFFS_X",			// [0]
				"OFFS_Y",			// [1]
				"OFFS_Z",			// [2]
				"INIT_POS",			// [3]
				"DBG_TXT",			// [4]
				"TKOFF_ALT",		// [5]
				"FOLL_NUM",			// [6]
				"FOLL_ID_1",		// [7]
				"FOLL_ID_2"};		// [8]

/**
 * Parameter values
 * WARNING: all of the parameters have a type of uint8_t (unsigned char),
 * but receiving from and transmitting to the GCS is using float data type
 */
uint8_t			param_values[ONBOARD_PARAM_COUNT] = {
				0,					// [0]
				0,					// [1]
				0,					// [2]
				0,					// [3]
				1,					// [4]
				2,					// [5]
				2,					// [6]
				2,					// [7]
				3};					// [8]

/**
 * Definitions
 */
#define 		OFFS_X_p 		param_values[0]
#define 		OFFS_Y_p 		param_values[1]
#define 		OFFS_Z_p 		param_values[2]
#define 		INIT_POS_p		param_values[3]
#define 		DBG_TXT_p		param_values[4]
#define 		TKOFF_ALT_p 	param_values[5]
#define 		FOLL_NUM_p 		param_values[6]
#define 		FOLL_ID_1_p		param_values[7]
#define 		FOLL_ID_2_p		param_values[8]

/**
 * Load all parameters from EEPROM
 */
void param_load_all()
{
	for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
	{
		param_values[i] = EEPROM.read(i);
	}
}

/**
 * Save all parameters to EEPROM
 */
void param_save_all()
{
	for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
	{
		EEPROM.write(i, param_values[i]);
	}
}

/**
 * Send single parameter by param_index
 */
void param_send(uint8_t param_index, HardwareSerial &uart)
{
	mavlink_message_t p_msg;

	mavlink_msg_param_value_pack(SYS_ID_MY, COMP_ID_MY, &p_msg,
			param_names[param_index], (float)param_values[param_index],
			MAV_PARAM_TYPE_REAL32, (uint16_t)ONBOARD_PARAM_COUNT, (uint16_t)param_index);
	send_message(&p_msg, uart);
}

/**
 * Handle MAVLink message PARAM_SET
 * Save received parameter to the EEPROM and replace value of param_values[i]
 */
void param_handle_set(mavlink_param_set_t param, HardwareSerial &uart)
{
	for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
	{
		if(strcmp(param_names[i], param.param_id) == 0)
		{
			param_values[i] = (uint8_t)param.param_value;
			EEPROM.write(i, param_values[i]);

			param_send(i, uart);

			if(DBG_TXT_p == 1)
			{
				statustext_send("PARAM_SAVED", uart);
			}
			break;
		}
	}
}

/**
 * Send all parameters when requested
 */
void param_handle_req_list(HardwareSerial &uart)
{
	for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
	{
		param_send(i, uart);
	}
}

/**
 * Send single parameter when it is requested
 */
void param_handle_req_value(mavlink_param_request_read_t rq, HardwareSerial &uart)
{
	if(rq.param_index == (-1))
	{
		for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
			{
				if(strcmp(param_names[i], rq.param_id) == 0)
				{
					param_send(i, uart);
					break;
				}
			}
	}else
	{
		param_send((uint8_t)rq.param_index, uart);
	}
}

#endif /* PARAMETERS_H_ */

/**
 * Параметры, хранимые в постоянной памяти EEPROM
 * ----------------------------------------------
 *
 * Значения параметров хранятся в постоянной изменяемой памяти МК (EEPROM),
 * что обеспечивает возможностью их перезаписи в процессе работы МК.
 * Значения имен параметров хранятся в постоянной неизменяемой памяти МК (Flash).
 *
 * Порядок заполнения:
 * -------------------
 * 1. Значение ONBOARD_PARAM_COUNT увеличить в сответствии с количеством
 * добавляемых параметров;
 * 2. Добавить новое определение строкового литерала для хранения его в программной памяти:
 * 		const char param_n[] PROGMEM = "PARAM_NAME";
 * 3. Добавить определенный выше литерал в таблицу имен параметров const char* const param_table[] PROGMEM = {...}
 * 4. Дописать в массив uint8_t	param_values[ONBOARD_PARAM_COUNT] значение инициализации
 * для добавляемых параметров (рекомендуется заполнять нулями);
 * 5. Для добавляемых параметров добавить дефайны для связи имен и значений (для удобочитаемости)
 * #define name_n_p param_values[n]
 */

#ifndef PARAM_HELPER_H_
#define PARAM_HELPER_H_

#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "parameters.h"

#define PARAM_NAME_LEN 			16

/**
 * Load parameter names from PROGMEM
 */
char param_name_buffer[PARAM_NAME_LEN] = {0};  // global name buffer used in char *get_by_index(uint8_t param_index)

char *get_by_index(uint8_t param_index)
{
	memset(param_name_buffer, 0, PARAM_NAME_LEN);
	return strcpy_P(param_name_buffer, (char*)pgm_read_word(&(param_table[param_index])));
}

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

	mavlink_msg_param_value_pack(MY_SYS_ID, MY_COMP_ID, &p_msg,
			get_by_index(param_index)
			, (float)param_values[param_index],
			MAV_PARAM_TYPE_REAL32, (uint16_t)ONBOARD_PARAM_COUNT, (uint16_t)param_index);
	send_message(&p_msg, uart);
}

/**
 * Handle MAVLink message PARAM_SET
 * Save received parameter to the EEPROM and replace value of param_values[i]
 */
void param_handle_set(mavlink_param_set_t &parameter, HardwareSerial &uart)
{
	for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
	{
		if(strcmp(get_by_index(i), parameter.param_id) == 0)
		{
			param_values[i] = (uint8_t)parameter.param_value;
			EEPROM.write(i, param_values[i]);

			param_send(i, uart);

			if(DBG_TXT_p == 1)
			{
				statustext_send("PS", uart);
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
void param_handle_req_value(mavlink_param_request_read_t &rq, HardwareSerial &uart)
{
	if(rq.param_index == (-1))
	{
		for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
			{
				if(strcmp(get_by_index(i), rq.param_id) == 0)
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

#endif /* PARAM_HELPER_H_ */

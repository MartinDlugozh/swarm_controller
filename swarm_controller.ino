/**
 * Программа для реализации режима совместного полета (РСП) нескольких коптеров
 *
 * LED Pattern:
 * R, G, B - initialization
 * R - initialized, no link with Pixhawk
 * G - initialized, link is good (heartbeat is OK)
 * R, B or G, B - initialized, arm/takeoff stage
 */

#define MAVLINK_COMM_NUM_CHANNELS 1
#define MAVLINK_COMM_NUM_BUFFERS 1

/**
 * Arduino inbuild headers
 */
#include "Arduino.h"
#include "HardwareSerial.h"
#include <avr/pgmspace.h>

/**
 * Board and software sepcific headers
 */
#include "mc_config.h"

/**
 * MAVLink headers
 */
//#include "../libraries/mavlink-c-lib/common/mavlink.h"
#include "./mavlink/ardupilotmega/mavlink.h"
#include "./mavlink/common/mavlink.h"
#include "mavlink_hlp.h"

/**
 * Parameters and common functions
 */
#include "parameters.h"
#include "mc_var.h"
#include "../libraries/LLtoUTM.h"

/**
 * Main source headers
 */
void loop_imm(void);
void loop_5Hz(void);
void loop_1Hz(void);
void do_request_stream_pos(HardwareSerial &uart, uint8_t sys_id);
void do_set_guided(HardwareSerial &uart, uint8_t sys_id_follower);
void do_arm(HardwareSerial &uart, uint8_t sys_id_follower);
void do_takeoff(float altitude, HardwareSerial &uart, uint8_t sys_id_follower);
void do_goto(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower);
void do_land(HardwareSerial &uart, uint8_t sys_id_follower);
void do_go_home(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower);
//inline int freeRam();

/**
 * Main loop timers
 * sys_tim - основной таймер (контроль загрузки процессора и т.п.)
 * loop_5Hz - отправка комманд ведомому
 * loop_1Hz - отладочные сообщения
 */
struct {
	volatile uint32_t sys_tim;
	volatile uint32_t loop_5Hz;
	volatile uint32_t loop_1Hz;
}timer = { 0, 0, 0 };

void setup()
{
	pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);
	pinMode(LED_BLUE, OUTPUT); 	digitalWrite(LED_BLUE, HIGH);
	pinMode(LED_RED, OUTPUT); 	digitalWrite(LED_RED, HIGH);

	UART_DEF.begin(BAUD_DEF);
	delay(5000);

	param_load_all();
	if(FOLL_NUM_p > 30) //
	{
		FOLL_NUM_p = 1;
	}
	follower[0].sys_id = FOLL_ID_1_p;
	follower[1].sys_id = FOLL_ID_2_p;
//	param_save_all();
	param_handle_req_list(UART_DEF);

	delay(100);
	do_request_stream_pos(UART_DEF, SYS_ID_LEADER);

	if(DBG_TXT_p == 1)
	{
		statustext_send("MC is READY", UART_DEF);
	}
	digitalWrite(LED_GREEN, LOW);
	digitalWrite(LED_BLUE, LOW);
}

void loop()
{
	loop_imm();

	if((millis() - timer.loop_5Hz) >= 200)
	{
		loop_5Hz();
		timer.loop_5Hz = millis();
	}

	if((millis() - timer.loop_1Hz) >= 1000)
	{
		loop_1Hz();
		timer.loop_1Hz = millis();
	}
}

void loop_imm(void)
{
	mavlink_message_t rd;
	mavlink_status_t status;

	while (UART_DEF.available() > 0) { 				 //если приняты данные
		uint8_t c = UART_DEF.read();					 //читаем

		if (mavlink_parse_char(MAVLINK_COMM_0, c, &rd, &status))
		{
			for (uint8_t i = 0; i < FOLL_NUM_p; i++)
			{
				if(rd.sysid == follower[i].sys_id)
				{
					switch (rd.msgid)
					{
					case MAVLINK_MSG_ID_HEARTBEAT:
					{
						if(follower[i].status == FOLL_STAT_UNKNOWN)	// если текущая система еще ни разу не біла идентифицирована
						{
							follower[i].status = FOLL_STAT_CONNECTED;	// регистрируем подключение в сеть
							do_request_stream_pos(UART_DEF, follower[i].sys_id);
							if(DBG_TXT_p == 1)
							{
								statustext_send("FOLL_CONNECTED", UART_DEF);
							}
						}
						follower[i].timer = millis();	// запускаем локальный таймер для мониторинга соединения
						break;
					}
					case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:	// принимаем текущие координаты ведомого в глобальной СК
					{
						follower[i].curr_z = (mavlink_msg_global_position_int_get_relative_alt(&rd) / (float)1000.0); // сразу же сохраняем в удобном UTM

						LLtoUTM((double)(mavlink_msg_global_position_int_get_lat(&rd) / (float)10000000.0),
								(double)(mavlink_msg_global_position_int_get_lon(&rd) / (float)10000000.0),
								follower[i].curr_x, follower[i].curr_y, follower[i].curr_zone);
						follower[i].curr_x = follower[i].curr_x * (float)0.997756835;
						follower[i].curr_y = follower[i].curr_y * (float)0.999420961;
						break;
					}
					default:
						break;
					}
				}
			}
			/**
			 * Обрабатываем телеметрию от ведущего
			 */
			if(rd.sysid == SYS_ID_LEADER)
			{
				switch (rd.msgid)
				{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					if(leader_connected == 0)
					{
						leader_connected = 1;
						digitalWrite(LED_GREEN, HIGH);
						digitalWrite(LED_RED, LOW);
					}
					timer.sys_tim = millis();
					break;
				}
				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				{
					uint16_t chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&rd);
					if((chan6_raw > 1500) && (flag_guide != GUIDE_GUIDED))		// если ведомые не в режиме следования и принята команда на переключение в РСП
					{
						digitalWrite(LED_BLUE, HIGH);

						for (uint8_t i = 0; i < FOLL_NUM_p; i++)
						{
							do_set_guided(UART_DEF, follower[i].sys_id);
						}
						delay(500);
						for (uint8_t i = 0; i < FOLL_NUM_p; i++)
						{
							do_arm(UART_DEF, follower[i].sys_id);
						}
						delay(1500);
						for (uint8_t i = 0; i < FOLL_NUM_p; i++)
						{
							do_takeoff(TKOFF_ALT_p, UART_DEF, follower[i].sys_id);
						}
						delay(TKOFF_ALT_p*1000);

						leader.home_x = leader.curr_x;		// текущие координаты лидера устанавливаем как координаты его home
						leader.home_y = leader.curr_y;
						leader.home_z = leader.curr_z;

						switch((uint8_t)INIT_POS_p)
						{
						case POS_START:		// с текущего положения: оставляем текщие координаты как home
						{
							for (uint8_t i = 0; i < FOLL_NUM_p; i++)
							{
								follower[i].home_x = follower[i].curr_x;
								follower[i].home_y = follower[i].curr_y;
								follower[i].home_z = 0;
							}
							break;
						}
						case POS_OFFS:	// со смещениями относительно лидера
						{
							for (uint8_t i = 0; i < FOLL_NUM_p; i++)
							{
								follower[i].home_x = leader.home_x + OFFS_X_p;
								follower[i].home_y = leader.home_y + OFFS_Y_p;
								follower[i].home_z = OFFS_Z_p;

								do_go_home(follower[i].home_x, follower[i].home_y, follower[i].home_z,
										UART_DEF, follower[i].sys_id);
							}
							break;
						}
						default:
							break;
						}

						flag_guide = GUIDE_GUIDED;
						digitalWrite(LED_BLUE, LOW);
					}

					if((chan6_raw < 1500) && (flag_guide == GUIDE_GUIDED))//MAV_CMD_NAV_LAND
					{
						leader.home_x = 0;
						leader.home_y = 0;

						if(DBG_TXT_p == 1)
						{
							statustext_send("LANDING", UART_DEF);
						}
						for (uint8_t i = 0; i < FOLL_NUM_p; i++)
						{
							do_land(UART_DEF, follower[i].sys_id);
						}

						flag_guide = GUIDE_LANDING;
					}
					break;
				}
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					leader.curr_z = (mavlink_msg_global_position_int_get_relative_alt(&rd) / (float)1000.0);

					LLtoUTM((double)(mavlink_msg_global_position_int_get_lat(&rd) / (float)10000000.0),
							(double)(mavlink_msg_global_position_int_get_lon(&rd) / (float)10000000.0),
							leader.curr_x, leader.curr_y, leader.curr_zone);
					leader.curr_x = leader.curr_x * (float)0.997756835;
					leader.curr_y = leader.curr_y * (float)0.999420961;

					leader.curr_hdg = mavlink_msg_global_position_int_get_hdg(&rd);

					x = leader.curr_x - leader.home_x;
					y = leader.curr_y - leader.home_y;

					if ((uint8_t)INIT_POS_p == POS_OFFS)
					{
						z = leader.curr_z + OFFS_Z_p;
					}else{
						z = leader.curr_z;
					}

					break;
				}
				default:
					break;
				}
			} /* Конец обработки телеметрии ведущего*/

			/**
			 * Обработка данных от наземной станции
			 * Прием параметров
			 */
			if (rd.sysid == SYS_ID_GCS)
			{
				switch (rd.msgid)
				{
				case MAVLINK_MSG_ID_PARAM_SET:
				{
					if(DBG_TXT_p == 1)
					{
						statustext_send("GOT_SOME_PARAM", UART_DEF);
					}

					mavlink_param_set_t param;

					mavlink_msg_param_set_decode(&rd, &param);
					param_handle_set(param, UART_DEF);
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				{
					param_handle_req_list(UART_DEF);
					if(DBG_TXT_p == 1)
					{
						statustext_send("GOT_PARAM_LIST_REQ", UART_DEF);
					}
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				{
					mavlink_param_request_read_t prq;
					mavlink_msg_param_request_read_decode(&rd, &prq);
					param_handle_req_value(prq, UART_DEF);
					if(DBG_TXT_p == 1)
					{
						statustext_send("GOT_PARAM_VAL_REQ", UART_DEF);
					}
					break;
				}
				default:
					break;
				}
			}
		}
	}
}

void loop_5Hz(void)
{
	for (uint8_t i = 0; i < FOLL_NUM_p; i++)
	{
		follower[i].curr_x = x;
		follower[i].curr_y = y;
		follower[i].curr_z = z;
		if(leader.curr_hdg != UINT16_MAX)
		{
			follower[i].curr_hdg = leader.curr_hdg;
		}

		if(flag_guide == GUIDE_GUIDED)
		{
			do_goto(follower[i].curr_x, follower[i].curr_y, follower[i].curr_z,
					UART_DEF, follower[i].sys_id);
		}

		if(flag_guide == GUIDE_LANDING)
		{
			do_land(UART_DEF, follower[i].sys_id);
		}
	}
}

void loop_1Hz(void)
{
	heartbeat_send(UART_DEF);
	systime_send(UART_DEF);

	if((millis() - timer.sys_tim) > CONNECTION_LOSS_TIME)
	{
		digitalWrite(LED_GREEN, LOW);
		digitalWrite(LED_RED, HIGH);
	}

	for (uint8_t i = 0; i < FOLL_NUM_p; i++)
	{
		if(((millis() - follower[i].timer) > CONNECTION_LOSS_TIME) && (follower[i].status != FOLL_STAT_UNKNOWN))
		{
			follower[i].status = FOLL_STAT_LOST;
			if(DBG_TXT_p == 1)
			{
				statustext_send("FOLL_CONNECT_LOST", UART_DEF);
			}
		}
	}
}

void do_request_stream_pos(HardwareSerial &uart, uint8_t sys_id)
{
	mavlink_message_t r_msg;
	mavlink_msg_request_data_stream_pack(SYS_ID_MY, COMP_ID_MY, &r_msg,
			sys_id, 0, MAV_DATA_STREAM_POSITION, 10, 1);
	send_message(&r_msg, uart);
	delay(500);
}

void do_set_guided(HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("START GUIDING", uart);
	}

	mavlink_message_t g_msg;
	mavlink_msg_set_mode_pack(SYS_ID_MY, COMP_ID_MY, &g_msg, sys_id_follower, GUIDED_B, GUIDED_C);
	send_message(&g_msg, uart);
//	delay(500);

	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &g_msg, sys_id_follower, 0,
			MAV_CMD_NAV_GUIDED_ENABLE, 1, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart);
//	delay(500);

	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &g_msg, sys_id_follower, 0,
			MAV_CMD_DO_GUIDED_LIMITS, 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart);
//	delay(500);
}

void do_arm(HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("ARMING", uart);
	}

	mavlink_message_t a_msg;
	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &a_msg, sys_id_follower, 0,
			MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
	send_message(&a_msg, uart);
//	delay(1500);
}

void do_takeoff(float altitude, HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("TAKEOFF", uart);
	}

	mavlink_message_t t_msg;
	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &t_msg, sys_id_follower, 0,
			MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
	send_message(&t_msg, uart);
	delay(500);

	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &t_msg, sys_id_follower, 0,
			MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
	send_message(&t_msg, uart);

	if(DBG_TXT_p == 1)
	{
		statustext_send("TAKEOFF IS DONE", uart);
	}
}

void do_goto(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t g_msg;

	// http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
	mavlink_msg_set_position_target_local_ned_pack(SYS_ID_MY, COMP_ID_MY, &g_msg,
			millis(), sys_id_follower, 0, MAV_FRAME_LOCAL_NED, (uint16_t)TYPE_POS_BITMASK,
			x, y, (-z), 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart);
}

void do_land(HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t l_msg;
	mavlink_msg_command_long_pack(SYS_ID_MY, COMP_ID_MY, &l_msg, sys_id_follower, 0,
			MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&l_msg, uart);
}

void do_go_home(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t h_msg;
	mavlink_msg_set_position_target_global_int_pack(SYS_ID_MY, COMP_ID_MY, &h_msg,
			millis(), sys_id_follower, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, TYPE_POS_BITMASK,
			(int32_t)(x*10000000.0), (int32_t)(y*10000000.0), z,
			0, 0, 0, 0, 0, 0, 0, 0);
	delay(sqrt(pow(OFFS_X_p, 2) + pow(OFFS_Y_p, 2))*1000);
}

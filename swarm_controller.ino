/**
 * Программа для реализации режима совместного полета (РСП) нескольких коптеров
 *
 * LED Pattern:
 * R, G, B - initialization
 * R - initialized, no link with Pixhawk
 * G - initialized, link is good (heartbeat is OK)
 * R, B or G, B - initialized, arm/takeoff stage
 */

#include "Arduino.h"
#include "inc.h"

//int freeRam()
//{
//  extern int __heap_start, *__brkval;
//  int v;
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//}

/**
 * Main source headers
 */
void loop_imm(void);
void loop_50Hz(void);
void loop_5Hz(void);
void loop_1Hz(void);

void setup()
{
	buzz_init();
	UART_DEF.begin(BAUD_DEF);

	param_load_all();
	if(FOLL_NUM_p > 30)
	{
		FOLL_NUM_p = 1;
	}
	follower[0].sys_id = FOLL_ID_1_p;
	follower[1].sys_id = FOLL_ID_2_p;

	param_handle_req_list(UART_DEF);

	delay(100);
	do_request_stream_pos(UART_DEF, SYS_ID_LEADER);

	if(DBG_TXT_p == 1)
	{
		statustext_send("MC_is_READY", UART_DEF);
	}

	main_timer.coord_dt[1] = millis();
	connect.leader = 0;
}

void loop()
{
	loop_imm();

	if((millis() - main_timer.loop_50Hz) >= 20)
	{
		loop_50Hz();
		main_timer.loop_50Hz = millis();
	}

	if((millis() - main_timer.loop_5Hz) >= 200)
	{
		loop_5Hz();
		main_timer.loop_5Hz = millis();
	}

	if((millis() - main_timer.loop_1Hz) >= 1000)
	{
		loop_1Hz();
		main_timer.loop_1Hz = millis();
	}
}

/**
 * Прием сообщений MAVLink от ведущего БПЛА и от наземной станции управления
 * Выполняестя всегда, как только есть свободное процессорное время и проверяет наличие
 * информации в приемном буфере UART; если что-то пришло, передаем это парсеру
 */
void loop_imm(void)
{
	if(UART_DEF.available() > 0)
	{
		mavlink_message_t rd;
		mavlink_status_t status;

		do{
		uint8_t c = UART_DEF.read();

		if (mavlink_parse_char(MAVLINK_COMM_0, c, &rd, &status))
		{
			handler_followers(rd); // обрабатываем телеметрию от ведомых

			if(rd.sysid == SYS_ID_LEADER)	// обрабатываем телеметрию от ведущего
			{
				handler_leader(rd);
			}

			if (rd.sysid == SYS_ID_GCS) 	// обработка данных от наземной станции (прием и отправка параметров)
			{
				handler_gcs(rd);
			}
		}
		}while(UART_DEF.available() > 0);
	}
}

/**
 * Обновление вывода периферийных блоков и таймера
 * Вызывает обновление состояния вывода пьезоизлучателя а также
 * проверяет на переполнение таймер задержки при переходе к следующей стадии РСП
 */
void loop_50Hz(void)
{
	buzz_update();

	// Проверка переполнения таймера
	if(((millis() - main_timer.guide) >= guide_timer_delay) && (flag_guide_timer == TIMER_SET))
	{
		flag_guide_timer = TIMER_RELEASED;  // освободили таймер
		if(flag_guide == GUIDE_LANDING)
		{
			flag_guide = GUIDE_NOT_GUIDED;	// после посадки переходим в состояние инициализации
		}else{
			flag_guide++;	// или переходим к следующей стадии РСП
		}
	}
}

/**
 * Отправка команд выдомым (позиционнирование)
 */
void loop_5Hz(void)
{
	for (uint8_t i = 0; i < FOLL_NUM_p; i++) // for each folower
	{
		if(connect.leader == 1)		// if leader is connected
		{
			if(POS_TYPE_p == 0)		// update follower coordinates
			{
				follower[i].curr_x = relative_c.x;
				follower[i].curr_y = relative_c.y;
				follower[i].curr_z = relative_c.z;
			}else if(POS_TYPE_p == 1){
				follower[i].curr_x = relative_p.x;
				follower[i].curr_y = relative_p.y;
				follower[i].curr_z = relative_p.z;
			}

			if(leader.curr_hdg != UINT16_MAX)
			{
				follower[i].curr_hdg = leader.curr_hdg;
			}

			if(flag_guide == GUIDE_GUIDED)
			{
				do_goto(follower[i].curr_x, follower[i].curr_y, follower[i].curr_z, follower[i].curr_hdg,
						UART_DEF, follower[i].sys_id);
			}
		}

		if(flag_guide == GUIDE_LANDING)
		{
			do_land(UART_DEF, follower[i].sys_id);
		}
	}
}

/**
 * Проверка состояния соединиений и отправка сообщений, необходимых для
 * идентификации системы в сети
 */
void loop_1Hz(void)
{
	heartbeat_send(UART_DEF);
	systime_send(UART_DEF);

	if((millis() - leader.timer) > CONNECTION_LOSS_TIME)
	{
		if(connect.leader == 1)
		{
			connect.leader = 0;
		}
	}

	for (uint8_t i = 0; i < FOLL_NUM_p; i++)
	{
		if(((millis() - follower[i].timer) > CONNECTION_LOSS_TIME) && (follower[i].status != FOLL_STAT_UNKNOWN))
		{
			follower[i].status = FOLL_STAT_LOST;
			buzz(200);
		}
	}

//	char buf[10];
//	snprintf(buf, sizeof(buf), "%d", freeRam());
//	statustext_send(buf, UART_DEF);
}

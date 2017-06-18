#ifndef HANDLERS_H_
#define HANDLERS_H_

#include "actions.h"

void handler_gcs(mavlink_message_t &rd)
{
	switch (rd.msgid)
	{
	case MAVLINK_MSG_ID_PARAM_SET:
	{
		buzz(200);
		mavlink_param_set_t param;

		mavlink_msg_param_set_decode(&rd, &param);
		param_handle_set(param, UART_DEF);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	{
		param_handle_req_list(UART_DEF);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	{
		mavlink_param_request_read_t prq;
		mavlink_msg_param_request_read_decode(&rd, &prq);
		param_handle_req_value(prq, UART_DEF);
		break;
	}
	default:
		break;
	}
}

void handler_leader(mavlink_message_t &rd)
{
	switch (rd.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		if(connect.leader == 0)
		{
			connect.leader = 1;
			buzz(1000);
		}
		leader.timer = millis();
		break;
	}
	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
	{
		uint16_t chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&rd);
		if((chan6_raw > 1500) && (flag_guide != GUIDE_GUIDED))		// если ведомые не в режиме следования и принята команда на переключение в РСП
		{
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
//			delay(TKOFF_ALT_p*1000);

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
					follower[i].home_z = leader.home_z + OFFS_Z_p;

					do_go_home(follower[i].home_x, follower[i].home_y, follower[i].home_z, leader.curr_zone,
							UART_DEF, follower[i].sys_id);
				}
				break;
			}
			default:
				break;
			}

			flag_guide = GUIDE_GUIDED;
			buzz(200);
		}

		if((chan6_raw < 1500) && (flag_guide == GUIDE_GUIDED))//MAV_CMD_NAV_LAND
		{
			leader.home_x = 0;
			leader.home_y = 0;

			if(DBG_TXT_p == 1)
			{
				statustext_send("LD", UART_DEF);
			}
			for (uint8_t i = 0; i < FOLL_NUM_p; i++)
			{
				do_land(UART_DEF, follower[i].sys_id);
			}

			flag_guide = GUIDE_LANDING;
			buzz(200);
		}
		break;
	}
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	{
		main_timer.coord_dt[0] = main_timer.coord_dt[1];
		main_timer.coord_dt[1] = millis();
		uint16_t dt = main_timer.coord_dt[1] - main_timer.coord_dt[0];

		leader.curr_z = (mavlink_msg_global_position_int_get_relative_alt(&rd) / (float)1000.0);

		LLtoUTM((double)(mavlink_msg_global_position_int_get_lat(&rd) / (float)10000000.0),
				(double)(mavlink_msg_global_position_int_get_lon(&rd) / (float)10000000.0),
				leader.curr_x, leader.curr_y, leader.curr_zone);
		leader.curr_x = leader.curr_x * (float)0.997756835;
		leader.curr_y = leader.curr_y * (float)0.999420961;
		leader.curr_hdg = mavlink_msg_global_position_int_get_hdg(&rd);

		leader.vx = mavlink_msg_global_position_int_get_vx(&rd);
		leader.vy = mavlink_msg_global_position_int_get_vy(&rd);
		leader.vz = mavlink_msg_global_position_int_get_vz(&rd);

		relative_c.x = leader.curr_x - leader.home_x;
		relative_c.y = leader.curr_y - leader.home_y;
		relative_c.z = leader.curr_z - leader.home_z;

		relative_p.x = leader.vx * dt;
		relative_p.y = leader.vy * dt;
		relative_p.z = leader.vz * dt;

		// TODO: check behavior of altitude control for the case of using offsets (line 190)
//					if ((uint8_t)INIT_POS_p == POS_OFFS)
//					{
//						z = leader.curr_z + OFFS_Z_p;
//					}else{
//						z = leader.curr_z;
//					}

		break;
	}
	default:
		break;
	}
}

void handler_followers(mavlink_message_t &rd)
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
					connect.followers++;
					do_request_stream_pos(UART_DEF, follower[i].sys_id);
					if(DBG_TXT_p == 1)
					{
						statustext_send("FC", UART_DEF);
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
}

#endif /* HANDLERS_H_ */

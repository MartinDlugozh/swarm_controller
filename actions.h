#ifndef ACTIONS_H_
#define ACTIONS_H_

void do_request_stream_pos(HardwareSerial &uart, uint8_t sys_id)
{
	mavlink_message_t r_msg;
	mavlink_msg_request_data_stream_pack(MY_SYS_ID, MY_COMP_ID, &r_msg,
			sys_id, 0, MAV_DATA_STREAM_POSITION, 10, 1);
	send_message(&r_msg, uart);
}

void do_set_guided(HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("SG", uart);
	}

	mavlink_message_t g_msg;
	mavlink_msg_set_mode_pack(MY_SYS_ID, MY_COMP_ID, &g_msg, sys_id_follower, GUIDED_B, GUIDED_C);
	send_message(&g_msg, uart); g_msg = {0};

	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &g_msg, sys_id_follower, 0,
			MAV_CMD_NAV_GUIDED_ENABLE, 1, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart); g_msg = {0};

	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &g_msg, sys_id_follower, 0,
			MAV_CMD_DO_GUIDED_LIMITS, 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart);
}

void do_arm(HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("AR", uart);
	}

	mavlink_message_t a_msg;
	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &a_msg, sys_id_follower, 0,
			MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
	send_message(&a_msg, uart);
}

void do_takeoff(float altitude, HardwareSerial &uart, uint8_t sys_id_follower)
{
	if(DBG_TXT_p == 1)
	{
		statustext_send("TK", uart);
	}

	mavlink_message_t t_msg;
	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &t_msg, sys_id_follower, 0,
			MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
	send_message(&t_msg, uart); t_msg = {0};

	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &t_msg, sys_id_follower, 0,
			MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
	send_message(&t_msg, uart);

	if(DBG_TXT_p == 1)
	{
		statustext_send("TD", uart);
	}
}

void do_goto(float x, float y, float z, uint16_t hdg, HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t g_msg;

	// http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
	mavlink_msg_set_position_target_local_ned_pack(MY_SYS_ID, MY_COMP_ID, &g_msg,
			millis(), sys_id_follower, 0, MAV_FRAME_LOCAL_NED, (uint16_t)TYPE_POS_BITMASK,
			x, y, (-z), 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart); g_msg = {0};

	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &g_msg, sys_id_follower, 0,
			MAV_CMD_CONDITION_YAW, (hdg/100.0f), 180, 0, 0, 0, 0, 0, 0);
	send_message(&g_msg, uart);
}

void do_land(HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t l_msg;
	mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &l_msg, sys_id_follower, 0,
			MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&l_msg, uart);
}

void do_go_home(float x, float y, float z, int zone, HardwareSerial &uart, uint8_t sys_id_follower)
{
	mavlink_message_t h_msg;
	mavlink_msg_set_position_target_global_int_pack(MY_SYS_ID, MY_COMP_ID, &h_msg,
			millis(), sys_id_follower, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, TYPE_POS_BITMASK,
			(int32_t)(x*10000000.0f), (int32_t)(y*10000000.0f), z,
			0, 0, 0, 0, 0, 0, 0, 0);
	send_message(&h_msg, uart); h_msg = {0};

	double lat, lon;
	UTMtoLL((x/0.997756835f), (y/0.999420961f), zone, lat, lon);

	mavlink_msg_set_home_position_pack(MY_SYS_ID, MY_COMP_ID, &h_msg,
			sys_id_follower, (int32_t)(lat*10000000.0f), (int32_t)(lon*10000000.0f), z,
			0, 0, 0, 0, 0, 0, 0);
	send_message(&h_msg, uart);
}

#endif /* ACTIONS_H_ */

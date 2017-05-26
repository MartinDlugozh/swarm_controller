#ifndef _ML_HELPERS_H_
#define _ML_HELPERS_H_

void send_message(mavlink_message_t* msg, HardwareSerial &uart)				//функция отправки сообщения
{
	uint8_t buf[256];				//создаем буфер
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);//пишем туда наше сообщение
	for (uint16_t i = 0; i < len; i++)					//по байту отправляем содержимое буфера
	{
		uart.write(buf[i]);
	}
}

void heartbeat_send(HardwareSerial &uart)
{
	mavlink_message_t hb;

//	mavlink_msg_heartbeat_pack(MY_SYS_ID, MY_COMP_ID, &hb,
//			MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC,
//			MAV_MODE_FLAG_STABILIZE_ENABLED, MAV_MODE_STABILIZE_ARMED, MAV_STATE_ACTIVE);
//	send_message(&hb, uart);
	mavlink_msg_heartbeat_pack(SYS_ID_MY, COMP_ID_MY, &hb,
				MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC,
				0, 0, 0);
	send_message(&hb, uart);
}

void systime_send(HardwareSerial &uart)
{
	mavlink_message_t st;

	mavlink_msg_system_time_pack(SYS_ID_MY, COMP_ID_MY, &st,
			0, millis());
	send_message(&st, uart);
}

void statustext_send(const char *text, HardwareSerial &uart)
{
	mavlink_message_t tx;

	mavlink_msg_statustext_pack(SYS_ID_MY,
			COMP_ID_MY, &tx, MAV_SEVERITY_INFO, text);
	send_message(&tx, uart);
}

#endif

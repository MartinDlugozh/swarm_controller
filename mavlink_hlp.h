#ifndef _ML_HELPERS_H_
#define _ML_HELPERS_H_

#define MAVLINK_TX_BUFFER 128;

void send_message(mavlink_message_t* msg, HardwareSerial &uart)				//функция отправки сообщения
{
	uint8_t buf[sizeof(mavlink_message_t)];				//создаем буфер
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);//пишем туда наше сообщение
	for (uint16_t i = 0; i < len; i++)					//по байту отправляем содержимое буфера
	{
		uart.write(buf[i]);
	}
}

void heartbeat_send(HardwareSerial &uart)
{
	mavlink_message_t hb;

	mavlink_msg_heartbeat_pack(MY_SYS_ID, MY_COMP_ID, &hb,
				MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC,
				0, 0, 0);
	send_message(&hb, uart);
}

void systime_send(HardwareSerial &uart)
{
	mavlink_message_t st;

	mavlink_msg_system_time_pack(MY_SYS_ID, MY_COMP_ID, &st,
			0, millis());
	send_message(&st, uart);
}

void statustext_send(const char *text, HardwareSerial &uart)
{
	mavlink_message_t tx;

	mavlink_msg_statustext_pack(MY_SYS_ID,
			MY_COMP_ID, &tx, MAV_SEVERITY_INFO, text);
	send_message(&tx, uart);
}

#endif

#ifndef MC_CONFIG_H_
#define MC_CONFIG_H_

#define UART_0 					Serial					// Номера портов - основной
#define UART_1 					Serial1					// Номера портов - отладочный
#define UART_2					Serial2
#define UART_3					Serial3
#define UART_DEF				UART_1

#define BAUD_38					38400
#define BAUD_57					57600
#define BAUD_115				115200
#define BAUD_DEF				BAUD_57					// Скорость порта для соединения с автопилотом

#define SYS_ID_GCS 				255

#define SYS_ID_MY				99						// SYSTEM_ID микроконтроллера
#define COMP_ID_MY				0						// COMPONNENT_ID микроконтроллера

#define SYS_ID_LEADER 			1						// SYSTEM_ID ведущего коптера
#define SYS_ID_FOLLOWER 		2						// SYSTEM_ID ведомого коптера (в плане перенести в раздел параметров для большего числа бортов в группе)
#define FOLL_NUM_MAX 			2

#define TYPE_POS_BITMASK		0b0000111111111000 		// Маска для SET_POS_TARGET - NED(x,y,-z)

#define GUIDED_B				89						// Режим Guided (Set mode)
#define GUIDED_C				4						// Режим Guided (Set mode)
#define STABILIZE_B				81						// Режим Stabilize (Set mode)
#define STABILIZE_C				0						// Режим Stabilize (Set mode)

#define GUIDE_NOT_GUIDED 		0						// Флаг стадии полета - инициализация
#define GUIDE_GUIDED 			1						// ФСП - РСП
#define GUIDE_LANDING 			2						// ФСП - посадка ведомого

#define FOLL_STAT_UNKNOWN		0
#define FOLL_STAT_CONNECTED		1
#define FOLL_STAT_NOT_GUIDED	2
#define FOLL_STAT_GUIDED		3
#define FOLL_STAT_LANDING		4
#define FOLL_STAT_LOST			10

#define CONNECTION_LOSS_TIME	5000

#define POS_START 				0						// Значение INIT_POS_p - оффсет по месту старта
#define POS_OFFS				1						// Значение INIT_POS_p - оффсет от MAV1_HOME

#define LED_GREEN				51
#define LED_BLUE				52
#define LED_RED					53

#endif /* MC_CONFIG_H_ */

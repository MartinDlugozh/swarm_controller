#ifndef MC_VAR_H_
#define MC_VAR_H_

/**
 * Main loop timers
 * coord_dt[2] - приращение времени
 * 				 (интегрирование скорости при определении приращения координаты)
 * loop_50Hz - обновление вывода периферийных блоков
 * loop_5Hz - отправка команд ведомым
 * loop_1Hz - отладочные сообщения и heartbeat
 */
struct {
	volatile uint32_t coord_dt[2];
	volatile uint32_t guide;
	volatile uint32_t loop_50Hz;
	volatile uint32_t loop_5Hz;
	volatile uint32_t loop_1Hz;
}main_timer = { 0, 0, 0, 0, 0 };

/**
 * Connection timers
 * leader - состояние соединения с АП ведущего БПЛА
 * followers - состояние соединения с АП ведомого БПЛА
 * gcs - - состояние соединения с наземной станцией
 */
struct Connection{
	uint8_t leader;
	uint8_t followers;
	uint8_t gcs;
}connect = { 0, 0, 0, };

/**
 * Main loop flag
 * флаг состояния выполнения главного цикла
 */
uint8_t flag_guide = 0; // стадия выполнения программы РСП (значения описаны в заголовке "mc_config.h"
uint8_t flag_guide_timer = 0; // флаг таймера задержек при переходе к следующей стадии

/**
 * Значение задержки при переходе к следующей стадии РСП
 */
uint16_t guide_timer_delay = 0;

/**
 * Координаты лидера в системе NED (D отрицателен)
 * leader_home_x и leader_home_y необходимы, т.к. переход от глобальных координат GPS в NED
 * выполняется на этом МК
 */
struct
{
	float x;
	float y;
	float z;
}relative_c = { 0, 0, 0 }, relative_p = { 0, 0, 0 };

/**
 * AP parameters
 * Параметры подключенных к МК автопилотов
 * ...
 * timer - время крайнего heartbeat от АП
 */
struct
{
	uint8_t sys_id;
	uint8_t status;
	volatile uint32_t timer;
	double home_x;
	double home_y;
	double home_z;
	double curr_x;
	double curr_y;
	double curr_z;
	uint16_t curr_hdg;
	int curr_zone;
	float vx;
	float vy;
	float vz;
}follower[FOLL_NUM_MAX] = {0}, leader = {0};

#endif /* MC_VAR_H_ */

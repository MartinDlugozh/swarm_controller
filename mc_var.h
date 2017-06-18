#ifndef MC_VAR_H_
#define MC_VAR_H_

/**
 * Main loop timers
 * sys_tim - основной таймер (контроль загрузки процессора и т.п.)
 * loop_5Hz - отправка комманд ведомому
 * loop_1Hz - отладочные сообщения
 */
struct {
	volatile uint32_t sys_tim;
	volatile uint32_t coord_dt[2];
	volatile uint32_t loop_50Hz;
	volatile uint32_t loop_5Hz;
	volatile uint32_t loop_1Hz;
}main_timer = { 0, 0, 0, 0, 0, 0 };

struct Connection{
	uint8_t leader;
	uint8_t followers;
	uint8_t gcs;
}connect = { 0, 0, 0, };

uint8_t flag_guide = 0;

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

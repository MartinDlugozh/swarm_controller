#ifndef MC_VAR_H_
#define MC_VAR_H_

uint8_t 	flag_guide			= 0;

uint8_t 	leader_connected 	= 0;
uint8_t 	followers_connected = 0;

/**
 * Координаты лидера в системе NED (D отрицателен)
 * leader_home_x и leader_home_y необходимы, т.к. переход от глобальных координат GPS в NED
 * выполняется на этом МК
 */
//double 		leader_x 			= 0;
//double 		leader_y 			= 0;
//float 		leader_z 			= 0;
//int 		zone 				= 0;
//
//float 		leader_home_x 		= 0;
//float 		leader_home_y 		= 0;

float 		x 					= 0;
float 		y 					= 0;
float 		z 					= 0;

//float 		follower_home_x 	= 0;
//float 		follower_home_y 	= 0;
//float 		follower_home_z 	= 0;

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
}follower[FOLL_NUM_MAX] = {0}, leader = {0};
#endif /* MC_VAR_H_ */

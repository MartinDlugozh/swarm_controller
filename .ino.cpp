#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-05-25 13:49:33

#include "Arduino.h"
#include "Arduino.h"
#include "HardwareSerial.h"
#include <avr/pgmspace.h>
#include "mc_config.h"
#include "./mavlink/ardupilotmega/mavlink.h"
#include "./mavlink/common/mavlink.h"
#include "mavlink_hlp.h"
#include "parameters.h"
#include "mc_var.h"
#include "../libraries/LLtoUTM.h"
void setup() ;
void loop() ;
void loop_imm(void) ;
void loop_5Hz(void) ;
void loop_1Hz(void) ;
void do_request_stream_pos(HardwareSerial &uart, uint8_t sys_id) ;
void do_set_guided(HardwareSerial &uart, uint8_t sys_id_follower) ;
void do_arm(HardwareSerial &uart, uint8_t sys_id_follower) ;
void do_takeoff(float altitude, HardwareSerial &uart, uint8_t sys_id_follower) ;
void do_goto(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower) ;
void do_land(HardwareSerial &uart, uint8_t sys_id_follower) ;
void do_go_home(float x, float y, float z, HardwareSerial &uart, uint8_t sys_id_follower) ;

#include "swarm_controller.ino"


#endif

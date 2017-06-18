#ifndef INC_H_
#define INC_H_

/**
 * Arduino inbuild headers
 */
#include "HardwareSerial.h"
#include <avr/pgmspace.h>

/**
 * Board and software sepcific headers
 */
#include "mc_config.h"
#include "mc_var.h"
#include "buzz.h"

/**
 * MAVLink headers
 */
#include "./mavlink_avr/ardupilotmega/mavlink.h"
#include "./mavlink_avr/common/mavlink.h"
#include "./mavlink_avr/protocol.h"
#include "./mavlink_avr/mavlink_helpers.h"
#include "./mavlink_avr/checksum.h"
#include "./mavlink_avr/mavlink_types.h"
#include "mavlink_hlp.h"

/**
 * Parameters and common functions
 */
#include "../libraries/LLtoUTM.h"
#include "param_helper.h"
#include "actions.h"
#include "handlers.h"

#endif /* INC_H_ */

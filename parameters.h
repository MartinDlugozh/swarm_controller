#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#define ONBOARD_PARAM_COUNT 10

/**
 * Definitions
 */
#define 		OFFS_X_p 		param_values[0]
#define 		OFFS_Y_p 		param_values[1]
#define 		OFFS_Z_p 		param_values[2]
#define 		INIT_POS_p		param_values[3]
#define 		DBG_TXT_p		param_values[4]
#define 		TKOFF_ALT_p 	param_values[5]
#define 		POS_TYPE_p	 	param_values[6]
#define 		FOLL_NUM_p 		param_values[7]
#define 		FOLL_ID_1_p		param_values[8]
#define 		FOLL_ID_2_p		param_values[9]

/**
 * Parameter names
 * (PROGMEM string literals)
 */
const char param_0[] PROGMEM = "OFFS_X";				// [0]
const char param_1[] PROGMEM = "OFFS_Y";				// [1]
const char param_2[] PROGMEM = "OFFS_Z";				// [2]
const char param_3[] PROGMEM = "INIT_POS";				// [3]
const char param_4[] PROGMEM = "DBG_TXT";				// [4]
const char param_5[] PROGMEM = "TKOFF_ALT";				// [5]
const char param_6[] PROGMEM = "POS_TYPE";				// [6]
const char param_7[] PROGMEM = "FOLL_NUM";				// [7]
const char param_8[] PROGMEM = "FOLL_ID_1";				// [8]
const char param_9[] PROGMEM = "FOLL_ID_2";				// [9]

/**
 * Parameter names table
 */
const char* const param_table[] PROGMEM = {
		param_0,					// [0]
		param_1,					// [1]
		param_2,					// [2]
		param_3,					// [3]
		param_4,					// [4]
		param_5,					// [5]
		param_6,					// [6]
		param_7,					// [7]
		param_8,					// [8]
		param_9						// [9]
};

/**
 * Parameter values
 * WARNING: all of the parameters have a type of uint8_t (unsigned char)
 * because of using default Arduino EEPROM library,
 * but receiving from and transmitting to the GCS is using float data type
 */
uint8_t			param_values[ONBOARD_PARAM_COUNT] = {
				0,					// [0]
				0,					// [1]
				0,					// [2]
				0,					// [3]
				0,					// [4]
				0,					// [5]
				0,					// [6]
				0,					// [7]
				0,					// [8]
				0};					// [9]



#endif /* PARAMETERS_H_ */

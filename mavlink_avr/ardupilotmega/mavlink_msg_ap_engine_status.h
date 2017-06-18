// MESSAGE AP_ENGINE_STATUS PACKING

#define MAVLINK_MSG_ID_AP_ENGINE_STATUS 220

MAVPACKED(
typedef struct __mavlink_ap_engine_status_t {
 float tmp; /*< Temperature*/
 float ffr; /*< Fuel Flow Rate*/
 float frm; /*< Fuel Remaining*/
 uint16_t rpm; /*< RPM*/
}) mavlink_ap_engine_status_t;

#define MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN 14
#define MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_220_LEN 14
#define MAVLINK_MSG_ID_220_MIN_LEN 14

#define MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC 14
#define MAVLINK_MSG_ID_220_CRC 14



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AP_ENGINE_STATUS { \
	220, \
	"AP_ENGINE_STATUS", \
	4, \
	{  { "tmp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ap_engine_status_t, tmp) }, \
         { "ffr", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ap_engine_status_t, ffr) }, \
         { "frm", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ap_engine_status_t, frm) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ap_engine_status_t, rpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AP_ENGINE_STATUS { \
	"AP_ENGINE_STATUS", \
	4, \
	{  { "tmp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ap_engine_status_t, tmp) }, \
         { "ffr", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ap_engine_status_t, ffr) }, \
         { "frm", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ap_engine_status_t, frm) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ap_engine_status_t, rpm) }, \
         } \
}
#endif

/**
 * @brief Pack a ap_engine_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rpm RPM
 * @param tmp Temperature
 * @param ffr Fuel Flow Rate
 * @param frm Fuel Remaining
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_engine_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t rpm, float tmp, float ffr, float frm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN];
	_mav_put_float(buf, 0, tmp);
	_mav_put_float(buf, 4, ffr);
	_mav_put_float(buf, 8, frm);
	_mav_put_uint16_t(buf, 12, rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN);
#else
	mavlink_ap_engine_status_t packet;
	packet.tmp = tmp;
	packet.ffr = ffr;
	packet.frm = frm;
	packet.rpm = rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_ENGINE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
}

/**
 * @brief Pack a ap_engine_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm RPM
 * @param tmp Temperature
 * @param ffr Fuel Flow Rate
 * @param frm Fuel Remaining
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_engine_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t rpm,float tmp,float ffr,float frm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN];
	_mav_put_float(buf, 0, tmp);
	_mav_put_float(buf, 4, ffr);
	_mav_put_float(buf, 8, frm);
	_mav_put_uint16_t(buf, 12, rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN);
#else
	mavlink_ap_engine_status_t packet;
	packet.tmp = tmp;
	packet.ffr = ffr;
	packet.frm = frm;
	packet.rpm = rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_ENGINE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
}

/**
 * @brief Encode a ap_engine_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_engine_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_engine_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_engine_status_t* ap_engine_status)
{
	return mavlink_msg_ap_engine_status_pack(system_id, component_id, msg, ap_engine_status->rpm, ap_engine_status->tmp, ap_engine_status->ffr, ap_engine_status->frm);
}

/**
 * @brief Encode a ap_engine_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ap_engine_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_engine_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ap_engine_status_t* ap_engine_status)
{
	return mavlink_msg_ap_engine_status_pack_chan(system_id, component_id, chan, msg, ap_engine_status->rpm, ap_engine_status->tmp, ap_engine_status->ffr, ap_engine_status->frm);
}

/**
 * @brief Send a ap_engine_status message
 * @param chan MAVLink channel to send the message
 *
 * @param rpm RPM
 * @param tmp Temperature
 * @param ffr Fuel Flow Rate
 * @param frm Fuel Remaining
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_engine_status_send(mavlink_channel_t chan, uint16_t rpm, float tmp, float ffr, float frm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN];
	_mav_put_float(buf, 0, tmp);
	_mav_put_float(buf, 4, ffr);
	_mav_put_float(buf, 8, frm);
	_mav_put_uint16_t(buf, 12, rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS, buf, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
#else
	mavlink_ap_engine_status_t packet;
	packet.tmp = tmp;
	packet.ffr = ffr;
	packet.frm = frm;
	packet.rpm = rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
#endif
}

/**
 * @brief Send a ap_engine_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ap_engine_status_send_struct(mavlink_channel_t chan, const mavlink_ap_engine_status_t* ap_engine_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ap_engine_status_send(chan, ap_engine_status->rpm, ap_engine_status->tmp, ap_engine_status->ffr, ap_engine_status->frm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS, (const char *)ap_engine_status, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ap_engine_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t rpm, float tmp, float ffr, float frm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, tmp);
	_mav_put_float(buf, 4, ffr);
	_mav_put_float(buf, 8, frm);
	_mav_put_uint16_t(buf, 12, rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS, buf, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
#else
	mavlink_ap_engine_status_t *packet = (mavlink_ap_engine_status_t *)msgbuf;
	packet->tmp = tmp;
	packet->ffr = ffr;
	packet->frm = frm;
	packet->rpm = rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ENGINE_STATUS, (const char *)packet, MAVLINK_MSG_ID_AP_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_AP_ENGINE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE AP_ENGINE_STATUS UNPACKING


/**
 * @brief Get field rpm from ap_engine_status message
 *
 * @return RPM
 */
static inline uint16_t mavlink_msg_ap_engine_status_get_rpm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field tmp from ap_engine_status message
 *
 * @return Temperature
 */
static inline float mavlink_msg_ap_engine_status_get_tmp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ffr from ap_engine_status message
 *
 * @return Fuel Flow Rate
 */
static inline float mavlink_msg_ap_engine_status_get_ffr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field frm from ap_engine_status message
 *
 * @return Fuel Remaining
 */
static inline float mavlink_msg_ap_engine_status_get_frm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a ap_engine_status message into a struct
 *
 * @param msg The message to decode
 * @param ap_engine_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ap_engine_status_decode(const mavlink_message_t* msg, mavlink_ap_engine_status_t* ap_engine_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	ap_engine_status->tmp = mavlink_msg_ap_engine_status_get_tmp(msg);
	ap_engine_status->ffr = mavlink_msg_ap_engine_status_get_ffr(msg);
	ap_engine_status->frm = mavlink_msg_ap_engine_status_get_frm(msg);
	ap_engine_status->rpm = mavlink_msg_ap_engine_status_get_rpm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN;
        memset(ap_engine_status, 0, MAVLINK_MSG_ID_AP_ENGINE_STATUS_LEN);
	memcpy(ap_engine_status, _MAV_PAYLOAD(msg), len);
#endif
}

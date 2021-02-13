/****************************************************************************
 *
 *	Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file messages.hpp
 *
 * Spektrum telemetry message formatting for SRXL
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 */

#pragma once

#pragma pack(push, 1)

typedef uint8_t UINT8;
typedef int8_t INT8;
typedef uint16_t UINT16;
typedef int16_t INT16;
typedef uint32_t UINT32;
typedef int32_t INT32;
typedef uint64_t UINT64;
typedef float FP32;

#include "spektrum_telemetry_defs.h"

typedef struct telemety_frame_s {
	UN_TELEMETRY telemetry;
} telemetry_frame_t;

#pragma pack(pop)

#ifndef MIN
#  define MIN(a,b) a < b ? a : b
#endif

#define TELE_FRAMETYPE_SID          0x00

#define MAX_SUBSCRIPTIONS 8

typedef const struct orb_metadata *const orb_item_t;

using namespace matrix;

static inline uint8_t binToBcd8(uint8_t num)
{
	uint8_t p2 = (uint8_t)((num / 10UL) % 10UL);
	uint8_t p1 = (uint8_t)((num / 1UL) % 10UL);
	return (p2 << 4) | (p1);
}

static inline uint16_t binToBcd16(uint16_t num)
{
	uint8_t p4 = (uint8_t)((num / 1000UL) % 10UL);
	uint8_t p3 = (uint8_t)((num / 100UL) % 10UL);
	uint8_t p2 = (uint8_t)((num / 10UL) % 10UL);
	uint8_t p1 = (uint8_t)((num / 1UL) % 10UL);
	return (p4 << 12) | (p3 << 8) | (p2 << 4) | (p1);
}

static inline uint32_t binToBcd32(uint32_t num)
{
	uint8_t p8 = (uint8_t)((num / 10000000UL) % 10UL);
	uint8_t p7 = (uint8_t)((num / 1000000UL) % 10UL);
	uint8_t p6 = (uint8_t)((num / 100000UL) % 10UL);
	uint8_t p5 = (uint8_t)((num / 10000UL) % 10UL);
	uint8_t p4 = (uint8_t)((num / 1000UL) % 10UL);
	uint8_t p3 = (uint8_t)((num / 100UL) % 10UL);
	uint8_t p2 = (uint8_t)((num / 10UL) % 10UL);
	uint8_t p1 = (uint8_t)((num / 1UL) % 10UL);
	return (p8 << 28) | (p7 << 24) | (p6 << 20) | (p5 << 16) | (p4 << 12) | (p3 << 8) | (p2 << 4) | (p1);
}

static constexpr uint16_t leToBe16(uint16_t num)
{
	return ((num & 0xFF) << 8) | (num >> 8);
}

class SpektrumTelemetryItem
{
protected:

	orb_item_t *m_topics;
	int m_subscriptions[MAX_SUBSCRIPTIONS];
	int m_topics_count;
	bool m_updated;
	bool m_started;

public:
	SpektrumTelemetryItem(orb_item_t topics[], int topics_count)
		: m_topics(topics)
		, m_topics_count((topics_count < MAX_SUBSCRIPTIONS) ? topics_count : MAX_SUBSCRIPTIONS)
		, m_started(false)
	{}

	bool topicsUpdated()
	{
		if (!m_started) {
			start();
		}

		bool updated = false;

		for (int i = 0; i < m_topics_count && !updated; i++) {
			orb_check(m_subscriptions[i], &updated);
		}

		return updated;
	}

	void getSubscriptionData(void *const data[])
	{
		for (int i = 0; i < m_topics_count; i++) {
			orb_copy(m_topics[i], m_subscriptions[i], data[i]);
		}
	}

	void start()
	{
		if (m_started) {
			return;
		}

		for (int i = 0; i < m_topics_count; i++) {
			m_subscriptions[i] = orb_subscribe(m_topics[i]);
		}

		m_started = true;
	}

	~SpektrumTelemetryItem()
	{
		if (!m_started) {
			return;
		}

		for (int i = 0; i < m_topics_count; i++) {
			orb_unsubscribe(m_subscriptions[i]);
		}
	}

	virtual bool update(UN_TELEMETRY *item) = 0;
};

class PowerBoxTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	void *const data[1];

public:
	PowerBoxTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status) }, 1)
	, data { &batt }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_POWERBOX *pb = &item->powerBox;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		pb->identifier = TELE_DEVICE_PBOX;
		pb->sID = TELE_FRAMETYPE_SID;

		pb->volt1 = ((uint16_t)(batt.voltage_filtered_v * 100.0f));
		pb->capacity1 = ((uint16_t)(batt.capacity * 10.0f));

		pb->volt2 = 0;
		pb->capacity2 = 0;

		pb->alarms = (batt.warning > battery_status_s::BATTERY_WARNING_LOW) ? TELE_PBOX_ALARM_VOLTAGE_1 |
			     TELE_PBOX_ALARM_CAPACITY_1 : 0;

		return true;
	}
};

class HVTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	void *const data[1];

public:
	HVTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status) }, 1)
	, data { &batt }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_HV *pb = &item->hv;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		pb->identifier = TELE_DEVICE_VOLTAGE;
		pb->sID = TELE_FRAMETYPE_SID;

		uint16_t value = (uint16_t)(batt.voltage_filtered_v * 100.0f);
		pb->volts = leToBe16(value);

		return true;
	}
};

class IHighTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	void *const data[1];

public:
	IHighTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status) }, 1)
	, data { &batt }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_IHIGH *pb = &item->amps;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		pb->identifier = TELE_DEVICE_AMPS;
		pb->sID = TELE_FRAMETYPE_SID;

		uint16_t value = (uint16_t)(batt.current_filtered_a / IHIGH_RESOLUTION_FACTOR);
		pb->current = leToBe16(value);

		return true;
	}
};

class TeleRpmTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	vehicle_air_data_s airdata;
	void *const data[2];

public:
	TeleRpmTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status), ORB_ID(vehicle_air_data) }, 2)
	, data { &batt, &airdata }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_RPM *rpm = &item->rpm;

		if (topicsUpdated()) {
			getSubscriptionData(data);
		}

		rpm->identifier = TELE_DEVICE_RPM;
		rpm->sID = TELE_FRAMETYPE_SID;

		rpm->microseconds = 0xffff;
		rpm->volts = leToBe16(batt.voltage_filtered_v * 100.0f);
		rpm->temperature = leToBe16(airdata.baro_temp_celcius * 1.8f + 32.0f);
		rpm->dBm_A = 0x7f;
		rpm->dBm_B = 0x7f;
		rpm->spare[0] = 0xffff;
		rpm->spare[1] = 0xffff;

		return true;
	}
};


class TeleQosTelemetryItem : public SpektrumTelemetryItem
{
#if defined(ADC_SCALED_V5_SENSE)
	system_power_s power;
	void *const data[1];
#endif

public:
	TeleQosTelemetryItem()
#if defined(ADC_SCALED_V5_SENSE)
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(system_power) }, 1)
	, data { &power }
#else
		: SpektrumTelemetryItem((orb_item_t[]) {}, 0)
#endif
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_QOS *rpm = &item->qos;

#if defined(ADC_SCALED_V5_SENSE)

		if (topicsUpdated()) {
			getSubscriptionData(data);
		}

#endif

		rpm->identifier = TELE_DEVICE_QOS;
		rpm->sID = TELE_FRAMETYPE_SID;
		rpm->A = 0xffff;
		rpm->B = 0xffff;
		rpm->L = 0xffff;
		rpm->R = 0xffff;
		rpm->F = 0xffff;
		rpm->H = 0xffff;
#if defined(ADC_SCALED_V5_SENSE)
		rpm->rxVoltage = leToBe16(power.voltage5v_v * 100.0f);
#else
		rpm->rxVoltage = 0xffff;
#endif

		return true;
	}
};

class FlightPackTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	void *const data[1];

public:
	FlightPackTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status) }, 1)
	, data { &batt }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_FP_MAH *fp = &item->fpMAH;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		fp->identifier = TELE_DEVICE_FP_MAH;
		fp->sID = TELE_FRAMETYPE_SID;

		uint16_t temperature = (uint16_t)(batt.temperature * 10.0f);

		fp->current_A = (uint16_t)(batt.current_filtered_a * 10.0f);
		fp->chargeUsed_A = (uint16_t)(batt.discharged_mah);
		fp->temp_A = ((temperature == 0) ? 0xFFFF : temperature);

		fp->current_B = 0x7FFF;
		fp->chargeUsed_B = 0x7FFF;
		fp->temp_B = 0xFFFF;

		fp->spare = 0xffff;

		return true;
	}
};

class RxMahTelemetryItem : public SpektrumTelemetryItem
{
	battery_status_s batt;
	void *const data[1];

public:
	RxMahTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(battery_status) }, 1)
	, data { &batt }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_RX_MAH *fp = &item->rxMAH;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		fp->identifier = TELE_DEVICE_RX_MAH;
		fp->sID = TELE_FRAMETYPE_SID;

		fp->current_A = (uint16_t)(batt.current_filtered_a * 100.0f);
		fp->chargeUsed_A = (uint16_t)(batt.discharged_mah * 10.0f);
		fp->volts_A = (uint16_t)(batt.voltage_filtered_v * 100.0f);

		fp->current_B = 0x7FFF;
		fp->chargeUsed_B = 0x7FFF;
		fp->chargeUsed_B = 0x7FFF;

		return true;
	}
};

class AttMagTelemetryItem : public SpektrumTelemetryItem
{
	vehicle_attitude_s att;
	vehicle_magnetometer_s mag;
	vehicle_local_position_s pos;
	void *const data[3];

public:
	AttMagTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(vehicle_attitude), ORB_ID(vehicle_magnetometer), ORB_ID(vehicle_local_position) },
	3)
	, data { &att, &mag, &pos }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_ATTMAG *attMag = &item->attMag;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		Quatf q(att.q);
		Eulerf euler(q);

		float rad2deg = (float)(180.0 / M_PI);

		attMag->identifier = TELE_DEVICE_ATTMAG;
		attMag->sID = TELE_FRAMETYPE_SID;

		attMag->attRoll = (int16_t)(rad2deg * euler.phi() * 10.0f);                     // pulse leading edges
		attMag->attPitch = (int16_t)(rad2deg * euler.theta() * 10.0f);   // vbat is in units of 0.1V
		attMag->attYaw = (int16_t)(rad2deg * euler.psi() * 10.0f);                     // temperature
		attMag->magX = mag.magnetometer_ga[0] * 10000.0f;                     // temperature
		attMag->magY = mag.magnetometer_ga[1] * 10000.0f;                     // temperature
		attMag->magZ = mag.magnetometer_ga[2] * 10000.0f;                     // temperature
		attMag->heading = (int16_t)(rad2deg * pos.yaw * 10.0f);                     // temperature

		return true;
	}
};

class GpsLocTelemetryItem : public SpektrumTelemetryItem
{
	vehicle_gps_position_s gps;
	void *const data[1];
public:
	GpsLocTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(vehicle_gps_position) }, 1)
	, data { &gps }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_GPS_LOC *gpsloc = &item->gpsloc;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		float rad2deg = (float)(180.0 / M_PI);

		gpsloc->identifier = TELE_DEVICE_GPS_LOC;
		gpsloc->sID = TELE_FRAMETYPE_SID;

		gpsloc->altitudeLow = binToBcd16(gps.alt / 100);
		gpsloc->latitude = binToBcd32(gps.lat / 10);
		gpsloc->longitude = binToBcd32(gps.lon / 10);
		gpsloc->course = binToBcd16((uint16_t)(rad2deg * gps.cog_rad * 10.0f));
		gpsloc->HDOP = binToBcd8((uint8_t)(gps.hdop * 10.0f));
		gpsloc->GPSflags =
			((abs(gps.lon) >= 100000000UL) ? GPS_INFO_FLAGS_LONGITUDE_GREATER_99 : 0) |
			((gps.lon > 0) ? GPS_INFO_FLAGS_IS_EAST : 0) |
			((gps.lat > 0) ? GPS_INFO_FLAGS_IS_NORTH : 0) |
			((gps.alt < 0) ? GPS_INFO_FLAGS_NEGATIVE_ALT : 0) |
			((gps.fix_type > 2) ? GPS_INFO_FLAGS_3D_FIX : 0) |
			((gps.fix_type > 1) ? GPS_INFO_FLAGS_GPS_FIX_VALID : 0) |
			((gps.fix_type > 0) ? GPS_INFO_FLAGS_GPS_DATA_RECEIVED : 0);

		return true;
	}
};

class GpsStatTelemetryItem : public SpektrumTelemetryItem
{
	vehicle_gps_position_s gps;
	void *const data[1];
public:
	GpsStatTelemetryItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(vehicle_gps_position) }, 1)
	, data { &gps }
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_GPS_STAT *gpsstat = &item->gpsstat;

		if (!topicsUpdated()) {
			return false;
		}

		getSubscriptionData(data);

		gpsstat->identifier = TELE_DEVICE_GPS_STATS;
		gpsstat->sID = TELE_FRAMETYPE_SID;

		gpsstat->speed = binToBcd16((uint16_t)(gps.vel_m_s * 1.943844f * 10.0f));
		time_t time_gps = gps.time_utc_usec / 1000000ULL;
		struct tm tm_gps;
		gmtime_r(&time_gps, &tm_gps);
		gpsstat->UTC = (binToBcd8(tm_gps.tm_hour) << 24) | (binToBcd8(tm_gps.tm_min) << 16) | (binToBcd8(
					tm_gps.tm_sec) << 8) | 0;
		gpsstat->numSats = binToBcd8(gps.satellites_used);
		gpsstat->altitudeHigh = binToBcd8(gps.alt / 100000);

		return true;
	}
};

class LogMessageItem : public SpektrumTelemetryItem
{
	log_message_s message;
	void *const data[1];

	const char *m_text;
	uint8_t m_bytes_left;
	int8_t m_line;
public:
	LogMessageItem()
		: SpektrumTelemetryItem((orb_item_t[]) { ORB_ID(log_message) }, 1)
	, data { &message }
	, m_bytes_left(0)
	, m_line(0)
	{}

	virtual bool update(UN_TELEMETRY *item)
	{
		STRU_TELE_TEXTGEN *textgen = &item->textgen;

		if (topicsUpdated()) {
			getSubscriptionData(data);
			m_bytes_left = strlen((char *)message.text);
			m_line = -1;
			m_text = (char *)message.text;
		}

		if (m_bytes_left == 0) {
			return false;
		}

		textgen->identifier = TELE_DEVICE_TEXTGEN;
		textgen->sID = TELE_FRAMETYPE_SID;

		textgen->lineNumber = m_line;

		if (m_line < 0) {
			textgen->text[0] = '\0';

		} else if (m_line == 0) {
			snprintf(textgen->text, sizeof(textgen->text), "PX4: %d", message.severity);

		} else if (m_line <= 8) {
			if (m_bytes_left >= sizeof(textgen->text)) {
				memcpy(textgen->text, m_text, sizeof(textgen->text));
				m_bytes_left -= sizeof(textgen->text);
				m_text += sizeof(textgen->text);

			} else {
				memcpy(textgen->text, m_text, m_bytes_left);
				textgen->text[m_bytes_left] = '\0';
				m_bytes_left = 0;
			}

		} else {
			m_bytes_left = 0;
			return false;
		}

		m_line++;

		return true;
	}
};

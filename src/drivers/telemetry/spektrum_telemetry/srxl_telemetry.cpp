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
 * @file dsm_telemety.cpp
 *
 * Telemetry handling for Spektrum DSM/SRXL
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 */

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <math.h>

#include <board_config.h>
// #include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
// #include <drivers/drv_rc_input.h>
// #include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_defines.h>
#include <matrix/math.hpp>

#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/log_message.h>

#include <lib/rc/dsm.h>
#include <lib/rc/srxl.hpp>
#include "srxl_telemetry.hpp"
#include "messages.hpp"

using namespace time_literals;

static SrxlEncoder srxl;

// #define DSM_IGNORE_REQUIRED_ITEMS

#if !defined(DSM_IGNORE_REQUIRED_ITEMS)
static SpektrumTelemetryItem *const _requiredTelemetryItems[] = {
	new TeleRpmTelemetryItem(),
	new TeleQosTelemetryItem(),
};
#endif

static SpektrumTelemetryItem *const _telemetryItems[] = {
#if defined(DSM_IGNORE_REQUIRED_ITEMS)
	new TeleRpmTelemetryItem(),
	new TeleQosTelemetryItem(),
#endif
	new FlightPackTelemetryItem(),
#if defined(DSM_ATT_MAG_TELEMETRY)
	new AttMagTelemetryItem(),
#endif
#if defined(DSM_GPS_TELEMETRY)
	new GpsLocTelemetryItem(),
	new GpsStatTelemetryItem(),
#endif
	new LogMessageItem(),
};

#define TELEMETRY_ITEM_COUNT (sizeof(_telemetryItems) / sizeof(SpektrumTelemetryItem*))


SRXLTelemetry::DSMTelemetry(int fd):
	_fd(fd)
{
#if !defined(DSM_IGNORE_REQUIRED_ITEMS)
	_requiredTelemetryItems[0]->start();
	_requiredTelemetryItems[1]->start();
#endif
	for (size_t i = 0; i < TELEMETRY_ITEM_COUNT; i++) {
		_telemetryItems[i]->start();
	}
}

bool SRXLTelemetry::update()
{
	static uint32_t telemetry_frame_count = 0;
	telemetry_frame_t frame;

#if !defined(DSM_IGNORE_REQUIRED_ITEMS)

	switch (telemetry_frame_count % 3) {
	case 0:
		_requiredTelemetryItems[0]->update(&frame.telemetry);
		break;

	case 1:
		_requiredTelemetryItems[1]->update(&frame.telemetry);
		break;

	default:
	case 2:
		for (size_t i = 0; i < TELEMETRY_ITEM_COUNT; i++) {
			SpektrumTelemetryItem *item = _telemetryItems[(telemetry_frame_count / 3 + i) % TELEMETRY_ITEM_COUNT];

			if (item->update(&frame.telemetry)) {
				break;
			}
		}

		break;
	}

#else

	for (size_t i = 0; i < TELEMETRY_ITEM_COUNT; i++) {
		SpektrumTelemetryItem *item = _telemetryItems[(telemetry_frame_count + i) % TELEMETRY_ITEM_COUNT];

		if (item->update(&frame.telemetry)) {
			break;
		}
	}

#endif

	srxl.setPayload(&frame, sizeof(frame));
	telemetry_frame_count++;
	return true;
}


void SRXLTelemetry::set_uart_single_wire(int uart, bool single_wire)
{
	if (ioctl(uart, TIOCSSINGLEWIRE, single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL |
			SER_SINGLEWIRE_PULLDOWN) : 0) < 0) {
		PX4_WARN("setting TIOCSSINGLEWIRE failed");
	}
}

bool SRXLTelemetry::update( const hrt_abstime & now )
{
	const int update_rate_hz = 10;

	if (now - _last_update <= 1_s / (update_rate_hz * num_data_types)) {
		return false;
	}

	bool sent = false;
	switch(_next_type){
	case 0:
	case 1:
	case 2:
	case 3:
	default:
		sent = update();
		break;
	}

	_last_update = now;
	_next_type = (_next_type + 1) & num_data_types;

	return sent;
}


size_t SRXLTelemetry::write_next(int fd)
{
	size_t total = 0;
	SrxlBuffer *srxlBuffer;
	size_t srxlBufLen = srxl.getFrame(&srxlBuffer);

	while (srxlBufLen > 0) {
		ssize_t written = write(fd, &(*srxlBuffer)[total], srxlBufLen);
		srxlBufLen -= written;
		total += written;
	}

	return total;
}

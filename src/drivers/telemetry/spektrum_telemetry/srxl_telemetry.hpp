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
 * @file dsm_telemetry.h
 *
 * Helper class for DSM/SRXL telemetry support
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 * @author Daniel Williams <equipoise@gmail.com>
 */

#pragma once

#include <cstdint>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <px4_platform_common/px4_config.h>

// #include <drivers/device/device.h>
// #include <drivers/drv_rc_input.h>
// #include <drivers/drv_pwm_output.h>
// #include <drivers/drv_sbus.h>

class SRXLTelemetry
{
public:

	/// \brief enforce using the construct with a serial-port file-descriptior
	SRXLTelemetry() = delete;

	/**
	 * @param fd file descriptor for the UART to use. It is expected to be configured
	 * already.
	 */
	SRXLTelemetry(int _fd);

	~SRXLTelemetry() = default;


	/**
	 * Send telemetry data. Call this regularly (i.e. at 100Hz), it will automatically
	 * limit the sending rate.
	 * @return true if new data sent
	 */
	bool update();
	bool update( const hrt_abstime & now );

	size_t write_next(int fd);

	// copied from frsky_telemetry module.  Not sure if it'll be useful, yet.
	// usage:
	//      s
	void set_uart_single_wire(int uart, bool single_wire);

private:
	hrt_abstime _last_update{0};

	/// number of different telemetry data types
	static constexpr int num_data_types{4}; // not sure what this number should actually be....
	int _next_type{0};

	int _fd;

};

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

#include <stdint.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>
#include <px4_defines.h>

class DSMTelemetry
{
public:

	DSMTelemetry() = delete;
	/**
	 * @param uart_fd file descriptor for the UART to use. It is expected to be configured
	 * already.
	 */
	DSMTelemetry(int uart_fd);
	~DSMTelemetry() = default;

	/**
	 * Send telemetry data. Call this regularly (i.e. at 100Hz), it will automatically
	 * limit the sending rate.
	 * @return true if new data sent
	 */
	bool update( const hrt_abstime & now );

private: // deprecated functions
	void dsm_update_telemetry(void);
	void dsm_init_telemetry(void);

	// dead code: this function is never called
	// size_t srxl_write_next(int dsm_fd);

private:
	hrt_abstime _last_update{0};

	/// number of different telemetry data types
	static constexpr int num_data_types{4}; // not sure what this number should actually be....
	int _next_type{0};

	int _uart_fd;

};

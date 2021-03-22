/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 * @file spektrum_telemetry.cpp
 * @author Kurt Kiefer <kekiefer@gmail.com>
 * @author Daniel Williams <equipoise@gmail.com>
 *
 *
 */


#include <board_config.h>
#include <px4_defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/input_rc.h>

#include <lib/rc/sbus.h>
#include <lib/rc/dsm.h>
#include "dsm_telemetry.h"

using namespace time_literals;

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int spektrum_task;
static int uart;
static volatile bool should_bind = false;

static unsigned long int sentPackets = 0;
/* Default values for arguments */
const char *rc_device_name = NULL;

/* functions */
static void usage(void);
static int spektrum_telemetry_thread_main(int argc, char *argv[]);

extern "C" __EXPORT int spektrum_telemetry_main(int argc, char *argv[]);

/**
 * Print command usage information
 */
static void usage()
{
	PRINT_MODULE_DESCRIPTION("Spektrum Telemetry support.");

	PRINT_MODULE_USAGE_NAME("spektrum_telemetry", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', RC_SERIAL_PORT, "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
}

/**
 * The daemon thread.
 */
static int spektrum_telemetry_thread_main(int argc, char *argv[])
{
	rc_device_name = RC_SERIAL_PORT;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			rc_device_name = myoptarg;
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	/* Open UART */
	uart = open(rc_device_name, O_RDWR | O_NOCTTY);

	if (uart <= 0) {
		rc_device_name = NULL;
		return -1;
	}

	dsm_proto_init();
	int ret = dsm_config(uart, board_supports_single_wire(RC_UXART_BASE));

	if (ret != 0) {
		close(uart);
		rc_device_name = NULL;
		return -1;
	}

	dsm_init_telemetry();
	dsm_update_telemetry();

	thread_running = true;

	PX4_INFO("starting Spektrum DSM telemetry");

	// wakeup source
	struct pollfd fds[1] = {};
	fds[0].fd = uart;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {

		if (should_bind) {
			ssize_t bret = dsm_bind_srxl(uart);
			PX4_INFO("bind returns %d", bret);
			should_bind = false;
		}

		uint8_t rcs_buf[SBUS_BUFFER_SIZE] {};

		// wait for up to 20ms for data
		int pret = ::poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// error in poll, exit
		if (pret < 0) {
			break;
		}

		// timed out - periodic check for thread_should_exit
		if (pret == 0) {
			continue;
		}

		if (1) { //fds[0].revents & POLLIN) {

			const hrt_abstime cycle_timestamp = hrt_absolute_time();

			int newBytes = ::read(uart, &rcs_buf[0], SBUS_BUFFER_SIZE);

			if (newBytes <= 0) {
				PX4_INFO("error reading uart");
				continue;
			}

			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
			uint16_t raw_rc_count{};
			uint8_t phase;
			bool dsm_11_bit;
			bool rc_updated;

			// parse new data
			rc_updated = dsm_parse(cycle_timestamp, &rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count,
					       &dsm_11_bit, nullptr, nullptr, &phase, input_rc_s::RC_INPUT_MAX_CHANNELS);

			if (rc_updated && phase == 0) {
				if (srxl_write_next(uart) > 0) {
					dsm_update_telemetry();
					sentPackets++;
				}
			}
		}
	}

	close(uart);

	rc_device_name = NULL;

	thread_running = false;

	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int spektrum_telemetry_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_ERR("missing command");
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("spektrum_telemetry already running");
			return 0;
		}

		thread_should_exit = false;
		spektrum_task = px4_task_spawn_cmd("spektrum_telemetry",
						   SCHED_DEFAULT,
						   SCHED_PRIORITY_SLOW_DRIVER, 1350,
						   spektrum_telemetry_thread_main,
						   (char *const *)argv);

		while (!thread_running) {
			usleep(200);
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			PX4_WARN("spektrum_telemetry already stopped");
			return 0;
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(1000000);
			PX4_INFO(".");
		}

		PX4_INFO("terminated.");
		rc_device_name = NULL;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {

			PX4_INFO("running");
			PX4_INFO("port: %s", rc_device_name);
			PX4_INFO("packets sent: %d", sentPackets);

			return 0;

		} else {
			PX4_INFO("not running");
			return 0;
		}
	}

	if (!strcmp(argv[1], "bind")) {
		should_bind = true;
		return 0;
	}

	PX4_ERR("unrecognized command");
	usage();
	return 0;
}

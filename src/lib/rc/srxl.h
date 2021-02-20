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
 * @file srxl.h
 *
 * RC protocol definition for Spektrum SRXL
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 * @author Daniel Williams <equipoise@gmail.com>
 */

#pragma once

#define SRXL_SPEKTRUM_HEADER		0xA5
#define SRXL_VERSION_TELEMETRY		0x80
#define SRXL_VERSION_BIND_INFO		0x41
#define SRXL_MAX_LENGTH				64

typedef uint8_t SrxlBuffer[SRXL_MAX_LENGTH];

// // Source:
// // https://github.com/SpektrumRC/SRXL2/blob/master/Docs/SRXL2%20Specification.pdf
// #define SRXL2_HEADER_ID			0xA6
// #define SRXL2_PACKET_MIN_LENGTH		5
// #define SRXL2_PACK_MAX_LENGTH		80
//
// typedef uint8_t Srxl2Buffer[SRXL_MAX_LENGTH];

class SrxlEncoder
{

	typedef struct srxl_frame_header_s {
		uint8_t header;
		uint8_t version;
		uint8_t length;
		uint8_t payload[];
	} srxl_frame_header_t;

public:
	void setPayload(void *payload, size_t length, uint8_t version = SRXL_VERSION_TELEMETRY);
	size_t getFrame(SrxlBuffer **buffer);

	static constexpr bool isSpektrumTelemetry(void *frame)
	{
		return ((srxl_frame_header_t *)frame)->header == SRXL_SPEKTRUM_HEADER
		       && (((srxl_frame_header_t *)frame)->version == SRXL_VERSION_TELEMETRY
			   || ((srxl_frame_header_t *)frame)->version == SRXL_VERSION_BIND_INFO);
	}

	static constexpr size_t getFrameLength(void *frame)
	{
		return ((srxl_frame_header_t *)frame)->length;
	}

private:

	SrxlBuffer srxlBuf[SRXL_MAX_LENGTH];
	size_t srxlBufLen = 0;
};

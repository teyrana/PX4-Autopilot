/****************************************************************************
 *
 *	Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file dsm.h
 *
 * RC protocol definition for Spektrum RC
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Daniel Williams <equipoise@gmail.com>
 *
 * ## Sources
 * .1. [ Spektrum Remote Receiver Interfacing. Rev A, 2016/04/12 ]( https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf )
 * .2. [ Spektrum Remote Receiver Interfacing. Rev G, 9.1 ]( https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/Remote%20Receiver%20Interfacing.pdf )
 */

#pragma once

#include <stdint.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_config.h>


class DSMDecoder {
// ====== ====== ====== ====== Public API Methods ====== ====== ====== ======
public:
	DSMDecoder() = delete;
	DSMDecoder(const char *device);

	~DSMDecoder();

	// this is the primary external interface
	int 	input( uint16_t *num_values, uint8_t *n_bytes, uint8_t **bytes);

	int	parse(const uint64_t now, const unsigned len );

#if defined(SPEKTRUM_POWER)
	void	bind(uint16_t cmd, int pulses);
#endif

private:
	int decode(hrt_abstime rx_time, uint16_t* num_values);

	bool decode_channel(uint16_t raw, unsigned shift, uint8_t &channel, uint16_t &value);

	/**
	* Attempt to guess if receiving 10 or 11 bit channel values
	*
	* @param[in] reset true=reset the 10/11 bit state to unknown
	*/
	bool guess_format(bool reset);

// ====== ====== ====== ====== Define types and constants ====== ====== ====== ======
public: //private:  // should be private, but public during refactor
	constexpr static size_t frame_size = 16;  ///< DSM Frame size in bytes
	constexpr static size_t channels_per_frame = 7;   ///< no source found
	constexpr static size_t channels_per_receiver = 20; ///< no source found
	constexpr static size_t work_buffer_size = 24;  // arbitrary size.

	enum DSM_DECODE_STATE {
		DSM_DECODE_STATE_DESYNC = 0,
		DSM_DECODE_STATE_SYNC
	};

// ====== ====== Define Telemetry Types ====== ======
private:
#pragma pack(push, 1)
	/// \brief describes the 16-byte packet that this codec decodes
	typedef struct {
		union {
			struct {
				uint8_t fades;  ///< aka dropped frames
				uint8_t system;
			} internal;
			struct {
				uint16_t fades;  ///< aka dropped frames
			} external;
		};
		uint16_t servo[channels_per_frame];
	} message_t;
#pragma pack(pop)

       // verify that the struct is packing to minimal size, and therefore
       // that it will correctly decode the binary packets
       static_assert( frame_size == sizeof(message_t), "DSM Telemetry frame is not the expected size!");

       // possible values for the `message_t::internal::system` field
       enum SYSTEM_PROTOCOLS {
               protocol_DSM2_22MS_1024 = 0x01,
               protocol_DSM2_11MS_2048 = 0x12,
               protocol_DSMQ_22MS_2048 = 0xa2,
               protocol_DSMX_11MS_2048 = 0xb2
       };

       // Servo Field 1024 Mode
       // This format is used only by DSM2/22ms mode. All other modes use 2048 data.
       constexpr static uint16_t MASK_1024_CHANID = 0xFC00; ///< Bits 14-11 = Channel Id
       constexpr static uint16_t MASK_1024_SXPOS  = 0x03ff; ///< Bits 10-0: Servo Position

       // Servo Field 2048 Mode
       // This format is used by all protocols except DSM2/22ms mode.
       constexpr static uint16_t MASK_2048_PHASE  = 0x8000; ///< Bit 15 == Servo Phase
       constexpr static uint16_t MASK_2048_CHANID = 0x7800; ///< Bits 14-11 = Channel Id
       constexpr static uint16_t MASK_2048_SXPOS  = 0x07ff; ///< Bits 10-0: Servo Position

       // Channel Identifiers
       enum CHANNEL_IDENTIFIERS {
               Channel_Throttle = 0,
               Channel_Aileron = 1,
               Channel_Elevator = 2,
               Channel_Rudder = 3,
               Channel_Gear = 4,
               Channel_Aux1 = 5,
               Channel_Aux2 = 6,
               Channel_Aux3 = 7,
               Channel_Aux4 = 8,
               Channel_Aux5 = 9,
               Channel_Aux6 = 10,
               Channel_Aux7 = 11,
       } ;


// ====== ====== Define Bind Types ====== ======
private:
#pragma pack(push, 1)
typedef struct {
	uint8_t request;
	uint64_t guid;
	uint8_t type;
	uint32_t chip_id;
} dsm_bind_t;
#pragma pack(pop)

enum DSM_CMD {			/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES,
	DSM_CMD_BIND_REINIT_UART
};

	// Note: Internal vs External receivers:
	//	1. "Internal" -- defined as the primary or only receiver to communicate with the transmitter.
	//	2. "External" receiver -- a satellite or backup receiver, which takes its bind info from
	//	  the primary/"internal" receiver
	//							 Pulses     Mode      Protocol  Interval
	constexpr static uint16_t bind_pulse_count_int_DSMX_11 = 9;    //   Internal  DSMx      11ms
	constexpr static uint16_t bind_pulse_count_ext_DSMX_11 = 10;   //   External  DSMx      11ms

	// To put a receiver into bind mode, within 200ms of power application the host device needs to issue a
	// series of falling pulses. The number of pulses issued selects which bind types will be accepted from
	// the transmitter. Selecting the 11ms mode automatically allows either 11ms or 22ms protocols to be
	// used. Selecting DSMX will automatically allow DSM2 to be used if the transmitter is unable to
	// support DSMX. For this reason, we recommend that 11ms DSMX be used (9 (“internal”) or 10
	// (“external”) pulses). -- [1], p2, Section 7.2
	constexpr static uint16_t default_bind_pulses = bind_pulse_count_int_DSMX_11;


// ====== ====== ====== ====== define instance state properties ====== ====== ====== ======
private: /// instance properties
public: //private:  // should be private, but public during refactor
	// bool system_protocol;
	    // => 11ms vs 22ms
	    // => DSM2 vs DSMX

	int _fd = -1;				///< File handle to the DSM UART
	hrt_abstime _last_rx_time;		///< Timestamp when we last received data
	hrt_abstime _last_frame_time;		///< Timestamp for start of last valid dsm frame
	DSM_DECODE_STATE _decode_state = DSM_DECODE_STATE_DESYNC;
	bool _11_bit;

	// dsm_frame_t &dsm_frame = rc_decode_buf.dsm.frame;///< DSM_BUFFER_SIZE DSM dsm frame receive buffer
	// dsm_buf_t &dsm_buf = rc_decode_buf.dsm.buf;	///< DSM_BUFFER_SIZE DSM dsm frame receive buffer

	uint16_t _chan_buf[channels_per_receiver];
	unsigned _partial_frame_count;	///< Count of bytes received for current dsm frame
	unsigned _channel_shift;	///< Channel resolution, 0=unknown, 10=10 bit (1024), 11=11 bit (2048)
	unsigned _frame_drops;		///< Count of incomplete DSM frames
	uint16_t _chan_count;		///< DSM channel count
	uint16_t _last_valid_channel_count; ///< last valid DSM channel count
	uint8_t _rssi_percent;


typedef   uint8_t frame_buffer_t[DSMDecoder::frame_size];        ///< DSM dsm frame receive buffer
typedef   uint8_t work_buffer_t[DSMDecoder::frame_size * 2];      ///< Define working buffer
	// _frame_buf_t dsm_frame = rc_decode_buf.dsm.frame;  ///< DSM_BUFFER_SIZE DSM dsm frame receive buffer
	// _work_buf_t dsm_buf = rc_decode_buf.dsm.buf;       ///< DSM_BUFFER_SIZE DSM dsm frame receive buffer

	frame_buffer_t _receive_buffer;  ///< receive buffer
	work_buffer_t _frame_buffer;    ///< working frame buffer

};


__BEGIN_DECLS

#pragma pack(push, 1)
typedef  struct dsm_decode_t {
	DSMDecoder::frame_buffer_t frame;
	DSMDecoder::work_buffer_t buf;
} dsm_decode_t;
#pragma pack(pop)

__END_DECLS

#include <cstring>
#include "crc16.h"
#include "srxl.h"

void SrxlEncoder::setPayload(void *payload, size_t length, uint8_t version)
{
	srxl_frame_header_t *srxl = (srxl_frame_header_t *)srxlBuf;

	uint16_t crc = crc16((const uint8_t *)payload, length);

	srxl->header = SRXL_SPEKTRUM_HEADER;
	srxl->version = version;
	srxl->length = length + sizeof(srxl_frame_header_t) + sizeof(uint16_t);

	/* copy the completed buffer to the tranmission buffer */
	memcpy(srxl->payload, payload, length);

	uint16_t *srxlCrc = static_cast<uint16_t *>(payload) + length;
	*srxlCrc = crc;

	srxlBufLen = srxl->length;
}

size_t SrxlEncoder::getFrame(SrxlBuffer **buffer)
{
	*buffer = srxlBuf;
	return srxlBufLen;
}

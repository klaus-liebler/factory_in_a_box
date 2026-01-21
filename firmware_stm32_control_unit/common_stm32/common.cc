#include "common.hh"

namespace {

// Little-endian stores/loads (byte-wise to avoid unaligned access issues on STM32)
inline void store_le16(uint16_t v, uint8_t *buf, size_t off) {
	buf[off + 0] = static_cast<uint8_t>(v >> 0);
	buf[off + 1] = static_cast<uint8_t>(v >> 8);
}

inline void store_le32(uint32_t v, uint8_t *buf, size_t off) {
	buf[off + 0] = static_cast<uint8_t>(v >> 0);
	buf[off + 1] = static_cast<uint8_t>(v >> 8);
	buf[off + 2] = static_cast<uint8_t>(v >> 16);
	buf[off + 3] = static_cast<uint8_t>(v >> 24);
}

inline void store_le64(uint64_t v, uint8_t *buf, size_t off) {
	buf[off + 0] = static_cast<uint8_t>(v >> 0);
	buf[off + 1] = static_cast<uint8_t>(v >> 8);
	buf[off + 2] = static_cast<uint8_t>(v >> 16);
	buf[off + 3] = static_cast<uint8_t>(v >> 24);
	buf[off + 4] = static_cast<uint8_t>(v >> 32);
	buf[off + 5] = static_cast<uint8_t>(v >> 40);
	buf[off + 6] = static_cast<uint8_t>(v >> 48);
	buf[off + 7] = static_cast<uint8_t>(v >> 56);
}

inline uint16_t load_le16(const uint8_t *buf, size_t off) {
	return static_cast<uint16_t>(buf[off + 0]) |
		   static_cast<uint16_t>(buf[off + 1]) << 8;
}

inline uint32_t load_le32(const uint8_t *buf, size_t off) {
	return static_cast<uint32_t>(buf[off + 0]) |
		   (static_cast<uint32_t>(buf[off + 1]) << 8) |
		   (static_cast<uint32_t>(buf[off + 2]) << 16) |
		   (static_cast<uint32_t>(buf[off + 3]) << 24);
}

inline uint64_t load_le64(const uint8_t *buf, size_t off) {
	return static_cast<uint64_t>(buf[off + 0]) |
		   (static_cast<uint64_t>(buf[off + 1]) << 8)  |
		   (static_cast<uint64_t>(buf[off + 2]) << 16) |
		   (static_cast<uint64_t>(buf[off + 3]) << 24) |
		   (static_cast<uint64_t>(buf[off + 4]) << 32) |
		   (static_cast<uint64_t>(buf[off + 5]) << 40) |
		   (static_cast<uint64_t>(buf[off + 6]) << 48) |
		   (static_cast<uint64_t>(buf[off + 7]) << 56);
}

} // namespace

size_t byteBuf2hexCharBuf(char* charBuf, size_t charBufLen, const uint8_t* byteBuf, size_t byteBufLen){
	if (!charBuf || charBufLen == 0) return 0;
	// Needed: "[" + n*("XX ") + "]" + NUL. We truncate if buffer is too small.
	char* p = charBuf;
	char* end = charBuf + charBufLen;

	if (p < end) *p++ = '['; else return 0;

	for (size_t i = 0; i < byteBufLen && (p + 3) < end; ++i) {
		// Write two hex chars plus a space; leave room for closing bracket and NUL
		int written = snprintf(p, static_cast<size_t>(end - p), "%02X ", byteBuf[i]);
		if (written <= 0) break;
		p += static_cast<size_t>(written);
	}

	if (p < end) {
		*p++ = ']';
	} else {
		// No room for closing bracket/NUL
		return charBufLen; // already filled to end
	}

	if (p < end) {
		*p = '\0';
	} else {
		*(end - 1) = '\0';
	}
	return static_cast<size_t>(p - charBuf);
}

bool GetBitInU8Buf(const uint8_t *buf, size_t offset, size_t bitIdx)
{
	uint8_t b = buf[offset + (bitIdx >> 3)];
	uint32_t bitpos = bitIdx & 0b111;
	return b & (1 << bitpos);
}

void WriteI8(int8_t value, uint8_t *buffer, uint32_t offset)
{
	uint8_t *ptr1 = (uint8_t *)&value;
	*(buffer + offset) = *ptr1;
}

void WriteI16(int16_t value, uint8_t *buffer, uint32_t offset)
{
	store_le16(static_cast<uint16_t>(value), buffer, offset);
}

void WriteI32(int32_t value, uint8_t *buffer, uint32_t offset)
{
	store_le32(static_cast<uint32_t>(value), buffer, offset);
}

void WriteI64(int64_t value, uint8_t *buffer, uint32_t offset)
{
	store_le64(static_cast<uint64_t>(value), buffer, offset);
}

int16_t ParseI16(const uint8_t *const buffer, uint32_t offset)
{
	int16_t step;
	uint8_t *ptr1 = (uint8_t *)&step;
	uint8_t *ptr2 = ptr1 + 1;
	*ptr1 = *(buffer + offset);
	*ptr2 = *(buffer + offset + 1);
	return step;
}

int32_t ParseI32(const uint8_t *const buffer, uint32_t offset)
{
	return static_cast<int32_t>(load_le32(buffer, offset));
}

void WriteU8(uint8_t value, uint8_t *buffer, uint32_t offset)
{
	*(buffer + offset) = value;
}

void WriteU16(uint16_t value, uint8_t *buffer, uint32_t offset)
{
	store_le16(value, buffer, offset);
}

void WriteU32(uint32_t value, uint8_t *buffer, uint32_t offset)
{
	store_le32(value, buffer, offset);
}

uint8_t ParseU8(const uint8_t *const buffer, uint32_t offset){
	return buffer[offset];
}

uint16_t ParseU16(const uint8_t *const buffer, uint32_t offset)
{
	return load_le16(buffer, offset);
}

uint32_t ParseU32(const uint8_t *const buffer, uint32_t offset)
{
	return load_le32(buffer, offset);
}

uint64_t ParseU64(const uint8_t *const buffer, uint32_t offset)
{
	return load_le64(buffer, offset);
}

float ParseF32(const uint8_t *const buffer, uint32_t offset)
{
	float value;
	uint8_t *ptr0 = (uint8_t *)&value;
	uint8_t *ptr1 = ptr0 + 1;
	uint8_t *ptr2 = ptr0 + 2;
	uint8_t *ptr3 = ptr0 + 3;
	*ptr0 = *(buffer + offset + 0);
	*ptr1 = *(buffer + offset + 1);
	*ptr2 = *(buffer + offset + 2);
	*ptr3 = *(buffer + offset + 3);
	return value;
}

void WriteI16_BigEndian(int16_t value, uint8_t *buffer, uint32_t offset)
{
	uint16_t v = static_cast<uint16_t>(value);
	buffer[offset + 0] = static_cast<uint8_t>(v >> 8);
	buffer[offset + 1] = static_cast<uint8_t>(v >> 0);
}

int16_t ParseI16_BigEndian(const uint8_t *const buffer, uint32_t offset)
{
	int16_t step;
	uint8_t *ptr1 = (uint8_t *)&step;
	uint8_t *ptr2 = ptr1 + 1;
	*ptr2 = *(buffer + offset);
	*ptr1 = *(buffer + offset + 1);
	return step;
}

void WriteU16_BigEndian(uint16_t value, uint8_t *buffer, size_t offset)
{
	buffer[offset + 0] = static_cast<uint8_t>(value >> 8);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 0);
}

void WriteU32_BigEndian(uint32_t value, uint8_t *buffer, size_t offset)
{
	buffer[offset + 0] = static_cast<uint8_t>(value >> 24);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 16);
	buffer[offset + 2] = static_cast<uint8_t>(value >> 8);
	buffer[offset + 3] = static_cast<uint8_t>(value >> 0);
}

uint16_t ParseU16_BigEndian(const uint8_t *const buffer, size_t offset)
{
	uint16_t step;
	uint8_t *ptr1 = (uint8_t *)&step;
	uint8_t *ptr2 = ptr1 + 1;
	*ptr2 = *(buffer + offset);
	*ptr1 = *(buffer + offset + 1);
	return step;
}

uint32_t ParseU32_BigEndian(const uint8_t *const buffer, size_t offset)
{
	uint32_t step;
	uint8_t *ptr1 = (uint8_t *)&step;
	uint8_t *ptr2 = ptr1 + 1;
	uint8_t *ptr3 = ptr1 + 2;
	uint8_t *ptr4 = ptr1 + 3;
	*ptr4 = *(buffer + offset);
	*ptr3 = *(buffer + offset + 1);
	*ptr2 = *(buffer + offset + 2);
	*ptr1 = *(buffer + offset + 3);
	return step;
}




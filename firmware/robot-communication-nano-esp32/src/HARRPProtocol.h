#ifndef HARRP_PROTOCOL_H
#define HARRP_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Simplified HRP2, 2026-11-13. All multibyte integers are little-endian.
// No source, destination, operation, or version field. Payloads stay opaque.
namespace HRP2 {
constexpr uint8_t MAGIC[4] = {'H', 'R', 'P', '2'};
enum Dataset : uint8_t { ARM = 0x00, FINGER = 0x01, SENSOR = 0x02, ERROR = 0x03 };
constexpr uint8_t MAX_DATASET_ID = 0x0F;
constexpr size_t HEADER_SIZE = 7;
constexpr size_t CRC_SIZE = 4;
constexpr size_t METADATA_SIZE = 12;
constexpr size_t MAX_PAYLOAD = 256;
constexpr size_t MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;
constexpr size_t ARM_PAYLOAD = METADATA_SIZE + 76;
constexpr size_t FINGER_PAYLOAD = METADATA_SIZE + 24;
constexpr size_t ERROR_PAYLOAD = 1;
constexpr uint8_t MAX_ERROR_CODE = 99;

struct Frame {
  uint8_t bytes[MAX_FRAME] = {};
  size_t length = 0;
};

enum class Validation : uint8_t {
  Valid, BadMagic, InvalidId, ReservedId, BadLength, BadCrc, InvalidError
};

inline uint16_t read16(const uint8_t* p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
inline uint32_t read32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline void write16(uint8_t* p, uint16_t value) {
  p[0] = uint8_t(value);
  p[1] = uint8_t(value >> 8);
}
inline void write32(uint8_t* p, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) p[i] = uint8_t(value >> (8 * i));
}
inline uint32_t crc32(const uint8_t* bytes, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
  }
  return crc ^ 0xFFFFFFFFu;
}

// Reserved IDs are in range but unsupported. No storage is allocated for them.
inline Validation validateHeader(const uint8_t* header) {
  if (memcmp(header, MAGIC, sizeof(MAGIC)) != 0) return Validation::BadMagic;
  uint8_t id = header[4];
  if (id > MAX_DATASET_ID) return Validation::InvalidId;
  if (id > ERROR) return Validation::ReservedId;
  size_t length = read16(header + 5);
  if (length > MAX_PAYLOAD) return Validation::BadLength;
  if (id == ARM && length != ARM_PAYLOAD) return Validation::BadLength;
  if (id == FINGER && length != FINGER_PAYLOAD) return Validation::BadLength;
  if (id == SENSOR && length < METADATA_SIZE) return Validation::BadLength;
  if (id == ERROR && length != ERROR_PAYLOAD) return Validation::BadLength;
  return Validation::Valid;
}

inline Validation validate(const uint8_t* bytes, size_t length) {
  if (length < HEADER_SIZE + CRC_SIZE || length > MAX_FRAME) return Validation::BadLength;
  Validation result = validateHeader(bytes);
  if (result != Validation::Valid) return result;
  size_t payload = read16(bytes + 5);
  if (length != HEADER_SIZE + payload + CRC_SIZE) return Validation::BadLength;
  if (crc32(bytes, length - CRC_SIZE) != read32(bytes + length - CRC_SIZE))
    return Validation::BadCrc;
  if (bytes[4] == ERROR && bytes[HEADER_SIZE] > MAX_ERROR_CODE)
    return Validation::InvalidError;
  return Validation::Valid;
}

// For producers/tests. The hub forwards received frames without re-encoding.
inline bool encode(Frame& output, uint8_t id, const uint8_t* payload, size_t length) {
  output.length = 0;
  if (length > MAX_PAYLOAD || (length != 0 && payload == nullptr)) return false;
  memcpy(output.bytes, MAGIC, sizeof(MAGIC));
  output.bytes[4] = id;
  write16(output.bytes + 5, uint16_t(length));
  if (validateHeader(output.bytes) != Validation::Valid) return false;
  if (length != 0) memcpy(output.bytes + HEADER_SIZE, payload, length);
  size_t total = HEADER_SIZE + length + CRC_SIZE;
  write32(output.bytes + total - CRC_SIZE, crc32(output.bytes, total - CRC_SIZE));
  if (validate(output.bytes, total) != Validation::Valid) return false;
  output.length = total;
  return true;
}
static_assert(MAX_FRAME == 267 && ARM_PAYLOAD + 11 == 99 && FINGER_PAYLOAD + 11 == 47,
              "HRP2 allocation mismatch");
} // namespace HRP2
#endif

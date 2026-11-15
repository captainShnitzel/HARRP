# HARRP V2 — `HARRPProtocol.h` Developer Guide

**Revision:** 0.4  
**Prepared:** 2026-11-15  
**Reference implementation:** `firmware/robot-communication-nano-esp32/src/HARRPProtocol.h`  
**Implementation baseline:** Git commit `918fcd0` (`feature/esp32-comm-hub-v2`)  
**Companion specification:** [`HARRP_V2_Communication_Protocol.md`](HARRP_V2_Communication_Protocol.md)

> **Status:** Initial handoff guide. It describes the uploaded `HARRPProtocol.h` implementation, not an unimplemented proposed API. Arm/finger payload contents, sensor definitions, error-code assignments and endpoint restart policy may evolve. The *wire format and existing helper behavior* below are the current baseline.

## 1. Purpose and scope

`HARRPProtocol.h` is the small, **hardware-independent C++ implementation of the HRP2 packet format** shared across HARRP controllers. It is a header-only library: all functions are `inline`, so no companion `.cpp` file is required.

It provides:

- Dataset identifiers and packet-size constants.
- A fixed-capacity byte buffer (`HRP2::Frame`).
- Little-endian 16-bit and 32-bit integer accessors.
- CRC-32/ISO-HDLC calculation.
- Basic header and complete-frame validation.
- Encoding complete, CRC-protected frames from **already prepared** payload bytes.

It **does not** open UART or UDP sockets, discover devices, choose where packets go, parse Cartesian targets, command motors, interpret sensors, manage Wi-Fi, generate session tokens, track sequence numbers, impose freshness deadlines, or decide fault/stop actions. Those responsibilities belong to the relevant firmware or application.

### Intended use by controller

| Component | Use of the shared library | Additional responsibility outside the library |
|---|---|---|
| Wearable Nano ESP32 | Encode operator datasets; validate received feedback/errors | Produce targets; maintain its own session/sequence metadata |
| Robot-side Nano ESP32 | Validate received frames and forward complete bytes unchanged | Wi-Fi ↔ UART bridge, READY flow control, diagnostics/display |
| Teensy 4.1 | Validate/consume arm frames; forward finger frames; encode reports when needed | Real-time arm control, safety, downstream palm routing |
| Palm ESP32-S3 | Validate/consume finger frames; encode sensor/error frames | Finger control and sensor acquisition |
| NVIDIA Thor (future) | Encode/decode the same HRP2 wire format | Communication service separately from AI/planning components |

**Design rule:** A forwarding controller should preserve the original complete valid frame, including its session, sequence, timestamp and CRC, rather than re-encoding or modifying it.

## 2. Where to find and include the header

Initial repository location:

```text
HARRP/
└── firmware/
    └── robot-communication-nano-esp32/
        └── src/
            ├── HARRPProtocol.h
            └── main_nano_esp32_comm_hub_v2.cpp
```

For code in the same `src` directory:

```cpp
#include "HARRPProtocol.h"
```

The header includes only standard C/C++ headers (`<stdint.h>`, `<stddef.h>`, `<string.h>`); it does not depend on Arduino or ESP32 APIs. Copying the *same revision* into another controller's project, or supplying it through a shared include directory, is sufficient to compile the core helpers. Keep definitions synchronized between projects; do not independently alter the wire format on one device.

**Namespace:** All definitions are under `HRP2::`.

## 3. HRP2 frame layout

All multibyte integer fields are **little-endian**.

| Absolute byte offset | Size | Name | Notes |
|---|---:|---|---|
| `0–3` | 4 B | Magic | ASCII `HRP2`, hexadecimal `48 52 50 32` |
| `4` | 1 B | Dataset ID | `0x00`–`0x0F` allocated identifier space |
| `5–6` | 2 B | Payload length | Unsigned 16-bit integer; counts *only* payload bytes |
| `7` onward | N B | Payload | Opaque from the transport library's perspective |
| `7+N` through `10+N` | 4 B | CRC32 | CRC of header + payload; transmitted little-endian |

**Formula:** `total_frame_bytes = 7 + payload_length + 4 = 11 + payload_length`.

The library currently sets `MAX_PAYLOAD = 256` and `MAX_FRAME = 267`. No protocol version, message type, source address or destination address is present.

### Dataset ID assignments

| Hex ID | `HRP2` symbol | Dataset | Current status |
|---|---|---|---|
| `0x00` | `HRP2::ARM` | Arm target/control | Supported; fixed 88-byte payload including metadata |
| `0x01` | `HRP2::FINGER` | Finger target/control | Supported; fixed 36-byte payload including metadata |
| `0x02` | `HRP2::SENSOR` | Sensor feedback | Supported; variable payload, at least 12 bytes |
| `0x03` | `HRP2::ERROR` | Error report | Supported; exactly 1 payload byte, value `0..99` |
| `0x04–0x0F` | — | Reserved for future datasets | **Rejected by current library** as `ReservedId` |
| `0x10–0xFF` | — | Outside assigned ID space | Rejected as `InvalidId` |

**Important:** Although 16 IDs are available, *only the four defined IDs are accepted now*. Assigning a reserved ID later requires a coordinated library, documentation and firmware update.

### Control/feedback payload prefix

For `ARM`, `FINGER` and `SENSOR`, the first 12 bytes **inside the payload** are reserved for stream metadata:

| Payload offset | Width | Meaning |
|---|---:|---|
| `0–3` | 4 B | Session token (`uint32_t`) |
| `4–7` | 4 B | Sequence number (`uint32_t`) |
| `8–11` | 4 B | Producer timestamp (`uint32_t`, currently monotonic milliseconds by convention) |
| `12` onward | Variable | Application-defined bytes |

This metadata is **not part of the seven-byte common header**. The library verifies overall payload length, not whether session/sequence/timestamp values are sensible. Session restart acceptance, stream ordering and freshness belong to the final data consumer.

### Current size allocations

| Dataset | Metadata | Application bytes | Payload length | Entire frame |
|---|---:|---:|---:|---:|
| Arm `0x00` | 12 | 76 | 88 | 99 B |
| Finger `0x01` | 12 | 24 | 36 | 47 B |
| Sensor `0x02` | 12 | 0–244 | 12–256 | 23–267 B |
| Error `0x03` | None | 1 | 1 | 12 B |

The arm and finger sizes are **allocations**, not specifications of the actual float layout or mechanical units. Sensor fields are delegated to the student responsible for sensing and return data. Error numbers and their meaning remain open; the existing header only restricts the numeric code to `0..99`.

## 4. Complete function reference — exact C++ calls, arguments and return values

There are **exactly eight public functions** in the current `HARRPProtocol.h`. The examples below show the syntax **as used inside a program** rather than only giving function declarations. All names belong to the `HRP2::` namespace. Start with:

```cpp
#include "HARRPProtocol.h"
```

### 4.0 Before the functions: library-defined types, members and names

**This subsection is required reading.** `HARRPProtocol.h` contains not only eight functions, but also a namespace, two enum types, a packet-buffer structure, and constants. Some expressions in the function examples refer to **members defined by the library**, not generic variable names.

#### The actual `HRP2::Frame` definition

The header declares:

```cpp
struct Frame {
    uint8_t bytes[MAX_FRAME] = {};
    size_t length = 0;
};
```

It is in the `HRP2` namespace. A program creates an instance using:

```cpp
HRP2::Frame frame;
```

Here is the **exact identity** of every part:

| Expression | Defined by whom? | Meaning | Can the programmer rename it? |
|---|---|---|---|
| `HRP2` | Library | Namespace containing all protocol types, constants, and functions | No, not without changing the library |
| `HRP2::Frame` | Library | C++ structure holding a binary packet and its valid byte count | No, not without changing the library |
| `frame` | Calling program | Name chosen for one instance of `HRP2::Frame` | **Yes**, e.g. `incomingFrame` |
| `.bytes` | **Library (`Frame` member)** | Fixed-capacity `uint8_t` array storing the full wire packet | **No**, not without changing the structure and its users |
| `.length` | **Library (`Frame` member)** | `size_t` count of *valid* bytes in `.bytes` | **No**, not without changing the structure and its users |
| `HRP2::MAX_FRAME` | Library | Buffer capacity: `267` bytes | Reference by this symbol, not a locally invented value |

**Critical distinction:** `HRP2::Frame` is a **software buffer wrapper**, **not** a packed C++ representation of the wire header. Only `frame.bytes[0]` through `frame.bytes[frame.length - 1]` are sent over UART/UDP. The separate C++ member `frame.length` is **not transmitted**; it tells your program how much of the array contains a complete frame.

The 267 bytes in `.bytes` are capacity; they do not mean there are always 267 bytes of valid data. A newly created `Frame` has `length == 0` and a zero-initialized array. It is not yet a valid packet.

You can select a different **instance** name:

```cpp
HRP2::Frame receivedPacket;       // Programmer-selected instance name
receivedPacket.bytes[4];          // Library-defined member name .bytes
receivedPacket.length;            // Library-defined member name .length
```

The dot `.` is the C++ **member access operator**. `frame.bytes` means “the `bytes` member belonging to the particular object named `frame`.” `frame.bytes + 5` evaluates to the address of byte index 5 within that object's packet array, **not** a new copy of the data and not an automatic search for a field.

#### Which names are predefined, and which are your own?

| Item | Example | Meaning |
|---|---|---|
| Library-defined type | `HRP2::Frame` | The packet storage type |
| Library-defined type | `HRP2::Validation` | An enum class identifying a validation result |
| Library-defined dataset constants | `HRP2::ARM`, `HRP2::FINGER`, `HRP2::SENSOR`, `HRP2::ERROR` | Accepted dataset IDs `0x00` to `0x03` |
| Library-defined size constants | `HRP2::HEADER_SIZE`, `HRP2::CRC_SIZE`, `HRP2::METADATA_SIZE`, `HRP2::MAX_PAYLOAD`, `HRP2::MAX_FRAME`, `HRP2::ARM_PAYLOAD`, `HRP2::FINGER_PAYLOAD`, `HRP2::ERROR_PAYLOAD` | Sizes used throughout the library |
| Library-defined limits | `HRP2::MAX_DATASET_ID`, `HRP2::MAX_ERROR_CODE` | Upper bounds for IDs and error codes |
| Library-defined marker | `HRP2::MAGIC` | Four ASCII bytes spelling `HRP2` |
| Library-defined validation values | `HRP2::Validation::Valid`, `...::BadCrc`, etc. | Specific outcomes returned by validators |
| Programmer-defined local variable | `frame`, `payload`, `result`, `sequenceNumber` | Names chosen by the application; the library does not supply their values |
| Parameter name in a declaration | `p`, `bytes`, `header`, `output`, `length` | Placeholder names for **arguments supplied by the caller**; not automatically populated variables |

`HRP2::Validation` is **not a ninth function**, and `HRP2::Frame` is **not a function**. Both are types declared alongside the eight functions.

#### Exact relationship between a frame and the wire-format offsets

For a complete arm frame:

```text
frame.bytes[0..3]     = magic: HRP2
frame.bytes[4]        = dataset ID
frame.bytes[5..6]     = payload length (2-byte little-endian integer)
frame.bytes[7..10]    = session token        (ARM/FINGER/SENSOR only)
frame.bytes[11..14]   = sequence number       (ARM/FINGER/SENSOR only)
frame.bytes[15..18]   = producer timestamp    (ARM/FINGER/SENSOR only)
frame.bytes[19..]     = application bytes     (ARM/FINGER/SENSOR only)
last 4 valid bytes    = CRC32
frame.length          = total number of valid bytes, **outside** the transmitted frame
```

The full packet starts at `frame.bytes`. The payload starts at `frame.bytes + HRP2::HEADER_SIZE` (currently `+ 7`). The source of a value is determined by the *position in the packet*, not by a magic variable name.

**Do not use the control offsets for error messages.** Error frames have no session/sequence/timestamp prefix: their one-byte error code is at `frame.bytes[7]`.

#### Where do the bytes actually come from?

**On the transmitting side:** The application allocates and fills a payload buffer, then calls `HRP2::encode()`. The encoder writes the *complete* frame into `output.bytes` and sets `output.length`. Only then does transport code send `output.bytes` for `output.length` bytes.

```cpp
uint8_t payload[HRP2::ARM_PAYLOAD] = {};
// The sending application must populate metadata and application bytes here.

HRP2::Frame outgoing;
bool ok = HRP2::encode(outgoing, HRP2::ARM, payload, sizeof(payload));
if (ok) {
    // The program's transport code sends outgoing.bytes for outgoing.length bytes.
}
```

**On the receiving side:** Your UART parser or UDP receiver reads bytes from the physical interface into a buffer and records the actual received byte count. You may use an `HRP2::Frame`, but the transport function must fill it; declaring the object alone does not read any data.

```cpp
HRP2::Frame incoming;
// Your UART parser / UDP receiver fills incoming.bytes
// and sets incoming.length to the actual COMPLETE frame size.

if (HRP2::validate(incoming.bytes, incoming.length)
      == HRP2::Validation::Valid) {
    // Only now access fields that are guaranteed to exist.
}
```

You **do not have to use `HRP2::Frame`** for every helper. A general byte array works too:

```cpp
uint8_t twoBytes[] = {0x58, 0x00};
uint16_t result = HRP2::read16(twoBytes);  // Returns 88
```

The `Frame` type is specifically required by the first argument of the library's `encode()` function; `read16()`, `read32()`, `crc32()`, `validateHeader()` and `validate()` accept pointers to bytes, regardless of whether those bytes reside in a `Frame` or another array.

#### Preconditions and common mistakes

- **Do not confuse buffer capacity with valid length.** `sizeof(frame.bytes)` is 267; `frame.length` is the number of valid packet bytes. The `length` argument to `validate()` is **total frame length**, whereas the `length` argument to `encode()` is **payload length**.
- **Do not assume a `Frame` already contains data.** Creating `HRP2::Frame frame;` allocates storage but does not receive anything. Your transport code or `encode()` must populate it.
- **Do not assume `encode()` generates application metadata.** Session token, sequence counter, producer timestamp and opaque application bytes must be filled into the payload by the sender before encoding; `encode()` builds the common header and CRC.
- **Do not assume `validate()` validates motion freshness or safety.** It validates the wire frame only. An ARM packet with all-zero metadata can pass structural/CRC validation if correctly encoded.
- **Do not read beyond an available buffer.** The raw integer helpers read exactly 2 or 4 bytes; `validateHeader()` reads at least 7; none of these pointers carry a buffer length or perform bounds checks. A UART receiver must collect enough bytes first.
- **Do not confuse protocol identifiers with implemented IDs.** The space permits `0x00..0x0F`, but the current library accepts only `0x00..0x03`; other in-range IDs return `ReservedId`.
- **Do not treat CRC as message delivery or authentication.** A valid frame can be stale, duplicated, inappropriate for execution, or sent by an untrusted peer. These are outside the shared library.

### 4.1 All eight functions at a glance

| # | Call exactly as used in code | Arguments, in order | Return / result | Main purpose |
|---:|---|---|---|---|
| 1 | `uint16_t value = HRP2::read16(p);` | `p` | `uint16_t` | Decode 2 little-endian bytes |
| 2 | `uint32_t value = HRP2::read32(p);` | `p` | `uint32_t` | Decode 4 little-endian bytes |
| 3 | `HRP2::write16(p, value);` | `p`, `value` | `void`; writes at `p` | Encode 16-bit integer into 2 bytes |
| 4 | `HRP2::write32(p, value);` | `p`, `value` | `void`; writes at `p` | Encode 32-bit integer into 4 bytes |
| 5 | `uint32_t crc = HRP2::crc32(bytes, length);` | `bytes`, `length` | `uint32_t` | Compute CRC-32/ISO-HDLC |
| 6 | `HRP2::Validation result = HRP2::validateHeader(header);` | `header` | `HRP2::Validation` | Check the seven-byte common header |
| 7 | `HRP2::Validation result = HRP2::validate(bytes, length);` | `bytes`, `length` | `HRP2::Validation` | Validate the entire received frame |
| 8 | `bool success = HRP2::encode(output, id, payload, length);` | `output`, `id`, `payload`, `length` | `bool`; fills `output` | Construct complete HRP2 frame |

**Return versus output:** `read16`, `read32`, and `crc32` return numbers; the validators return a status; `encode` returns success/failure **and** writes an output `Frame`; `write16`/`write32` return nothing because they write directly into the caller's byte array.

### 4.2 `read16()` — read a 16-bit unsigned integer

**Actual function call:**

```cpp
uint16_t value = HRP2::read16(p);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `p` | `const uint8_t*` | Pointer to **at least 2 readable bytes**, typically inside a received frame |
| Returned | `value` | `uint16_t` | The numerical value of those two bytes, little-endian (`0..65535`) |

**Purpose:** Reassemble two wire-format bytes into one integer, without changing the input.

**Typical HARRP use:**

```cpp
// First ensure the first seven frame bytes are actually available.
uint16_t payloadLength = HRP2::read16(frame.bytes + 5);
```

`frame.bytes` is a buffer filled by UART/UDP reception or by `encode`. Adding `5` points to absolute byte offsets `5–6`, where the HRP2 payload length is stored. For `{0x58, 0x00}`, the function returns **88**. It does not verify buffer bounds.

### 4.3 `read32()` — read a 32-bit unsigned integer

**Actual function call:**

```cpp
uint32_t value = HRP2::read32(p);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `p` | `const uint8_t*` | Pointer to **at least 4 readable bytes**, usually in a complete validated frame |
| Returned | `value` | `uint32_t` | The unsigned 32-bit integer encoded by the four little-endian bytes |

**Purpose:** Recover session numbers, sequence numbers, timestamps, transmitted CRC values, or other 32-bit fields. This helper does not interpret them.

**Typical HARRP use, *after* validating an ARM/FINGER/SENSOR frame:**

```cpp
uint32_t sessionToken   = HRP2::read32(frame.bytes + 7);
uint32_t sequenceNumber = HRP2::read32(frame.bytes + 11);
uint32_t timestampMs    = HRP2::read32(frame.bytes + 15);
```

These are **absolute frame offsets**: the 7-byte common header comes first. The same reads relative to the start of the payload are `read32(payload + 0)`, `read32(payload + 4)`, and `read32(payload + 8)`. The CRC, if needed, can be obtained as `HRP2::read32(frame.bytes + frame.length - HRP2::CRC_SIZE)`.

### 4.4 `write16()` — write a 16-bit unsigned integer

**Actual function call:**

```cpp
HRP2::write16(p, value);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `p` | `uint8_t*` | Pointer to **at least 2 writable bytes** in an array you own |
| 2 | `value` | `uint16_t` | Integer supplied by the programmer |
| Returned | — | `void` | Nothing is returned; two bytes at `p` are modified |

**Purpose:** Serialize a 16-bit integer into little-endian bytes.

```cpp
uint8_t header[7] = {};
uint16_t payloadLength = 88;
HRP2::write16(header + 5, payloadLength);
// header[5] == 0x58; header[6] == 0x00
```

Normally, use `HRP2::encode()` to construct the packet header; it already calls `write16()` to store the length. `write16()` is useful when constructing another 16-bit binary field yourself.

### 4.5 `write32()` — write a 32-bit unsigned integer

**Actual function call:**

```cpp
HRP2::write32(p, value);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `p` | `uint8_t*` | Pointer to **at least 4 writable bytes**, often the beginning of a payload or metadata field |
| 2 | `value` | `uint32_t` | Number obtained or maintained by the sending application |
| Returned | — | `void` | Nothing; writes four little-endian bytes at `p` |

**Purpose:** Serialize a 32-bit integer into the wire format. In HRP2, the **producer**, not the library, provides its own session token, stream sequence and timestamp.

```cpp
uint8_t payload[HRP2::ARM_PAYLOAD] = {};
uint32_t sessionToken = 0x12345678; // Example only: producer initializes this.
uint32_t sequenceNumber = 42;      // Producer maintains/increments this.
uint32_t timestampMs = millis();   // Arduino monotonic timestamp.

HRP2::write32(payload + 0, sessionToken);
HRP2::write32(payload + 4, sequenceNumber);
HRP2::write32(payload + 8, timestampMs);
// Bytes from payload + 12 onward are opaque arm application data.
```

`millis()` is supplied by the Arduino environment, not the HRP2 library. A Linux program would supply its own monotonic clock. The fixed session value above is only a teaching example.

### 4.6 `crc32()` — calculate a checksum

**Actual function call:**

```cpp
uint32_t checksum = HRP2::crc32(bytes, length);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `bytes` | `const uint8_t*` | Pointer to the beginning of the byte sequence being checked |
| 2 | `length` | `size_t` | Number of readable bytes to include in the calculation |
| Returned | `checksum` | `uint32_t` | CRC-32/ISO-HDLC numeric value |

**Purpose:** Detect accidental changes to a byte sequence by computing its 32-bit CRC. It does not append, compare or authenticate anything.

```cpp
// Only after frame.length has been established and is >= HRP2::CRC_SIZE.
uint32_t calculatedCRC = HRP2::crc32(
    frame.bytes,
    frame.length - HRP2::CRC_SIZE
);
```

In an HRP2 packet, the checksum covers the common header and payload but **not the four CRC bytes already attached to the end**. `encode()` calculates and appends the CRC automatically; `validate()` recalculates and checks it automatically. You typically call `crc32()` directly for testing or special diagnostics.

Standard test vector:

```cpp
const uint8_t sample[] = {'1','2','3','4','5','6','7','8','9'};
uint32_t result = HRP2::crc32(sample, sizeof(sample));
// result == 0xCBF43926
```

### 4.7 `validateHeader()` — validate a seven-byte header

**Actual function call:**

```cpp
HRP2::Validation result = HRP2::validateHeader(header);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `header` | `const uint8_t*` | Pointer to **at least 7 received readable bytes** starting at the magic field |
| Returned | `result` | `HRP2::Validation` | Detailed outcome of header checks |

**Purpose:** Reject malformed or unsupported frame headers **before** waiting for the rest of the UART frame.

It checks the `HRP2` magic, dataset ID range, whether the ID is implemented, maximum payload length, and dataset-specific length requirements. It does not calculate CRC32, check application data, or wait for missing bytes.

```cpp
if (receivedByteCount >= HRP2::HEADER_SIZE) {
    HRP2::Validation result = HRP2::validateHeader(rxBuffer);
    if (result == HRP2::Validation::Valid) {
        size_t expectedBytes = HRP2::HEADER_SIZE
            + HRP2::read16(rxBuffer + 5) + HRP2::CRC_SIZE;
        // Receive until expectedBytes is available before calling validate().
    }
}
```

`rxBuffer` is populated by the calling UART receive code. `validateHeader()` never reads a UART port itself.

### 4.8 `validate()` — validate a complete packet

**Actual function call:**

```cpp
HRP2::Validation result = HRP2::validate(bytes, length);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `bytes` | `const uint8_t*` | Pointer to the beginning of one **complete** candidate frame |
| 2 | `length` | `size_t` | **Actual received frame byte count**, including header, payload and CRC |
| Returned | `result` | `HRP2::Validation` | `Valid` or a specified rejection reason |

**Purpose:** Verify the packet's complete framing and CRC before any controller forwards or consumes it.

The current implementation checks total frame size (`11..267`), header contents, exact declared versus received length, CRC32 equality and error-code range (`0..99`) for the error dataset.

```cpp
HRP2::Validation result = HRP2::validate(frame.bytes, frame.length);
if (result == HRP2::Validation::Valid) {
    uint8_t datasetID = frame.bytes[4];
    // Forward intact, or pass to the final application controller.
} else {
    // Drop the malformed/CRC-invalid/unsupported frame.
}
```

`frame.bytes` and `frame.length` are supplied by your UDP receiver or by a UART frame reassembly routine. **Never pass the buffer's capacity in place of the actual received frame length.** The library does not check session changes, sequence age, application freshness or motion safety.

### 4.9 `encode()` — construct a complete packet for transmission

**Actual function call:**

```cpp
bool success = HRP2::encode(output, id, payload, length);
```

| Position | Name | Required type | Where it comes from / role |
|---|---|---|---|
| 1 | `output` | `HRP2::Frame&` | Frame object created by the caller; gets filled by `encode()` |
| 2 | `id` | `uint8_t` | Dataset identifier: `HRP2::ARM`, `FINGER`, `SENSOR` or `ERROR` |
| 3 | `payload` | `const uint8_t*` | Byte array containing the **already-prepared payload** |
| 4 | `length` | `size_t` | **Payload size in bytes**, not total frame size |
| Returned | `success` | `bool` | `true` on successful encoding; otherwise `false` |

**Purpose:** Build the full wire-format frame. `encode()` inserts the `HRP2` magic, dataset ID, length, application-prepared payload and computed CRC32. It then validates the result.

```cpp
HRP2::Frame frame;
uint8_t payload[HRP2::ARM_PAYLOAD] = {};

// Before encoding, the sender should populate payload metadata and application data.
bool success = HRP2::encode(
    frame,          // output Frame, filled by the function
    HRP2::ARM,      // dataset ID 0x00
    payload,        // address of source payload bytes
    sizeof(payload) // payload byte count = 88
);

if (success) {
    // frame.bytes contains the complete HRP2 frame.
    // frame.length is 99 bytes for ARM; transmit exactly that many bytes.
}
```

On failure, `encode()` returns `false` and sets `output.length` to zero. It does not transmit anything itself. The `payload` buffer must be readable for `length` bytes and must **not overlap** `output.bytes` (the implementation uses `memcpy`). Control/sensor datasets require the application to populate the 12-byte metadata prefix first.

### 4.10 `Validation` result codes (a type, not a ninth function)

| Returned result | Meaning |
|---|---|
| `HRP2::Validation::Valid` | Frame/header passes all checks applicable to the call |
| `HRP2::Validation::BadMagic` | First four bytes do not match `HRP2` |
| `HRP2::Validation::InvalidId` | Dataset ID greater than `0x0F` |
| `HRP2::Validation::ReservedId` | ID is `0x04–0x0F` and not currently implemented |
| `HRP2::Validation::BadLength` | Declared or actual length conflicts with limits / dataset requirements |
| `HRP2::Validation::BadCrc` | CRC32 computed from bytes differs from appended value; returned by `validate()` |
| `HRP2::Validation::InvalidError` | Error code exceeds 99; returned by `validate()` |

`Valid` **does not** mean the application data is recent, has a permitted sequence or is safe to execute. Such checks belong to the final data consumer.

### 4.11 How to select the right function

| Developer's task | Function |
|---|---|
| Obtain payload size from the two-byte header field | `read16()` |
| Obtain session, sequence or timestamp from an accepted control frame | `read32()` |
| Write an additional 16-bit binary field | `write16()` |
| Pack a session token, sequence or timestamp into a payload | `write32()` |
| Compute a CRC directly for a test or diagnostic | `crc32()` |
| Validate a UART header after receiving at least 7 bytes | `validateHeader()` |
| Validate an entire UDP datagram / reconstructed UART frame | `validate()` |
| Create a new packet ready for UART/UDP transmission | `encode()` |

**Typical flow:** Producer fills payload → `encode()` → transport sends the completed bytes. Receiver accumulates a complete frame → `validate()` → bridge forwards unchanged, or final controller independently checks session/sequence/freshness and then processes application data.

## 5. Copy-ready examples

The following examples use only the current library API. They show how to generate or validate packets; they do **not** implement motor execution or session restart policy.

### Example A — Generate a diagnostic error frame

```cpp
#include "HARRPProtocol.h"

HRP2::Frame frame;
uint8_t errorCode = 42;  // Encoding example; meaning is not assigned yet.

bool ok = HRP2::encode(frame, HRP2::ERROR, &errorCode, 1);
if (ok) {
    // Send exactly frame.length bytes from frame.bytes.
    // For this dataset, frame.length == 12.
}
```

The encoded bytes are:

```text
48 52 50 32 03 01 00 2A B1 11 B3 34
|---HRP2---| ID |LEN| ERR |-- CRC32 --|
```

CRC input: `48 52 50 32 03 01 00 2A`  
CRC numeric value: `0x34B311B1`  
CRC on wire: `B1 11 B3 34`

### Example B — Wrap 76 opaque arm-control bytes

```cpp
#include "HARRPProtocol.h"
#include <string.h>

bool makeArmPacket(
    HRP2::Frame& out,
    uint32_t sessionToken,
    uint32_t sequence,
    uint32_t producedAtMs,
    const uint8_t applicationBytes[76])
{
    uint8_t payload[HRP2::ARM_PAYLOAD] = {};

    HRP2::write32(payload + 0, sessionToken);
    HRP2::write32(payload + 4, sequence);
    HRP2::write32(payload + 8, producedAtMs);
    memcpy(payload + HRP2::METADATA_SIZE, applicationBytes, 76);

    return HRP2::encode(out, HRP2::ARM, payload, sizeof(payload));
}
```

This constructs a **99-byte** arm frame. The 76 application bytes remain opaque. The data producer must choose and maintain a session token and sequence counter; the library does not do so.

For fingers, the same pattern applies with `HRP2::FINGER`, `HRP2::FINGER_PAYLOAD`, and **24 application bytes**.

### Example C — Validate a complete incoming frame

```cpp
#include "HARRPProtocol.h"

void onCompleteFrame(const uint8_t* bytes, size_t length) {
    if (HRP2::validate(bytes, length) != HRP2::Validation::Valid)
        return; // Reject corrupted, malformed or unsupported frames.

    const uint8_t id = bytes[4];
    const uint16_t payloadLength = HRP2::read16(bytes + 5);
    const uint8_t* payload = bytes + HRP2::HEADER_SIZE;

    if (id == HRP2::ERROR) {
        uint8_t errorCode = payload[0]; // 0..99; definition is external.
        (void)errorCode;
    } else {
        // Forward unchanged, or hand the opaque payload to its actual consumer.
        // The library does not execute commands.
        (void)payloadLength;
    }
}
```

This function assumes its caller has already assembled **one complete frame**. A UDP datagram can contain a complete frame; a UART byte stream needs its own accumulation, length checking, timeout and resynchronization logic.

### Example D — Read a stream's metadata after validation

```cpp
// Use only after successful full validation and for IDs 0x00..0x02.
const uint8_t* payload = frame.bytes + HRP2::HEADER_SIZE;
uint32_t sessionToken = HRP2::read32(payload + 0);
uint32_t sequence     = HRP2::read32(payload + 4);
uint32_t timestampMs  = HRP2::read32(payload + 8);
const uint8_t* applicationBytes = payload + HRP2::METADATA_SIZE;
```

**Do not** compare sequence numbers from different streams or devices. The final consuming controller decides whether a packet belongs to an accepted session and whether it is current enough to use. The bridge forwards valid packets without these decisions.

## 6. Correct receive/forward pipeline

```text
Transport receives bytes
       |
       v
Determine complete frame boundary
   - UDP: datagram must contain exactly one complete HRP2 frame
   - UART: find magic, wait for header, check declared length,
           accumulate frame, recover after malformed data
       |
       v
HRP2::validate(frame_bytes, received_length)
       |
   +---+--------------------+
   |                        |
 Invalid                   Valid
   |                        |
 Discard/log                 v
                      Route by dataset ID
                             |
             +---------------+---------------+
             |                               |
      Intermediate bridge              Final consumer
      forwards exact bytes              interprets its
      without modifications             application data,
                                       validates freshness
                                       and local safety
```

The shared header handles **frame format and integrity**, not UART driver state, READY signaling, Wi-Fi reconnection, retries, packet prioritization, or endpoint authorization.

For stream parsers, never trust a length field until the fixed header has been received and its limits checked. In particular, `validateHeader()` must not be called on a partial header.

## 7. CRC32: what is protected and what is not

For an encoded packet:

```text
[ HRP2 | ID | payload length | payload ] [ CRC32 ]
\------------ CRC32 input -------------/
```

The CRC covers the identity marker, dataset ID, declared length, stream metadata, and application bytes. Any of those accidentally changing in transit will generally invalidate the CRC. For an intact frame, `validate()` recomputes the CRC and compares it with the last four bytes.

**CRC32 is not an authentication mechanism and cannot repair damaged bytes.** A frame with a valid CRC can still contain unsafe requested motion, stale targets, incorrectly calibrated sensor values, or a sender's application-level mistake. These checks remain outside this library.

## 8. Basic host-side smoke test

Because the library has no Arduino dependencies, its core helpers can be tested with a normal C++ compiler.

Create `hrp2_smoke_test.cpp` adjacent to `HARRPProtocol.h`:

```cpp
#include "HARRPProtocol.h"
#include <assert.h>
#include <string.h>

int main() {
    const char* test = "123456789";
    assert(HRP2::crc32(reinterpret_cast<const uint8_t*>(test), 9)
           == 0xCBF43926u);

    HRP2::Frame frame;
    uint8_t code = 42;
    assert(HRP2::encode(frame, HRP2::ERROR, &code, 1));
    assert(frame.length == 12);

    const uint8_t expected[12] = {
        0x48, 0x52, 0x50, 0x32, 0x03, 0x01, 0x00, 0x2A,
        0xB1, 0x11, 0xB3, 0x34
    };
    assert(memcmp(frame.bytes, expected, sizeof(expected)) == 0);
    assert(HRP2::validate(frame.bytes, frame.length)
           == HRP2::Validation::Valid);

    frame.bytes[7] ^= 1; // Alter payload without recomputing CRC.
    assert(HRP2::validate(frame.bytes, frame.length)
           == HRP2::Validation::BadCrc);

    uint8_t arm[HRP2::ARM_PAYLOAD] = {};
    assert(HRP2::encode(frame, HRP2::ARM, arm, sizeof(arm)));
    assert(frame.length == 99);

    uint8_t finger[HRP2::FINGER_PAYLOAD] = {};
    assert(HRP2::encode(frame, HRP2::FINGER, finger, sizeof(finger)));
    assert(frame.length == 47);

    assert(!HRP2::encode(frame, 0x04, &code, 1)); // Reserved ID.
}
```

Example Linux/MinGW command:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -I. hrp2_smoke_test.cpp -o hrp2_smoke_test
./hrp2_smoke_test
```

A successful run has no assertion failures. This verifies basic packet encoding and validation, **not** UART timing, Wi-Fi interoperability, motor safety, or hardware readiness.

### Further test cases for contributors

| Test | Expected outcome |
|---|---|
| Wrong magic | `BadMagic` |
| Valid magic, ID `0x04` | `ReservedId` |
| Valid magic, ID `0x10` | `InvalidId` |
| Arm payload length other than 88 | `BadLength` |
| Finger payload length other than 36 | `BadLength` |
| Sensor payload length below 12 or above 256 | `BadLength` |
| Error payload length other than 1 | `BadLength` |
| Error code 100 with freshly computed CRC | `InvalidError` |
| Single flipped byte without updated CRC | `BadCrc` (assuming valid framing) |
| Extra/truncated bytes | `BadLength` |
| `encode()` called with unsupported ID | `false`; `output.length == 0` |

## 9. Limitations, safety and integration notes

1. **No streaming decoder in the library.** UART frame assembly and parser resynchronization are implemented separately in the hub; future controllers need equivalent transport integration.
2. **No application parsing.** There are no float, quaternion, sensor or servo helpers. Opaque payloads are intentional.
3. **No stream acceptance policy.** Session changes, sequence rollover and timestamp freshness are endpoint responsibilities. The precise restart-acceptance policy is still open.
4. **No guaranteed delivery.** The error dataset has no sequence or acknowledgment. CRC detects corruption of received frames but cannot identify every lost or duplicated event.
5. **No command authorization or safety decision.** Actual motor controllers enforce local limits, safety conditions, and eventual emergency-stop rules.
6. **No automatic support for reserved datasets.** New IDs require changes to `validateHeader()` and appropriate integration tests.
7. **Fixed sizes are part of compatibility.** Changing arm/finger payload length or metadata layout changes what existing firmware accepts.
8. **Legacy incompatibility.** Older draft1b also used magic `HRP2` but a different header. It is **not wire-compatible** with this simplified protocol. Update connected endpoints together; the magic string alone cannot detect a mixed-firmware deployment.
9. **No input pointer checks.** `validateHeader()`, integer accessors, `crc32()`, and `validate()` presume readable input buffers of the stated size; the caller must enforce bounds and valid pointers.
10. **Memory is fixed per frame.** Each `Frame` reserves 267 bytes. Multiple queued frames consume their own copies; size RAM buffers deliberately on microcontrollers.

### Change-control guidance

When updating `HARRPProtocol.h`, a contributor should update the matching protocol reference and this guide in the same commit/review cycle. Record any changed ID, maximum size, length rule, CRC parameter, metadata offset, or serialization rule. Include test vectors and confirm that every deployed controller is compatible before enabling robot operation.

**Project ownership boundary:** The communication workstream owns this common framing library. Arm, finger, sensor and safety developers own the meaning and handling of their application payloads. The shared header must remain independent of those domains.

## 10. Revision history

| Revision | Date | Description |
|---|---|---|
| 0.4 | 2026-11-15 | Added required prerequisites to the API chapter: full `Frame` definition, library-defined members versus programmer variables, packet-buffer ownership, transmitted versus in-memory fields, API symbol glossary, exact byte offsets, producer/receiver data provenance, and caller preconditions. No library implementation changes. |
| 0.3 | 2026-11-15 | Reorganized the complete eight-function API as directly usable C++ calls with argument order, variable origin, argument types, return values, examples, bounds requirements and workflow quick-reference. No library implementation changes. |
| 0.2 | 2026-11-15 | Expanded all eight public functions into a detailed API reference, including argument origins, return values, buffer preconditions, byte offsets, and producer/receiver usage. |
| 0.1 | 2026-11-15 | First standalone usage guide for `HARRPProtocol.h` as introduced by HRP2 Nano hub commit `918fcd0`. Includes API reference, binary examples, test harness, integration responsibilities and known limitations. |

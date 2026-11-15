# HARRP V2 — Communication Architecture and HRP2 Protocol

**Engineering handoff draft — Revision 0.1**  
**Date:** 13 November 2026  
**Project:** HARRP V2 robotic arm and anthropomorphic hand  
**Repository:** [`captainShnitzel/HARRP`](https://github.com/captainShnitzel/HARRP)  
**Firmware baseline:** `feature/esp32-comm-hub-v2`, commit `918fcd0`  
**Suggested repository location:** `docs/HARRP_V2_Communication_Protocol.md`

> **Document status:** Initial technical reference for students and project collaborators. It describes the agreed communication architecture and the first compiling Nano ESP32 implementation. It is **not** evidence of completed hardware integration, a frozen sensor specification, or a certified safety system. Items marked **TBD** must be resolved by the responsible developer before integration.
>
> **Source note:** The firmware snapshot used to prepare this draft predates the project's reported final manual edits. The developer reports that commit `918fcd0` includes a compiling build with Nano READY assigned to D2, the three-second UDP inactivity reset removed, and Wi-Fi/READY decoupled. Those final source edits and physical behavior have not been independently re-inspected here. Confirm against the actual committed tree before accepting this document as an implementation audit.

---

## 1. At a glance

HARRP is a fixed, multi-controller robot. Its communication software carries **complete binary datasets** between devices; it does not interpret application values or execute robotic actions.

| Design rule | Current decision |
|---|---|
| Protocol signature | ASCII **`HRP2`** (`48 52 50 32`) |
| Transport style | Push-based; no ordinary GET/REPLY exchange |
| Common frame | 4-byte magic + 1-byte ID + 2-byte payload length + payload + 4-byte CRC32 |
| Supported dataset IDs | `0x00`–`0x03`; `0x04`–`0x0F` reserved |
| Maximum payload / frame | 256 bytes / 267 bytes in the current implementation |
| Control/feedback metadata | 4-byte session token + 4-byte sequence + 4-byte producer timestamp |
| Error metadata | None; one binary error-code byte, decimal range 00–99 |
| Internal UART | Full duplex, initially 2,000,000 baud, with independent READY GPIOs |
| Wireless | Nano ESP32 Wi-Fi access point with UDP, port 5005 |
| Communication hub | Validates frame, routes by fixed dataset ID, displays/logs diagnostics; **does not execute commands** |
| Safety | Local control subsystem decides motion validity and performs required protective action |

**New-contributor rule:** Do not add device addresses, dynamic routing, connection negotiation, universal sequence checks, or an application-level request/response layer without an explicit architecture change.

## 2. Controllers and operating modes

### 2.1 Direct/manual operating mode

```text
USER / WEARABLE SIDE                         ROBOT SIDE

Nano ESP32                                  Nano ESP32 (communication hub)
operator targets ─────── Wi-Fi / UDP ──────> validated frame forwarding
received feedback <──── Wi-Fi / UDP ─────── diagnostics / display
                                                  ↕
                                     UART 2 Mbps + two READY GPIOs
                                                  ↕
                                            Teensy 4.1
                                         arm control (7 DOF)
                                         routing to/from palm
                                                  ↕
                                           Dedicated UART
                                                  ↕
                                           Palm ESP32-S3
                                        finger control / sensors
```

The user-side controller produces arm/finger datasets. The Teensy executes arm control on its own real-time schedule, and the palm controller executes finger control. Telemetry and errors travel in the opposite direction. The robot-side Nano merely bridges the wireless and wired interfaces, with local diagnostics.

### 2.2 Future AI-assisted operating mode

```text
User-side Nano ESP32
          ↕ Wi-Fi / UDP
NVIDIA Jetson AGX Thor
   ├── Communications service: frame forwarding / transport
   └── Separate AI/planning application: perception, sensor use,
       assisted target generation and trajectory planning
          ↕ Ethernet / UDP
Teensy 4.1 — final embedded arm controller and safety checks
          ↕ Dedicated UART
Palm ESP32-S3 — finger controller and sensor producer
```

The communications service on Thor is **not** the AI planner. The AI/planner may consume sensor and operator data and **produce new** arm targets; the communications service simply transports complete frames. The robot is intended to remain usable in direct mode with Thor absent. The upstream control-source selector is handled by the Teensy at boot, not by the Nano protocol.

### 2.3 Responsibilities

| Component | Must do | Must not do as a communications component |
|---|---|---|
| User-side Nano | Produce control frames, receive feedback, maintain producer stream metadata | Rely on hub to interpret movement |
| Robot-side Nano hub | Validate frame/CRC, apply fixed routing, buffer/forward, maintain displays/logs, recover interfaces | Perform IK, motion limits, sensor fusion, motor commands, or fault-stop decisions |
| Teensy 4.1 | Consume arm commands, enforce execution/safety rules, route finger/sensor/error traffic | Treat receipt of a CRC-valid frame alone as proof of a safe command |
| Palm ESP32-S3 | Consume finger commands, generate feedback and fault reports | Depend on hub for local actuator protection |
| Thor communications service | Carry HRP2 frames between interfaces | Implicitly alter targets or manage motors |
| Thor planning application | Generate assisted commands from targets, robot state and perception | Bypass Teensy's required local motion validation |

---

## 3. Universal HRP2 frame format

All multibyte integers use **little-endian** encoding. The frame is identical whether transported as the contents of one UDP datagram or carried in the UART byte stream.

```text
+------------+---------+----------------+------------------+---------+
| MAGIC      | DATA ID | PAYLOAD LENGTH | PAYLOAD          | CRC32   |
| 4 bytes    | 1 byte  | 2 bytes        | N bytes          | 4 bytes |
+------------+---------+----------------+------------------+---------+
    offset 0       4         5–6             7...            7+N...
```

| Offset | Size | Field | Rule |
|---|---:|---|---|
| `0–3` | 4 B | Magic | Literal bytes `48 52 50 32` (`HRP2`) |
| `4` | 1 B | Dataset ID | Assigned category; allowed on wire today: `00`–`03` |
| `5–6` | 2 B | Payload length (`N`) | Unsigned little-endian; includes dataset-specific metadata; excludes header and CRC |
| `7 ... 6+N` | `N` B | Payload | Treated as opaque by a forwarding gateway |
| `7+N ... 10+N` | 4 B | CRC32 | Unsigned little-endian CRC of every preceding frame byte |

**Total length:** `11 + N` bytes. **Current limit:** `0 ≤ N ≤ 256`, hence at most **267 bytes** per frame. Assigned datasets impose stricter requirements (Section 5). The currently implemented frame parser does not accept an arbitrary zero-length frame.

There are deliberately **no** fields for source device, destination device, general packet type, protocol version, or GET/REPLY operation. The protocol signature stays `HRP2` for this first implemented V2 format. An incompatible future format change must be coordinated across all endpoints; reusing `HRP2` does not make the older, incompatible draft1b packets valid.

### 3.1 Validation before forwarding

A receiver or bridge must:

1. Find the four-byte `HRP2` signature. On UART, recover from invalid prefixes without treating them as a packet.
2. Read the dataset ID and little-endian payload length.
3. Check the assigned ID and applicable payload limits; reject invalid/reserved IDs.
4. Wait for the **complete** frame, within its bounded receive storage.
5. Recalculate CRC32 over the header and payload and compare with the four CRC bytes.
6. Apply any dataset **structural** constraints, e.g., error code must be `0–99`.
7. Only then publish the complete frame to the predetermined forwarding path or endpoint.

The communication gateway stops here. Session acceptance, target validity, motion freshness, actuator limits and safety reactions belong to the **controller that consumes** the stream.

---

## 4. Dataset ID registry and fixed routing

IDs occupy one byte but only **16 identifiers** are allocated by the project, from `0x00` through `0x0F`.

| Hex ID | Decimal | Dataset | Normal producer(s) | Intended recipient / use | Current status |
|---|---:|---|---|---|---|
| `0x00` | 0 | Arm targets | Wearable in direct mode; assisted controller in Thor mode | Teensy 4.1 | Active; payload size allocated |
| `0x01` | 1 | Finger targets | Wearable / assisted controller | Palm ESP32-S3 via Teensy | Active; payload size allocated |
| `0x02` | 2 | Sensor feedback | Palm/sensing subsystem; authoritative producer TBD | Upstream wearable or Thor | Active ID; contents TBD |
| `0x03` | 3 | Error reports | Teensy, palm, and other fault producers | Upstream diagnostics and hub display/log | Active ID; codes TBD |
| `0x04` | 4 | Reserved | — | — | Not accepted by current library |
| `0x05` | 5 | Reserved | — | — | Not accepted by current library |
| `0x06` | 6 | Reserved | — | — | Not accepted by current library |
| `0x07` | 7 | Reserved | — | — | Not accepted by current library |
| `0x08` | 8 | Reserved | — | — | Not accepted by current library |
| `0x09` | 9 | Reserved | — | — | Not accepted by current library |
| `0x0A` | 10 | Reserved | — | — | Not accepted by current library |
| `0x0B` | 11 | Reserved | — | — | Not accepted by current library |
| `0x0C` | 12 | Reserved | — | — | Not accepted by current library |
| `0x0D` | 13 | Reserved | — | — | Not accepted by current library |
| `0x0E` | 14 | Reserved | — | — | Not accepted by current library |
| `0x0F` | 15 | Reserved | — | — | Not accepted by current library |

**Rule:** `0x10–0xFF` are outside the currently allocated ID space. IDs `0x04–0x0F` are inside the reserved space but remain unsupported until the shared protocol definitions and routing rules are updated.

### 4.1 Direction map for the first Nano hub

| Ingress interface | Dataset | Nano behavior |
|---|---|---|
| Wi-Fi/UDP from wearable | `0x00` arm | Forward original frame to Teensy UART |
| Wi-Fi/UDP from wearable | `0x01` fingers | Forward original frame to Teensy UART |
| Wi-Fi/UDP from wearable | `0x03` error | Record/display locally; current hub code **does not** forward this direction to Teensy |
| Wi-Fi/UDP from wearable | `0x02` sensors | Reject as wrong direction in current hub |
| UART from Teensy | `0x02` sensors | Forward original frame to wearable over UDP |
| UART from Teensy | `0x03` error | Record/display; queue original frame for UDP forwarding |
| UART from Teensy | `0x00` or `0x01` | Reject as wrong direction in current hub |

These are the **current hub's** direction-specific rules, not a general-purpose routing table for arbitrary network configurations. Adding a new dataset or control topology requires an explicit update to the appropriate gateway's fixed routing logic.

---

## 5. Dataset payload classes

### 5.1 Sequenced control/feedback: `0x00`, `0x01`, `0x02`

The first **12 payload bytes** are stream metadata. These bytes are *inside* the payload, not part of the seven-byte universal header.

| Offset relative to payload | Size | Field | Meaning |
|---|---:|---|---|
| `0–3` | 4 B | Session token (`uint32`) | Identifies a producer operating session, normally regenerated after restart |
| `4–7` | 4 B | Sequence (`uint32`) | Producer's increasing update counter for that particular stream |
| `8–11` | 4 B | Timestamp (`uint32`) | Producer-local monotonic sample time in milliseconds |
| `12...` | Variable | Application bytes | **Opaque** to the communications hub; semantics specified by dataset owner |

**Stream ownership:** An independently originating dataset stream has its **own** session/sequence. Do not compare an arm sequence with a sensor sequence or compare independently generated source counters. Any shared ID with multiple sequenced producers needs an agreed stream owner, aggregation strategy, or separate dataset allocation. The sensor student will determine the feedback architecture.

**Counter rollover:** A `uint32` sequence wraps naturally from `0xFFFFFFFF` to `0`. At the consuming endpoint, a standard same-session modular comparison is:

```cpp
uint32_t delta = incomingSequence - lastAcceptedSequence;
bool newer = delta != 0 && delta < 0x80000000u;
```

The comparison assumes a plausible forward gap smaller than half the counter range. Sequence numbers do not create delivery guarantees, and the hub does **not** validate them when forwarding.

**Session/restart policy — TBD for endpoint integration:** A changed session token indicates a possible sender restart. A receiver must not blindly switch back to a retired session when delayed packets arrive. An RTP-inspired consecutive-new-packet confirmation is a candidate approach; exact startup, reconnection and stream-change acceptance logic belongs to the endpoint developer. **No session negotiation or HELLO/ACK is part of HRP2.**

**Time/freshness:** Producer timestamps are from the producer's own monotonic clock. Without synchronized clocks, do not subtract another MCU's absolute timestamp from the receiver's clock. An endpoint may use local arrival time and its own application-level timeout rules. The hub must not make motion freshness decisions.

### 5.2 Arm targets — `0x00`

**Current reservation:** 76 application bytes, plus 12 metadata bytes, gives a payload of 88 bytes (`58 00` in the frame length field), giving a **99-byte complete frame**.

The original design estimate is 19 × IEEE-754 32-bit values (3 Cartesian target coordinates, 4 target-orientation quaternion components, and 12 posture-quaternion components). **For communication implementation, all 76 application bytes are opaque.** Exact byte ordering, coordinate frame, units, physical limits and IK interpretation are to be specified by the arm-control owner, not by this transport document.

### 5.3 Finger targets — `0x01`

**Current reservation:** 24 application bytes, plus 12 metadata bytes, gives a payload of 36 bytes (`24 00` in the frame length field), giving a **47-byte complete frame**.

The size is based on six anticipated 32-bit actuator target values. Their detailed representation, ordering and physical meaning are intentionally not defined here. The Teensy forwards finger frames; the palm controller is responsible for application interpretation and execution.

### 5.4 Sensor feedback — `0x02`

**Schema TBD; student-owned.** The number and type of sensors, data encoding, producer organization, sample rates, and downstream consumer behavior have not been finalized.

The *present library* accepts a sensor payload from **12 through 256 bytes**. This reserves a 12-byte stream metadata prefix and provides **0–244 bytes** for application data; frame size is **23–267 bytes**. This is a software capacity, **not** an endorsement of an empty or particular sensor schema.

Sensor owners must publish a revision to the shared dataset definition before their payload is treated as stable. They must also resolve how independent sensor producers share (or do not share) sequence ownership.

### 5.5 Diagnostic errors — `0x03`

Errors use **only one application byte**, without session, sequence, or timestamp. Their complete frame is **12 bytes**.

- The byte is a **binary integer** in the inclusive decimal range `00–99` (`0x00–0x63`), **not** two ASCII digit characters and not BCD.
- Every assigned code will eventually be globally unique across HARRP and identify its subsystem by the catalog entry; there is **no separate source address**.
- Codes, ownership, severity, conditions, and clearing semantics are **TBD**. In particular, the meaning of `00` as a possible clear or no-error condition is not yet agreed.
- The Nano may display/log received error numbers without assigning them robotic consequences. Some future errors will require a stop, but **the detecting/controlling embedded subsystem must enforce its own required protective action**; it must not depend on the hub, UDP delivery, or the display.
- Error frames are events in the current implementation. They are not sequenced, deduplicated or acknowledged by HRP2. This is sufficient for an initial diagnostic channel, **not guaranteed delivery of every fault event**.

### 5.6 Complete size table

| Dataset | Stream metadata | Application data | Frame payload (`N`) | Complete frame |
|---|---:|---:|---:|---:|
| `0x00` Arm | 12 B | 76 B | 88 B | **99 B** |
| `0x01` Fingers | 12 B | 24 B | 36 B | **47 B** |
| `0x02` Sensors | 12 B (provisional convention) | 0–244 B, TBD | 12–256 B | **23–267 B** |
| `0x03` Errors | 0 B | 1 B | 1 B | **12 B** |

---

## 6. CRC32: integrity verification

HARRP uses **CRC-32/ISO-HDLC** for every HRP2 frame, including errors. The common header and every payload byte are protected.

| Parameter | Value |
|---|---|
| Generator polynomial (normal) | `0x04C11DB7` |
| Reflected polynomial used in software | `0xEDB88320` |
| Initial remainder | `0xFFFFFFFF` |
| Input / output reflection | Yes / Yes |
| Final XOR | `0xFFFFFFFF` |
| Serialized CRC field | 4 bytes, **little-endian** |
| Independent standard test | ASCII `123456789` → `0xCBF43926` |

### 6.1 Worked error-frame example

This example is only an **encoding demonstration**: decimal error **42** is not yet assigned to any real fault.

```text
48 52 50 32   03   01 00   2A   B1 11 B3 34
|------------| |--| |-----| |--| |-----------|
   'HRP2'      ID   N = 1   42   CRC32 LE
```

1. The CRC input is the first eight bytes: `48 52 50 32 03 01 00 2A`.
2. CRC-32/ISO-HDLC produces `0x34B311B1`.
3. The sender appends the result in little-endian byte order: `B1 11 B3 34`.
4. The receiver independently computes the CRC over the first eight bytes and compares it with the decoded four-byte value.
5. If they differ, the frame is invalid and is discarded; its payload must not be published.

CRC32 is **not** encryption, sender authentication, reliable delivery, or command authorization. A frame can have a correct CRC while containing an unsafe target or coming from an unauthorized device. The application control and security requirements are separate.

---

## 7. Physical interfaces and readiness flow control

### 7.1 Nano ↔ Teensy UART

The initial configuration uses 2,000,000 baud, full-duplex UART and a common electrical ground. TX/RX are crossed. The protocol is binary; do not treat UART bytes as line-oriented text. There are two **independent** active-HIGH receiver-ready signals, one per receiving endpoint:

| Signal | Driven by | Read by | Connection |
|---|---|---|---|
| `TEENSY_READY` | Teensy | Nano | **Teensy GPIO27 → Nano D3** (existing) |
| `NANO_READY` | Nano | Teensy | **Nano D2 → Teensy input GPIO TBD** (new assignment) |

Total logical interconnect: TX, RX, GND, and two READY signals (five conductors). Use compatible 3.3 V logic. Physical pin mapping, board wiring, power-on default LOW, and signal timing must be confirmed on actual hardware. D2 corresponds to the Nano board's default Arduino digital pin name; use the board mapping rather than assuming arbitrary ESP32 GPIO numbers.

**READY contract:**

- `HIGH`: This receiver has sufficient capacity to accept the next **complete maximum-sized HRP2 frame** under the agreed headroom/flow-control rules.
- `LOW`: Do not start another frame in that direction.
- A sender checks the peer's READY before starting a new frame, without blocking its other tasks.
- Once a frame has started, it is finished even if READY falls during transmission.
- A continuously HIGH READY may allow back-to-back frames **only if** the receiver's buffer margin and timing genuinely support them; READY is level-based, not a per-packet edge handshake.
- UART readiness represents UART receiver capacity **only**. Wi-Fi connection status must not directly drive the READY GPIO.

**Current Nano implementation parameters (subject to hardware measurement):** 2,048-byte UART RX driver buffer; 1,024-byte TX driver buffer; 256 RX/TX bytes serviced per loop pass; maximum frame 267 bytes; receive-ready reserve of `2 × 267 + 128 = 662` bytes; stalled partial-frame timeout of 20 ms. These are implementation values, not validated guaranteed response times.

### 7.2 Wi-Fi / UDP

- Robot-side Nano operates as a Wi-Fi access point (development SSID: `robot`).
- Application traffic uses **UDP port 5005**.
- The hub learns the wearable IP address and source port from a valid accepted packet, using that endpoint for reverse UDP traffic.
- One UDP datagram is expected to contain **one complete HRP2 frame**. No GET/REPLY exchange is used.
- Loss, reordering and duplication can occur on UDP; endpoints responsible for control streams determine their sequence/session/freshness policy.
- **Normal silence is not a socket error.** The manually revised firmware removes the former three-second inactivity-triggered socket rebind. Actual AP, station, or socket failure is still handled by connection-recovery logic.
- Wi-Fi disconnect must **not** make UART unavailable. Nano READY stays governed by UART receive capacity; local error display and logging can continue without a wearable connection.

**Security note:** The firmware has used an easily guessed development AP credential. CRC does not authenticate control packets, and the current peer-learning model is not an authorization scheme. Change credentials and define sender authorization before allowing operation in a shared/untrusted radio environment or relying on wireless commands for safety-relevant use.

---

## 8. Buffering, forwarding and diagnostics

### 8.1 Latest-value versus event handling

Continuous commands/feedback and discrete errors have different queue semantics:

| Traffic | Current storage behavior | Reason / limitation |
|---|---|---|
| Outbound arm target | One pending latest-value slot | Avoid building a queue of obsolete movement commands |
| Outbound finger target | One pending latest-value slot | Same |
| Upstream sensors | One latest-value slot | Most recent snapshot replaces an unsent older snapshot |
| Upstream error forwarding | FIFO, 8 frames in current hub | Events should not be collapsed into one snapshot; FIFO overflow is possible |
| Local USB error log | FIFO, 16 entries in current hub | Local diagnostic history is bounded and volatile |
| Local link-event log | FIFO, 8 entries in current hub | Records transport state changes without blocking UART |

A complete UART frame already in transmission must remain immutable. A newly received snapshot may replace an **unsent** snapshot, not an in-flight frame. The hub forwards original valid bytes without recalculating or changing session, sequence, or payload values.

**Delivery caveat:** No general acknowledgement or guaranteed event delivery is implemented. Wi-Fi disconnection can clear unsent network-oriented buffers; local USB logs are separate and volatile. If a future application requires reliable preservation of every error, that is a separately specified safety/telemetry requirement.

### 8.2 Display and USB diagnostics

The Nano retains its existing two-digit seven-segment display circuitry (SN74LS47N drivers) and USB serial diagnostics.

- Decimal error codes are shown as tens and ones digits when an error packet is received.
- In the current firmware model, the latest received error takes display priority over routine link-status indications and remains visible until reboot or a future explicitly defined display policy.
- Received code `00` is not yet established as a universal clear event; do **not** build an implicit global fault reset around it.
- USB Serial reports accepted error events and link events with local reception time, and can emit optional periodic counters.
- USB output is processed with a bounded, nonblocking write budget; an unavailable serial monitor must not stall packet forwarding.
- A red/green/blue status LED set also indicates transport status; exact meanings should be checked against the committed firmware during hardware bring-up.

A communications link-health indication is **not** the same thing as authorization to move. Error messages for logging and actual physical stop mechanisms must remain separate.

### 8.3 Startup and interruption behavior

The Nano starts with `NANO_READY = LOW` until its UART receiver is initialized and has headroom. It may then advertise READY regardless of whether its Wi-Fi access point is up.

When a wearable disconnects, the hub can invalidate its stored UDP peer and unsent control snapshots, preventing stale queued operator commands from being forwarded later. A frame already transmitting on UART is finished. The Teensy must independently apply its own loss-of-command policy; the Nano must not attempt to implement motor stop behavior.

---

## 9. Software organization and change management

### 9.1 Shared packet library

`firmware/robot-communication-nano-esp32/src/HARRPProtocol.h`

Header-only C++ definitions (no Arduino-dependent headers):

- Magic bytes and dataset constants.
- Maximum sizes and fixed arm/finger payload lengths.
- Little-endian integer helpers.
- CRC-32/ISO-HDLC calculation.
- Structural validation and frame encode helper.

This file is the initial **shared wire-format reference** for future Teensy, palm, and Thor code. Each platform may reuse it directly or implement equivalent logic, provided its produced bytes match the same test vectors.

### 9.2 Robot-side Nano hub

`firmware/robot-communication-nano-esp32/src/main_nano_esp32_comm_hub_v2.cpp`

Owns hardware integration: Wi-Fi AP / UDP, `Serial0` UART, GPIO READY, LED/display, frame buffering, UART parsing/resynchronization, diagnostic queues and nonblocking main loop. The firmware is not a general robotic application.

### 9.3 Implementation baseline

At the time of this draft, the working feature branch has been reported to compile and was pushed as:

```text
Branch:  feature/esp32-comm-hub-v2
Commit:  918fcd0
Title:   Implement simplified HRP2 Nano ESP32 communication hub
```

The previous communication implementation used a different, larger `HRP2` header with GET/REPLY and device addressing; it is **not wire-compatible** with this simplified HRP2 frame despite the same four ASCII magic bytes. The old V1 communication format is also incompatible. All participating senders and receivers must be updated together; do not mix old and new message formats.

**Source of truth:** This document describes intended behavior. The repository's *currently built and deployed* firmware is the source of truth for observed behavior. If implementation and specification differ, record the discrepancy rather than silently treating an untested behavior as confirmed.

---

## 10. Integration checklist for new contributors

The following is a practical handoff checklist; it is **not** a claim that these tests have already passed on physical hardware.

- [ ] Check out `feature/esp32-comm-hub-v2` or its later approved integration branch and review commit history.
- [ ] Confirm `HARRPProtocol.h` and the Nano firmware compile in the appropriate Arduino Nano ESP32 PlatformIO environment.
- [ ] Verify D2/D3 READY wiring, Teensy receiving GPIO, UART TX/RX crossing and 3.3 V electrical compatibility.
- [ ] Confirm UART startup, cold reset and two READY signals default to safe levels and recover afterward.
- [ ] Verify CRC32 with `123456789 → CBF43926` and the complete example error frame in Section 6.
- [ ] Verify a valid 99-byte arm frame and valid 47-byte finger frame travel unchanged from Wi-Fi to Teensy.
- [ ] Verify variable-size sensor feedback is transported from Teensy to the wearable without modification.
- [ ] Verify error frames reach local display/log and appropriate upstream destination; simulate queue overflow.
- [ ] Test malformed header, invalid/reserved ID, bad length, corrupted CRC, truncated UART frame, and noise before a valid frame.
- [ ] Test sustained 2 Mbps UART traffic, back-to-back frames, full-duplex traffic and READY flow control with a logic analyzer.
- [ ] Confirm that Wi-Fi silence alone does not close the socket or toggle UART READY; test actual station and AP failure recovery.
- [ ] Confirm that stale commands do not resume unexpectedly after link or producer restart; test this in final consuming controllers.
- [ ] Agree on real error-code definitions and fault/stop policies **before** enabling safety-relevant robotic motion.
- [ ] Update this document whenever a dataset definition, pin assignment, buffer policy or supported transport is changed.

### Expected interfaces from collaborating students

| Work package | Required contribution | Current owner/status |
|---|---|---|
| Arm control | Interpret the allocated 76 bytes, validate targets, define real-time execution rules | HARRP arm-control workstream; application details TBD |
| Palm/fingers | Interpret 24-byte finger application data and implement local actuator protection | Palm workstream; application details TBD |
| Sensors and return stream | Determine sensors, byte representation, sample rates, producer ownership, and consumer requirements | Assigned to another student; **TBD** |
| Errors and safety | Define global 00–99 codebook, severity, local fault response, stop/clear behavior | **TBD** |
| Communication hub | Maintain transport, packet validation, Wi-Fi/UART recovery and local diagnostics | Initial compiling version complete; physical validation pending |
| Thor integration | Implement equivalent frame transport, and separately integrate AI/planning application | Future stage |

---

## 11. Known limitations and deliberately open decisions

| Item | Status | Rationale / next action |
|---|---|---|
| Sensor application payload | **OPEN** | Sensor student will specify fields, data sizes, sampling and return path |
| Sensor sequence ownership | **OPEN** | Multiple independent sensor producers cannot share one sequence comparison without an agreed aggregator or new ID assignment |
| Error-code definitions | **OPEN** | 00–99 globally unique numbering is reserved; meanings and fault origins will be assigned later |
| Stop-triggering errors | **OPEN** | Local fault classification and control reaction to be designed separately |
| Error `00` semantics | **OPEN** | Do not assume global fault clear |
| Session restart acceptance | **OPEN at consumer** | Use an established sequence-restart pattern, possibly RTP-inspired; decide/test at final consuming MCU, not Nano bridge |
| Application value conventions | **OPEN outside transport** | Units, coordinate frames, actuator value encoding and field order belong to arm/palm teams; the hub treats bytes as opaque |
| Teensy input for Nano D2 READY | **OPEN hardware** | Assign and wire the receiving Teensy pin |
| READY timing and parser capacity | **UNVERIFIED ON HARDWARE** | Validate with sustained full-duplex traffic, including backpressure and cold boot |
| Wi-Fi security | **DEVELOPMENT ONLY** | Harden credentials and define authorized sender model before deployment |
| Reliable delivery of every error | **NOT PROVIDED** | Bounded volatile queues, UDP transport and no ACK are insufficient for guaranteed event delivery |
| Compatibility with older HRP2 draft1b | **INCOMPATIBLE** | Update all endpoints together; signature alone cannot distinguish these two drafts |

## 12. Revision history

| Revision | Date | Description |
|---|---|---|
| **0.1** | 2026-11-13 | Initial student handoff draft. Documents fixed-routing HRP2, shared header, dataset registry, allocated sizes, CRC, Nano hardware interfaces, buffering and current open decisions. |

### Revision procedure

When changing the protocol, update **both** the relevant code definitions and this document in the same development cycle. Record: rationale, affected dataset IDs, exact old/new byte layouts, backward compatibility, test evidence, and the developer responsible. Application payload revisions should be owned and reviewed by the corresponding subsystem team.

---

**Project principle:** *Transport validated bytes; execute commands only in the responsible controller. Keep safety decisions local, and keep future dataset definitions explicit rather than inferred.*

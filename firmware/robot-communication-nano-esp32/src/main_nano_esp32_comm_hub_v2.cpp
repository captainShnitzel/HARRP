// HARRP V2 simplified HRP2 bridge, 2026-11-13.
// Adapted from v2.0-draft1b: retain display wiring, CRC utilities, bounded stream
// parsing, USB backpressure and Wi-Fi event flags. Protocol reference is in
// docs/HARRP_V2_Communication_Protocol.md. Configure NANO_READY before wiring.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/uart.h>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include "HARRPProtocol.h"

#ifndef HARRP_NANO_READY_PIN 
#define HARRP_NANO_READY_PIN D2 
#endif
#ifndef HARRP_DIAGNOSTIC_LEVEL
#define HARRP_DIAGNOSTIC_LEVEL 1 // 0 silent, 1 errors/link events, 2 periodic counters.
#endif

namespace Config {
constexpr char AP_SSID[] = "robot";
constexpr char AP_PASSWORD[] = "12345678"; // Existing development configuration.
constexpr uint16_t UDP_PORT = 5005;
constexpr uint32_t UART_BAUD = 2000000;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint8_t TEENSY_READY = D3; // Teensy GPIO27 -> Nano D3, unchanged.
constexpr int NANO_READY = HARRP_NANO_READY_PIN;
constexpr uint8_t LED_RED = 14, LED_GREEN = 15, LED_BLUE = 16;
constexpr uint8_t DISPLAY_ONES[4] = {A0, A1, A2, A3};
constexpr uint8_t DISPLAY_TENS[4] = {A4, A5, A6, A7};
constexpr bool DISPLAY_ENABLED = true;
constexpr int DIAGNOSTIC_LEVEL = HARRP_DIAGNOSTIC_LEVEL;
constexpr size_t UART_RX_BYTES = 2048;
constexpr size_t UART_TX_BYTES = 1024;
constexpr size_t UART_BYTES_PER_PASS = 256;
// Reserve a complete in-flight frame, another frame granted by READY, and FIFO
// headroom. The peer must also gate each new frame and reserve in-flight space.
constexpr size_t RX_READY_HEADROOM = 2 * HRP2::MAX_FRAME + 128;
constexpr uint32_t UART_PARTIAL_TIMEOUT_MS = 20;
constexpr uint8_t UDP_PACKETS_PER_PASS = 4;
constexpr uint8_t ERROR_FORWARD_SLOTS = 8;
constexpr uint8_t ERROR_LOG_SLOTS = 16;
constexpr uint8_t LINK_LOG_SLOTS = 8;
constexpr size_t USB_BYTES_PER_PASS = 96;
constexpr uint32_t WIFI_RETRY_MS = 2000;
constexpr uint32_t WIFI_HEALTH_MS = 250;
constexpr uint32_t LINK_ACTIVITY_MS = 1000;
constexpr uint32_t DIAGNOSTIC_PERIOD_MS = 1000;
static_assert(UART_RX_BYTES > RX_READY_HEADROOM, "RX ring cannot grant a frame");
static_assert(DIAGNOSTIC_LEVEL >= 0 && DIAGNOSTIC_LEVEL <= 2, "Diagnostic level must be 0..2");
}

using HRP2::Frame;
struct PendingFrame { Frame frame; bool pending = false; };
// Only assigned continuous IDs have slots. Already-started TX has separate storage.
PendingFrame toTeensy[2];
PendingFrame toWearableSensor;
struct ErrorFrames {
  Frame frames[Config::ERROR_FORWARD_SLOTS];
  uint8_t head = 0;
  uint8_t count = 0;
} toWearableErrors;

struct Counters {
  uint32_t udpAccepted = 0, uartAccepted = 0;
  uint32_t invalid = 0, crcErrors = 0, unsupported = 0, wrongDirection = 0;
  uint32_t replaced = 0, uartTimeouts = 0, uartFramesSent = 0, udpFramesSent = 0;
  uint32_t errorEvents = 0, errorLogLost = 0, errorForwardLost = 0;
  uint32_t wifiRetries = 0, socketFailures = 0, peerChanges = 0, linkLogLost = 0;
  uint32_t maxLoopUs = 0;
} counters;

struct ErrorEvent { uint32_t receivedMs = 0; uint8_t code = 0; };
ErrorEvent errorLog[Config::ERROR_LOG_SLOTS];
uint8_t errorHead = 0, errorCount = 0;
bool hasDisplayedError = false;
uint8_t displayedError = 0;

enum class LinkEvent : uint8_t { ApUp, ApDown, PeerUp, PeerDown, SocketDown, ReadyUnconfigured, UartUnavailable };
struct LinkReport { uint32_t receivedMs = 0; LinkEvent event = LinkEvent::ApDown; };
LinkReport linkLog[Config::LINK_LOG_SLOTS];
uint8_t linkHead = 0, linkCount = 0;
char usbLine[384] = {};
size_t usbLength = 0, usbOffset = 0;
uint32_t lastStatusMs = 0;

WiFiUDP udp;
std::atomic<uint32_t> wifiEvents{0};
constexpr uint32_t EVENT_AP_STOP = 1, EVENT_DISCONNECT = 2;
bool apReady = false, socketReady = false, peerKnown = false;
IPAddress peerAddress;
uint16_t peerPort = 0;
uint32_t lastUdpMs = 0, lastUartMs = 0;
bool seenUart = false;
bool wifiAttempted = false, socketAttempted = false;
uint32_t lastWifiAttemptMs = 0, lastSocketAttemptMs = 0, lastHealthMs = 0;
uint32_t socketStartedMs = 0;

Frame uartRx, uartTx;
size_t uartTxOffset = 0;
bool uartTxActive = false;
uint8_t nextUartDataset = 0;
uint32_t uartLastByteMs = 0;
bool uartInitialized = false, readyPinOkay = false;

void logLink(LinkEvent event) {
  if (linkCount == Config::LINK_LOG_SLOTS) {
    linkHead = (linkHead + 1) % Config::LINK_LOG_SLOTS;
    --linkCount;
    ++counters.linkLogLost;
  }
  LinkReport& report = linkLog[(linkHead + linkCount) % Config::LINK_LOG_SLOTS];
  report.receivedMs = millis();
  report.event = event;
  ++linkCount;
}

void retainError(const Frame& frame) {
  // Codes have been structurally validated, including the decimal 00..99 bound.
  if (errorCount == Config::ERROR_LOG_SLOTS) {
    errorHead = (errorHead + 1) % Config::ERROR_LOG_SLOTS;
    --errorCount;
    ++counters.errorLogLost;
  }
  ErrorEvent& event = errorLog[(errorHead + errorCount) % Config::ERROR_LOG_SLOTS];
  event.receivedMs = millis();
  event.code = frame.bytes[HRP2::HEADER_SIZE];
  ++errorCount;
  ++counters.errorEvents;
  displayedError = event.code;
  hasDisplayedError = true; // Remains until reboot; code 00 is an ordinary code.
}

void publish(PendingFrame& slot, const Frame& frame) {
  if (slot.pending) ++counters.replaced;
  memcpy(slot.frame.bytes, frame.bytes, frame.length);
  slot.frame.length = frame.length;
  slot.pending = true;
}

void resetPeer() {
  if (peerKnown) { ++counters.peerChanges; logLink(LinkEvent::PeerDown); }
  peerKnown = false;
  peerPort = 0;
  // Drop obsolete unsent traffic from the old connection in both directions.
  // A UART frame already started stays immutable and finishes normally.
  for (PendingFrame& slot : toTeensy) slot.pending = false;
  toWearableSensor.pending = false;
  counters.errorForwardLost += toWearableErrors.count;
  toWearableErrors.head = toWearableErrors.count = 0;
  if (uartTxActive && uartTxOffset == 0) uartTxActive = false;
}


void socketFailed() {
  if (socketReady) {
    ++counters.socketFailures;
    logLink(LinkEvent::SocketDown);
  }
  socketReady = false;
  udp.stop();
  resetPeer();
}

bool validated(const Frame& frame) {
  HRP2::Validation result = HRP2::validate(frame.bytes, frame.length);
  if (result == HRP2::Validation::Valid) return true;
  ++counters.invalid;
  if (result == HRP2::Validation::BadCrc) ++counters.crcErrors;
  if (result == HRP2::Validation::ReservedId) ++counters.unsupported;
  return false;
}

void forwardValidatedFrame(const Frame& frame, bool fromUdp, IPAddress address, uint16_t port) {
  uint8_t id = frame.bytes[4];
  // Fixed paths. Wearable errors terminate at the hub display/log. Robot errors
  // also continue upstream. Reject data arriving against its assigned direction.
  if ((fromUdp && id == HRP2::SENSOR) ||
      (!fromUdp && (id == HRP2::ARM || id == HRP2::FINGER)) ||
      (fromUdp && port == 0)) {
    ++counters.wrongDirection;
    return;
  }
  if (fromUdp) {
    if (peerKnown && (address != peerAddress || port != peerPort)) resetPeer();
    if (!peerKnown) { ++counters.peerChanges; logLink(LinkEvent::PeerUp); }
    peerKnown = true;
    peerAddress = address;
    peerPort = port;
    lastUdpMs = millis();
    ++counters.udpAccepted;
    if (id == HRP2::ERROR) retainError(frame);
    else publish(toTeensy[id], frame); // Only ARM=0 and FINGER=1 can reach here.
  } else {
    seenUart = true;
    lastUartMs = millis();
    ++counters.uartAccepted;
    if (id == HRP2::SENSOR) publish(toWearableSensor, frame);
    else {
      retainError(frame);
      if (toWearableErrors.count == Config::ERROR_FORWARD_SLOTS) {
        ++counters.errorForwardLost; // Preserve earlier unsent error events.
      } else {
        uint8_t tail = (toWearableErrors.head + toWearableErrors.count) % Config::ERROR_FORWARD_SLOTS;
        toWearableErrors.frames[tail] = frame;
        ++toWearableErrors.count;
      }
    }
  }
}

void serviceUartTx() {
  if (!uartInitialized) return;
  if (uartTxActive && uartTxOffset == uartTx.length) {
    // Poll completion without waiting. Do not queue another complete frame in
    // the driver until its predecessor has physically drained and READY is read.
    if (uart_wait_tx_done(UART_NUM_0, 0) != ESP_OK) return;
    uartTxActive = false;
    ++counters.uartFramesSent;
  }
  int capacity = Serial0.availableForWrite();
  if (capacity <= 0) return;
  if (!uartTxActive) {
    if (digitalRead(Config::TEENSY_READY) != HIGH) return;
    for (uint8_t count = 0; count < 2; ++count) {
      uint8_t id = (nextUartDataset + count) % 2;
      if (!toTeensy[id].pending) continue;
      uartTx = toTeensy[id].frame;
      toTeensy[id].pending = false;
      nextUartDataset = (id + 1) % 2;
      uartTxOffset = 0;
      uartTxActive = true;
      break;
    }
    if (!uartTxActive) return;
  }
  // A zero-byte write has not started a frame. Continue honoring READY, and
  // replace such an unstarted snapshot if a more recent value has arrived.
  if (uartTxOffset == 0) {
    PendingFrame& slot = toTeensy[uartTx.bytes[4]];
    if (slot.pending) { uartTx = slot.frame; slot.pending = false; }
    if (digitalRead(Config::TEENSY_READY) != HIGH) return;
  }
  size_t count = uartTx.length - uartTxOffset;
  if (count > size_t(capacity)) count = size_t(capacity);
  if (count > Config::UART_BYTES_PER_PASS) count = Config::UART_BYTES_PER_PASS;
  uartTxOffset += Serial0.write(uartTx.bytes + uartTxOffset, count);
}

void updateNanoReady() {
  if (!readyPinOkay) return;
  int occupied = Serial0.available();
  bool ready = uartInitialized && occupied >= 0 &&
               size_t(occupied) <= Config::UART_RX_BYTES - Config::RX_READY_HEADROOM;
  digitalWrite(Config::NANO_READY, ready ? HIGH : LOW);
}

void discardUartPrefix(size_t count) {
  uartRx.length -= count;
  memmove(uartRx.bytes, uartRx.bytes + count, uartRx.length);
}

void parseUartBuffer() {
  // The scan is bounded by one maximum frame and the per-pass RX byte budget.
  while (uartRx.length >= 4) {
    if (memcmp(uartRx.bytes, HRP2::MAGIC, 4) != 0) { discardUartPrefix(1); continue; }
    if (uartRx.length < HRP2::HEADER_SIZE) return;
    if (HRP2::validateHeader(uartRx.bytes) != HRP2::Validation::Valid) {
      ++counters.invalid;
      if (uartRx.bytes[4] >= 4 && uartRx.bytes[4] <= 15) ++counters.unsupported;
      discardUartPrefix(1);
      continue;
    }
    size_t expected = HRP2::HEADER_SIZE + HRP2::read16(uartRx.bytes + 5) + HRP2::CRC_SIZE;
    if (uartRx.length < expected) return;
    HRP2::Validation result = HRP2::validate(uartRx.bytes, expected);
    if (result != HRP2::Validation::Valid) {
      ++counters.invalid;
      if (result == HRP2::Validation::BadCrc) ++counters.crcErrors;
      discardUartPrefix(1);
      continue;
    }
    Frame frame;
    frame.length = expected;
    memcpy(frame.bytes, uartRx.bytes, expected);
    discardUartPrefix(expected);
    forwardValidatedFrame(frame, false, IPAddress(), 0);
  }
}

void serviceUartRx() {
  if (!uartInitialized) return;
  uint32_t now = millis();
  if (uartRx.length != 0 && Serial0.available() == 0 &&
      uint32_t(now - uartLastByteMs) > Config::UART_PARTIAL_TIMEOUT_MS) {
    // Discard one stalled candidate, but scan its suffix for a complete frame.
    // A truncated packet with a plausible length must not hide its successor.
    discardUartPrefix(1);
    parseUartBuffer();
    uartRx.length = 0;
    ++counters.uartTimeouts;
  }
  updateNanoReady();
  for (size_t count = 0; count < Config::UART_BYTES_PER_PASS; ++count) {
    if (Serial0.available() <= 0) break;
    int value = Serial0.read();
    if (value < 0) break;
    uartLastByteMs = millis();
    uartRx.bytes[uartRx.length++] = uint8_t(value);
    parseUartBuffer();
  }
  updateNanoReady();
}

void serviceUdpRx() {
  if (!socketReady) return;
  for (uint8_t count = 0; count < Config::UDP_PACKETS_PER_PASS; ++count) {
    errno = 0;
    int size = udp.parsePacket();
    int socketError = errno;
    if (size <= 0) {
      if (socketError != 0 && socketError != EAGAIN && socketError != EWOULDBLOCK && socketError != EINTR)
        socketFailed();
      return;
    }
    IPAddress address = udp.remoteIP();
    uint16_t port = udp.remotePort();
    if (size > int(HRP2::MAX_FRAME)) { udp.flush(); ++counters.invalid; continue; }
    Frame frame;
    int received = udp.read(frame.bytes, size_t(size));
    if (received != size) { udp.flush(); ++counters.invalid; continue; }
    frame.length = size_t(received);
    if (validated(frame)) forwardValidatedFrame(frame, true, address, port);
  }
}

bool sendUdp(const Frame& frame) {
  if (!udp.beginPacket(peerAddress, peerPort)) { socketFailed(); return false; }
  size_t count = udp.write(frame.bytes, frame.length);
  bool sent = udp.endPacket() == 1;
  if (count != frame.length || !sent) { socketFailed(); return false; }
  ++counters.udpFramesSent;
  return true;
}

void serviceUdpTx() {
  if (!socketReady || !peerKnown) return;
  // At most one error and one latest sensor frame per loop; neither can starve.
  if (toWearableErrors.count != 0) {
    const Frame& frame = toWearableErrors.frames[toWearableErrors.head];
    if (!sendUdp(frame)) return;
    toWearableErrors.head = (toWearableErrors.head + 1) % Config::ERROR_FORWARD_SLOTS;
    --toWearableErrors.count;
  }
  if (toWearableSensor.pending && sendUdp(toWearableSensor.frame))
    toWearableSensor.pending = false;
}

void onWifiEvent(WiFiEvent_t event) {
  // Arduino invokes this callback in a stack task. Only exchange atomic flags.
  if (event == ARDUINO_EVENT_WIFI_AP_STOP) wifiEvents.fetch_or(EVENT_AP_STOP);
  if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) wifiEvents.fetch_or(EVENT_DISCONNECT);
}

void serviceWifi() {
  uint32_t now = millis();
  uint32_t events = wifiEvents.exchange(0);
  if (events & EVENT_AP_STOP) {
    if (apReady) logLink(LinkEvent::ApDown);
    apReady = false;
    socketFailed();
  } else if (events & EVENT_DISCONNECT) {
    socketReady = false;
    udp.stop(); // Discard old-station datagrams still queued in the socket.
    resetPeer();
  }
  if (uint32_t(now - lastHealthMs) >= Config::WIFI_HEALTH_MS) {
    lastHealthMs = now;
    if (apReady && (WiFi.getMode() != WIFI_AP || WiFi.softAPIP() == IPAddress(0, 0, 0, 0))) {
      apReady = false;
      logLink(LinkEvent::ApDown);
      socketFailed();
    }
    if (apReady && WiFi.softAPgetStationNum() == 0 && peerKnown) {
      socketReady = false;
      udp.stop();
      resetPeer();
    }
  }
  if (!apReady) {
    if (wifiAttempted && uint32_t(now - lastWifiAttemptMs) < Config::WIFI_RETRY_MS) return;
    wifiAttempted = true;
    lastWifiAttemptMs = now;
    ++counters.wifiRetries;
    WiFi.mode(WIFI_AP);
    apReady = WiFi.softAP(Config::AP_SSID, Config::AP_PASSWORD, 1, 0, 1);
    if (!apReady) return;
    logLink(LinkEvent::ApUp);
    socketAttempted = false;
  }
  if (socketReady) return;
  if (socketAttempted && uint32_t(now - lastSocketAttemptMs) < Config::WIFI_RETRY_MS) return;
  socketAttempted = true;
  lastSocketAttemptMs = now;
  udp.stop();
  socketReady = udp.begin(Config::UDP_PORT) == 1;
  socketStartedMs = now;
  if (!socketReady) { ++counters.socketFailures; logLink(LinkEvent::SocketDown); }
}

void writeGlyph(const uint8_t* pins, uint8_t glyph) {
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(pins[i], (glyph >> i) & 1);
}
void showCode(uint8_t left, uint8_t right) {
  if (!Config::DISPLAY_ENABLED) return;
  static uint8_t previous = 0xFF;
  uint8_t code = uint8_t((left << 4) | right);
  if (code == previous) return;
  previous = code;
  writeGlyph(Config::DISPLAY_TENS, left);
  writeGlyph(Config::DISPLAY_ONES, right);
}

void serviceDisplayAndLeds() {
  uint32_t now = millis();
  bool udpAlive = peerKnown && uint32_t(now - lastUdpMs) < Config::LINK_ACTIVITY_MS;
  bool uartAlive = seenUart && uint32_t(now - lastUartMs) < Config::LINK_ACTIVITY_MS;
  // Latest received error persists across link changes. 00 is displayed as 00.
  if (hasDisplayedError) showCode(displayedError / 10, displayedError % 10);
  else if (!apReady || !socketReady) showCode(13, 1); // Baseline E1 link glyph.
  else if (!udpAlive) showCode(0, 1);
  else if (!uartAlive) showCode(0, 3);
  else showCode(0, 7); // Transport activity, never a claim about motion permission.
  bool blink = (now / 300) % 2 != 0;
  digitalWrite(Config::LED_RED, apReady && socketReady ? HIGH : (blink ? LOW : HIGH));
  digitalWrite(Config::LED_GREEN, udpAlive && uartAlive ? LOW : HIGH);
  digitalWrite(Config::LED_BLUE, !udpAlive && blink ? LOW : HIGH);
}

const char* linkName(LinkEvent event) {
  switch (event) {
    case LinkEvent::ApUp: return "AP_UP";
    case LinkEvent::ApDown: return "AP_DOWN";
    case LinkEvent::PeerUp: return "WEARABLE_UP";
    case LinkEvent::PeerDown: return "WEARABLE_DOWN";
    case LinkEvent::SocketDown: return "UDP_SOCKET_DOWN";
    case LinkEvent::ReadyUnconfigured: return "NANO_READY_UNCONFIGURED_OR_CONFLICTING";
    case LinkEvent::UartUnavailable: return "UART_INITIALIZATION_FAILED";
  }
  return "UNKNOWN";
}

void serviceUsbReports() {
  if (Config::DIAGNOSTIC_LEVEL == 0) return;
  if (!Serial) { usbOffset = 0; return; }
  if (usbLength == 0) {
    int length = 0;
    if (errorCount != 0) {
      const ErrorEvent& event = errorLog[errorHead];
      length = snprintf(usbLine, sizeof(usbLine), "ERROR ms=%lu code=%02u lost=%lu\n",
          (unsigned long)event.receivedMs, unsigned(event.code), (unsigned long)counters.errorLogLost);
      errorHead = (errorHead + 1) % Config::ERROR_LOG_SLOTS;
      --errorCount;
    } else if (linkCount != 0) {
      const LinkReport& report = linkLog[linkHead];
      length = snprintf(usbLine, sizeof(usbLine), "LINK ms=%lu %s\n",
                        (unsigned long)report.receivedMs, linkName(report.event));
      linkHead = (linkHead + 1) % Config::LINK_LOG_SLOTS;
      --linkCount;
    } else if (Config::DIAGNOSTIC_LEVEL >= 2 &&
               uint32_t(millis() - lastStatusMs) >= Config::DIAGNOSTIC_PERIOD_MS) {
      lastStatusMs = millis();
      length = snprintf(usbLine, sizeof(usbLine),
          "udp_rx=%lu uart_rx=%lu uart_tx=%lu udp_tx=%lu bad=%lu crc=%lu reserved=%lu dir=%lu replace=%lu err=%lu log_lost=%lu forward_lost=%lu link_lost=%lu timeout=%lu wifi_retry=%lu socket_fail=%lu loop_us=%lu nano_ready_pin=%d\n",
          (unsigned long)counters.udpAccepted, (unsigned long)counters.uartAccepted,
          (unsigned long)counters.uartFramesSent, (unsigned long)counters.udpFramesSent,
          (unsigned long)counters.invalid, (unsigned long)counters.crcErrors,
          (unsigned long)counters.unsupported, (unsigned long)counters.wrongDirection,
          (unsigned long)counters.replaced, (unsigned long)counters.errorEvents,
          (unsigned long)counters.errorLogLost, (unsigned long)counters.errorForwardLost,
          (unsigned long)counters.linkLogLost, (unsigned long)counters.uartTimeouts,
          (unsigned long)counters.wifiRetries, (unsigned long)counters.socketFailures,
          (unsigned long)counters.maxLoopUs, Config::NANO_READY);
    }
    if (length > 0 && size_t(length) < sizeof(usbLine)) usbLength = size_t(length);
  }
  if (usbLength == 0) return;
  int capacity = Serial.availableForWrite();
  if (capacity <= 0) return;
  size_t count = usbLength - usbOffset;
  if (count > size_t(capacity)) count = size_t(capacity);
  if (count > Config::USB_BYTES_PER_PASS) count = Config::USB_BYTES_PER_PASS;
  usbOffset += Serial.write(reinterpret_cast<const uint8_t*>(usbLine + usbOffset), count);
  if (usbOffset == usbLength) usbOffset = usbLength = 0;
}

bool readyPinIsUsable() {
  if (Config::NANO_READY < 0 || Config::NANO_READY == Config::TEENSY_READY ||
      Config::NANO_READY == Config::LED_RED || Config::NANO_READY == Config::LED_GREEN ||
      Config::NANO_READY == Config::LED_BLUE || Config::NANO_READY == D0 || Config::NANO_READY == D1)
    return false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (Config::NANO_READY == Config::DISPLAY_ONES[i] ||
        Config::NANO_READY == Config::DISPLAY_TENS[i]) return false;
  }
  return true; // Logical collision check only; physical assignment needs confirmation.
}

void setup() {
  readyPinOkay = readyPinIsUsable();
  if (readyPinOkay) {
    digitalWrite(Config::NANO_READY, LOW);
    pinMode(Config::NANO_READY, OUTPUT);
  }
  pinMode(Config::TEENSY_READY, INPUT_PULLDOWN);
  pinMode(Config::LED_RED, OUTPUT);
  pinMode(Config::LED_GREEN, OUTPUT);
  pinMode(Config::LED_BLUE, OUTPUT);
  digitalWrite(Config::LED_RED, HIGH);
  digitalWrite(Config::LED_GREEN, HIGH);
  digitalWrite(Config::LED_BLUE, HIGH);
  if (Config::DISPLAY_ENABLED) {
    for (uint8_t i = 0; i < 4; ++i) {
      pinMode(Config::DISPLAY_ONES[i], OUTPUT);
      pinMode(Config::DISPLAY_TENS[i], OUTPUT);
    }
    showCode(0, 0);
  }
  if (Config::DIAGNOSTIC_LEVEL > 0) {
    Serial.begin(Config::USB_BAUD);
    Serial.setTxTimeoutMs(0); // A monitor disappearing mid-write must not stall UART.
  }
  if (!readyPinOkay) logLink(LinkEvent::ReadyUnconfigured);
  WiFi.onEvent(onWifiEvent);
  size_t rxSize = Serial0.setRxBufferSize(Config::UART_RX_BYTES);
  size_t txSize = Serial0.setTxBufferSize(Config::UART_TX_BYTES);
  Serial0.begin(Config::UART_BAUD);
  uartInitialized = rxSize >= Config::UART_RX_BYTES && txSize >= Config::UART_TX_BYTES &&
                    uart_is_driver_installed(UART_NUM_0);
  if (!uartInitialized) logLink(LinkEvent::UartUnavailable);
  serviceWifi();
  updateNanoReady();
}

void loop() {
  uint32_t startUs = micros();
  updateNanoReady();
  serviceWifi(); // Invalidate disconnected peers before servicing old socket input.
  serviceUartRx();
  serviceUartTx();
  serviceUdpRx();
  serviceUartTx();
  serviceUartRx();
  serviceUdpTx();
  serviceDisplayAndLeds();
  serviceUsbReports();
  updateNanoReady();
  uint32_t elapsedUs = uint32_t(micros() - startUs);
  if (elapsedUs > counters.maxLoopUs) counters.maxLoopUs = elapsedUs;
}

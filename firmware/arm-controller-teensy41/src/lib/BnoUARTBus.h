/******************************************************************************
 * File    : BnoUARTBus.h
 * Module  : 7Semi BNO08x — UART Transport Adapter (HDLC-like framing)
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * UART transport for the BNO08x SHTP protocol as seen on some modules that
 * encapsulate SHTP frames in an HDLC-like envelope:
 *
 *   0x7E <PID=0x01> <escaped SHTP bytes> 0x7E
 *
 * Escaping: each 0x7E or 0x7D in the SHTP stream is sent as 0x7D, (byte ^ 0x20).
 *
 * Responsibilities
 * ----------------
 * - begin(): initialize the UART (with optional RX/TX pin selection on ESP32)
 * - tx():    frame + escape SHTP bytes and send
 * - rx():    deframe + unescape a single SHTP frame into caller buffer
 *
 * IMPORTANT
 * ---------
 * Some BNO08x boards expose *raw SHTP over UART* (no 0x7E/0x7D framing). If
 * yours does, adapt tx()/rx() accordingly (remove the HDLC bits).
 *
 * SHTP Receive Contract (BnoBus::rx)
 * ----------------------------------
 * int rx(uint8_t* buf, size_t cap, uint16_t& len)
 * - On success:
 *     - 'len' = FULL SHTP length (header included; from the SHTP 4-byte header)
 *     - returns number of bytes copied into 'buf'
 * - If 'len' > cap:
 *     - the function *drains the rest of the UART frame* to keep the link aligned
 *     - returns 0 (caller should provide a larger buffer)
 * - On timeout or malformed frame: returns 0 (len may remain 0)
 *
 * ESP32 Pin Note
 * --------------
 * If you need custom RX/TX pins on ESP32, pass them to the constructor. This
 * adapter will call:
 *
 *   serial.begin(baud, SERIAL_8N1, rxPin, txPin)   // when pins are provided
//  *
 ******************************************************************************/

#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "BnoBus.h"

struct BnoUARTBus : public BnoBus {
  HardwareSerial* ser;
  uint32_t        baud;
  int             rxPin;
  int             txPin;
  int             intn;
  int             rst;
  /** -------------------------------------------------------------------------
   *  Constructor
   *
   *  - serial : HardwareSerial instance (e.g., Serial1)
   *  - baud   : UART baud rate (1,000,000 bps default)
   *  - rxPin  : optional RX pin (ESP32 only; -1 to use default)
   *  - txPin  : optional TX pin (ESP32 only; -1 to use default)
   *  - intnPin: optional INT pin (-1 if unused)
   *  - rstPin : optional RESET pin (-1 if unused)
   * ----------------------------------------------------------------------- */
  BnoUARTBus(HardwareSerial& serial,
             uint32_t baud = 1000000,
             int rxPin = -1,
             int txPin = -1,
             int intnPin = -1,
            int rstPin = -1)
  : ser(&serial), baud(baud), rxPin(rxPin), txPin(txPin), intn(intnPin),
      rst(rstPin) {}
  /** -------------------------------------------------------------------------
   *  begin()
   *  - Initialize UART peripheral with optional custom RX/TX pins
   *  - Apply RESET sequence if rst pin provided
   * ----------------------------------------------------------------------- */   
  bool begin() override {
    if (!ser) return false;
#if defined(ESP32)
    if (rxPin >= 0 || txPin >= 0) {
      ser->begin(baud, SERIAL_8N1,
                 rxPin < 0 ? -1 : rxPin,
                 txPin < 0 ? -1 : txPin);
    } else {
      ser->begin(baud);
    }
#else
    ser->begin(baud);
#endif
if (intn >= 0) {
      pinMode(intn, INPUT_PULLUP);
    }
    if (rst >= 0) {
      pinMode(rst, OUTPUT);
      digitalWrite(rst, HIGH);
      delay(5);
      digitalWrite(rst, LOW);
      delay(10);
      digitalWrite(rst, HIGH);
      delay(300);
    }
    return true;
  }

    /** -------------------------------------------------------------------------
   *  tx()
   *
   *  - Frame and escape one SHTP frame over UART
   *  - data : pointer to SHTP frame (header + payload)
   *  - n    : number of bytes to send
   *  - return : true if all bytes were sent successfully   
   *  ----------------------------------------------------------------------- */

  bool tx(const uint8_t* data, size_t n) override {
    if (!ser || !data || n == 0) return false;

    ser->write(0x7E);
    ser->write(0x01);

    for (size_t i = 0; i < n; ++i) {
      uint8_t b = data[i];
      if (b == 0x7E || b == 0x7D) {
        ser->write(0x7D);
        ser->write(uint8_t(b ^ 0x20));
      } else {
        ser->write(b);
      }
    }

    ser->write(0x7E);
    ser->flush();
    pump_(5);
    return true;
  }

  void pump_(uint32_t ms) {
    uint8_t pkt[256];
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      rx(pkt, sizeof(pkt));
      yield();
    }
  }
  /** -------------------------------------------------------------------------
   *  rx()
   *
   *  - Deframe and unescape one SHTP frame from UART
   *  - buf  : destination buffer (caller-provided)
   *  - cap  : capacity of 'buf' (must be >= 4 for SHTP header)
   *  - return : number of bytes copied into buf (0 on timeout/error)
   *  ----------------------------------------------------------------------- */
  
  int rx(uint8_t* buf, size_t cap) override {
  if (!ser || !buf || cap < 4) return 0;
  if (intn >= 0) { // optional INT pin
      int i = 0;
      while (digitalRead(intn)) { // wait for INT to go LOW
        i++;
        if (i > 1000) // Timeout after ~100 ms
          return 0;  // nothing to read
      }
    }
  const uint32_t timeout = 50;
  uint32_t deadline;

  // ---- Wait for start flag (0x7E) ----
  deadline = millis() + timeout;
  while ((int32_t)(millis() - deadline) < 0) {
    if (ser->available()) {
      if (ser->read() == 0x7E) break;
      return 0; // invalid start byte
    }
    yield();
  }

  // ---- Read PID ----
  int pid = -1;
  deadline = millis() + timeout;
  while ((int32_t)(millis() - deadline) < 0) {
    if (ser->available()) {
      int c = ser->read();
      if (c != 0x7E) {
        pid = c;
        break;
      }
    }
    yield();
  }
  // Serial.print("Data : ");
  // Serial.println(pid);
  if (pid != 0x01) return 0;

  // ---- Frame parsing ----
  uint8_t hdr[4];
  size_t hdrCnt = 0;
  uint16_t L = 0;
  bool ok = false;
  bool store = true;
  size_t len = 0;
  deadline = millis() + timeout;
  while ((int32_t)(millis() - deadline) < 0) {
    if (!ser->available()) { yield(); continue; }

    int b = ser->read();
    if (b < 0) continue;

    // End of frame
    if (b == 0x7E) break;

    // Escape handling
    if (b == 0x7D) {
      uint32_t escDeadline = millis() + timeout;
      while (!ser->available()) {
        if ((int32_t)(millis() - escDeadline) >= 0) return 0;
        yield();
      }
      b = ser->read();
      if (b < 0) return 0;
      b ^= 0x20;
    }

    // Header (4 bytes)
    if (hdrCnt < 4) {
      hdr[hdrCnt++] = uint8_t(b);
      if (hdrCnt == 4) {
        L = (uint16_t(hdr[0]) | (uint16_t(hdr[1]) << 8)) & 0x7FFF;
        if (L < 4 || L > 256) {
          drainToEndFlag_(timeout);
          return 0;
        }
        store = (L <= cap);
        if (!store) return 0;
        memcpy(buf, hdr, 4);
        len = 4;
        ok = true;
      }
      continue;
    }
    // Serial.print("Data: ");
    // for(int i =0;i<4;i++)
    // {Serial.print(hdr[i]);Serial.print(" ");} Serial.print("");
    // Payload
    if (ok && len < L) {
      buf[len++] = uint8_t(b);
    }
  }

  if (!ok || len < L) return 0;
  return int(len);
}


private:
  bool drainToEndFlag_(unsigned long timeoutMs) {
    unsigned long t = millis();
    while ((millis() - t) < timeoutMs) {
      if (ser->available()) {
        if (ser->read() == 0x7E) return true;
      } else yield();
    }
    return false;
  }
};

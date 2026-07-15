/******************************************************************************
 * File    : BnoI2CBus.h
 * Module  : 7Semi BNO08x — I2C Transport Adapter
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * I2C transport for the BNO08x SHTP protocol. Presents the uniform BnoBus
 * interface (begin/tx/rx) so the higher-level driver stays transport-agnostic.
 *
 * Address Notes
 * -------------
 * - Default 7-bit address is 0x4B (0x4A if AD0 pulled low).
 ******************************************************************************/

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "BnoBus.h"

#if defined(ARDUINO_ARCH_AVR)
#define BNO_WIRE_CHUNK 32
#else
#define BNO_WIRE_CHUNK 128
#endif

/**
 * BnoI2CBus — I²C transport for BNO08x SHTP
 * - Optional custom SDA/SCL pins where supported (ESP32/ESP8266/RP2040/STM32)
 * - Default device address: 0x4B (many boards); 0x4A on some
 * - Implements the unified BnoBus interface (begin/tx/rx)
 */
struct BnoI2CBus : public BnoBus
{
  TwoWire *w;
  int sda;      // use int so -1 sentinel works
  int scl;      // use int so -1 sentinel works
  uint8_t addr; // 7-bit I2C address (0x4B typical)
  uint32_t clk; // I2C clock (Hz)
  int intn;
  int rst;

  /**
   * ctor
   * - wire    : TwoWire instance (default Wire)
   * - sdaPin  : SDA pin (or -1 to use core default)
   * - sclPin  : SCL pin (or -1 to use core default)
   * - i2cAddr : device address (0x4B typical)
   * - clock   : bus speed (400 kHz default)
   */
  BnoI2CBus(TwoWire &wire = Wire,
            int sdaPin = -1,
            int sclPin = -1,
            uint8_t i2cAddr = 0x4B,
            uint32_t clock = 400000,
            int intnPin = -1,
            int rstPin = -1)
      : w(&wire),
        sda(sdaPin),
        scl(sclPin),
        addr(i2cAddr),
        clk(clock),
        intn(intnPin),
        rst(rstPin)
  {}  

  /**
   * Initialize I²C bus (with optional custom pins where supported).
   */
  bool begin() override
  {
    if (!w)
      return false;

    // Some cores allow custom SDA/SCL (order = SDA, SCL). AVR ignores them.
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
    if (sda >= 0 && scl >= 0)
    {
      w->begin(sda, scl);
    }
    else
    {
      w->begin();
    }
#else
    (void)sda;
    (void)scl;
    w->begin();
#endif

    // Set clock if supported by the core
#if defined(TWBR) || defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
    w->setClock(clk);
#endif
    /** INT pin
     *
     *  - Optional
     *  - Uses FALLING edge to signal FIFO has data
     */
 if (intn >= 0) {
      pinMode(intn, INPUT_PULLUP);
    }


    /** Reset sequence (recommended for BNO08x)
     *
     *  - Optional
     *  - Pulse LOW then allow device boot time
     */
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
  /**
   *  tx()
   *  - Transmit a complete SHTP frame (header+payload) in one transaction.
   */
  bool tx(const uint8_t *data, size_t n) override
  {
    if (!w || !data || n == 0)
      return false;
    w->beginTransmission(addr);
    w->write(data, n);
    return (w->endTransmission() == 0);
  }

  /** -------------------------------------------------------------------------
   *  rx()  
   * Receive one SHTP frame.
   * - Reads 4-byte header to obtain total length L (incl. header).
   * - Copies up to 'cap' bytes into 'buf' (partial copy allowed).
   * - Drains any excess bytes to keep the device FIFO aligned.
   * - buf  : destination buffer (caller-provided)
   * - cap  : capacity of 'buf' (must be >= 4 for SHTP header)  
   * - return : number of bytes copied into buf (0 on timeout/error)
   *  ----------------------------------------------------------------------- 
   */
 
  int rx(uint8_t *buf, size_t cap) override
  {
    /** Optional INT gating
     *
     *  - When enabled, rx() will only read after INT asserts
     *  - Can be bypassed for debug sessions
     */
    if (intn >= 0) { // optional INT pin
      int i = 0;
      while (digitalRead(intn)) { // wait for INT to go LOW
        i++;
        if (i > 1000) // Timeout after ~100 ms
          return 0;  // nothing to read
      }
    }
    
    if (!w || !buf || cap < 4)
      return 0;

    const uint32_t timeout = 100;
    uint32_t start = millis();

    while ((millis() - start) < timeout)
    {
      uint8_t count = w->requestFrom((uint8_t)addr, cap);

      if (count < 4)
        continue; // need at least header
      buf[0] = w->read();
      buf[1] = w->read();
      uint16_t len = (uint16_t(buf[0]) | (uint16_t(buf[1]) << 8)) & 0x7FFF;
      if (len < 4 || len > cap)
        continue; // invalid length
      for (uint16_t i = 2; i < len; i++)
      {
        buf[i] = w->read();
      }

      return len; // Read whatever arrives from this request
    }

    // Timeout without valid data
    return 0;
  }
};

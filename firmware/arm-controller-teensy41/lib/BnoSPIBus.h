/******************************************************************************
 * File    : BnoSPIBus.h
 * Module  : 7Semi BNO08x — SPI Transport Adapter
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * SPI transport for the BNO08x SHTP protocol. Presents the uniform BnoBus
 * interface  so the higher-level driver stays transport-agnostic.
 * Configuration options allow INT and RESET pins, SPI clock/mode,
 * and ESP32/ESP8266 custom SPI pin mapping.
 ******************************************************************************/

#pragma once
#include <SPI.h>
#include <Arduino.h>

#include "BnoBus.h"

/** ---------------------------------------------------------------------------
 *  BNO08x SPI transport configuration
 *
 *  - Uses standard 4-byte SHTP header
 *  - SPI_MODE0 and MSB-first expected by BNO08x
 *  - Default clock is 1 MHz (safe for most boards)
 *  ------------------------------------------------------------------------- */


/** ---------------------------------------------------------------------------
 *  Globals (single-device HAL)
 *
 *  - Used by the ISR and bus instance as a shared HAL layer
 *  - Assumes only a single BNO08x device is active on SPI
 *  - Allows static ISR to signal instance logic using g_intFlag
 *  ------------------------------------------------------------------------- */
static SPISettings g_settings(1000000, MSBFIRST, SPI_MODE0);

/** ---------------------------------------------------------------------------
 *  BnoSPIBus
 *
 *  - Implements the generic BnoBus transport interface
 *  - Provides SPI begin(), tx(), and rx() primitives for SHTP
 *  - Supports optional INT and RESET pins
 *  - Supports custom SPI pin mapping
 *  ------------------------------------------------------------------------- */
struct BnoSPIBus : public BnoBus {
  SPIClass *spi;
  int cs;
  int intn;
  int rst;
  uint32_t clk;
  uint8_t mode;
  int sck, miso, mosi;

  /** -------------------------------------------------------------------------
   *  Constructor
   *
   *  - Allows SPI object override (SPI, HSPI, VSPI, etc.)
   *  - Allows optional INT and RESET pins
   *  - Allows setting SPI clock + mode
   *  ----------------------------------------------------------------------- */
  BnoSPIBus(SPIClass &spi = SPI,
            int csPin = SS,
            int intnPin = -1,
            int rstPin = -1,
            uint32_t clock = 1000000,
            uint8_t spiMode = SPI_MODE0,
            int sckPin = -1,
            int misoPin = -1,
            int mosiPin = -1)
    : spi(&spi),
      cs(csPin),
      intn(intnPin),
      rst(rstPin),
      clk(clock),
      mode(spiMode),
      sck(sckPin),
      miso(misoPin),
      mosi(mosiPin) {}

  /** -------------------------------------------------------------------------
   *  begin()
   *
   *  - Initializes SPI peripheral and chip select pin
   *  - Configures INT pin interrupt if provided
   *  - Performs optional reset pulse sequence for BNO08x
   *  - Stores configuration into global variables
   *
   *  Notes:
   *  - Returns false if SPI object is invalid or CS pin is not set
   *  - INT pin is configured using INPUT_PULLUP and FALLING edge trigger
   *  ----------------------------------------------------------------------- */
  bool begin() override {
    if (!spi || cs < 0)
      return false;
    g_settings = SPISettings(clk, MSBFIRST, mode);

    /** SPI init
     *
     *  - Uses custom pins for SPI
     *  - Other MCUs rely on default SPI pins
     */
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
    if (sck >= 0 && miso >= 0 && mosi >= 0)
      spi->begin(sck, miso, mosi, cs);
    else
      spi->begin();
#else
    spi->begin();
#endif

    /** CS pin
     *
     *  - Active LOW
     *  - Default idle state HIGH
     */
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);

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

  /** -------------------------------------------------------------------------
   *  tx()
   *
   *  - Writes a raw SHTP packet to the device
   *  - Uses SPI transactions for safe multi-device bus usage
   *  - Transfers bytes using platform-appropriate method
   *
   *  Notes:
   *  - ESP32 supports writeBytes() for fast DMA-style write
   *  - Other MCUs use transfer() loop for compatibility
   *  ----------------------------------------------------------------------- */
  bool tx(const uint8_t *data, size_t n) override {
    if (!data || n == 0) return false;

    spi->beginTransaction(g_settings);
    digitalWrite(cs, LOW);
    delayMicroseconds(5);

#if defined(ARDUINO_ARCH_ESP32)
     spi->writeBytes(data, n);
     spi->writeBytes(data, n);
#else
    for (size_t i = 0; i < n; i++)
      spi->transfer(data[i]);
       for (size_t i = 0; i < n; i++)
      spi->transfer(data[i]);
#endif
    delayMicroseconds(5);
    digitalWrite(cs, HIGH);
    spi->endTransaction();

    return true;
  }

  /** -------------------------------------------------------------------------
   *  rx()
   *
   *  - Reads one or more packets from the BNO08x FIFO
   *  - Handles continuation packets (MSB of length set)
   *  - Copies the raw SHTP packet stream into caller buffer
   *
   *  Inputs:
   *  - buf    : output buffer to store packet(s)
   *  - cap    : buffer size (max allowed storage)
   *  - len    : populated with number of valid bytes received
   *
   *  Output:
   *  - Returns number of bytes read (0 if nothing / failure)
   *
   *  Notes:
   *  - INT-based gating is intentionally bypassable for debug
   *  - Each packet includes its own 4-byte header
   *  - Stops if packet size invalid or buffer would overflow
   *  ----------------------------------------------------------------------- */
  int rx(uint8_t *buf, size_t cap) override {
    uint16_t len = 0;
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
    while (true) {
      uint8_t hdr[4]; // SHTP header buffer

      spi->beginTransaction(g_settings); // start SPI transaction
      digitalWrite(cs, LOW); // select device

      /** Read 4-byte SHTP header
       *
       *  - First 2 bytes: packet length (with continuation flag)
       *  - Next 2 bytes : channel + sequence
       */
      for (int i = 0; i < 4; i++)
        hdr[i] = spi->transfer(0x00); // dummy write to clock data out
      /** Parse packet length and continuation flag */
      uint16_t pktLen = hdr[0] | (hdr[1] << 8);
      bool ok = pktLen & 0x8000;  // MSB = ok to continue
      pktLen &= ~0x8000;              // clear MSB
      /** Packet sanity check
       *
       *  - pktLen must at least include header
       *  - total must fit in user buffer
       */
      if (pktLen < 4 || pktLen > cap) {
        digitalWrite(cs, HIGH);
        spi->endTransaction();
        break;
      }

      /** Copy header into buffer */
      memcpy(buf + len, hdr, 4);

      /** Read payload bytes
       *
       *  - Payload length = pktLen - headerLen
       *  - Uses dummy reads (0xFF) to clock data out
       */
      uint16_t payloadLen = pktLen - 4; // exclude header
      //  Read payload bytes
      for (uint16_t i = 0; i < payloadLen; i++) {
        buf[len + 4 + i] = spi->transfer(0xFF);// dummy write to clock data out
      }

      digitalWrite(cs, HIGH); // deselect device
      spi->endTransaction();// end SPI transaction
      len += pktLen;// update total length
      /** Exit when FIFO drained */
      if (!ok) break;
    }
    delay(3);
    /** Clear interrupt flag after draining FIFO */
    return len;
  }
};

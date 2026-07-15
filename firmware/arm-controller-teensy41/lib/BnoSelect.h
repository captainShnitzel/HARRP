#pragma once

/**
 * Compile-time bus selection
 * - Define exactly one: BNO_USE_I2C / BNO_USE_SPI / BNO_USE_UART
 * - Includes only the chosen transport
 */
#if !defined(BNO_USE_I2C) && !defined(BNO_USE_SPI) && !defined(BNO_USE_UART)
  #define BNO_USE_I2C
#endif

#if (defined(BNO_USE_I2C) + defined(BNO_USE_SPI) + defined(BNO_USE_UART)) != 1
  #error "Define exactly one: BNO_USE_I2C or BNO_USE_SPI or BNO_USE_UART"
#endif

#ifdef BNO_USE_I2C
  #include "BnoI2CBus.h"
  using BnoSelectedBus = BnoI2CBus;
#endif
#ifdef BNO_USE_SPI
  #include "BnoSPIBus.h"
  using BnoSelectedBus = BnoSPIBus;
#endif
#ifdef BNO_USE_UART
  #include "BnoUARTBus.h"
  using BnoSelectedBus = BnoUARTBus;
#endif
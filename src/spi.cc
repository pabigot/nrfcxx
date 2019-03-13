// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

/* Implementation for nRF51 device series SPI peripheral interface.
 */

#include <nrfcxx/periph.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>

namespace nrfcxx {
namespace periph {

SPI::ssize_type
SPI::bus_configure (int psel_sck,
                    int psel_mosi,
                    int psel_miso,
                    uint32_t frequency,
                    uint32_t config)
{
  error_type ec{};

  do {
    /* Save the previous PPI index then clear the configuration. */
    int ppidx = configuration_.ppidx;
    configuration_ = {};
    configuration_.ppidx = ppidx;

    if ((0 > psel_sck)
        || (!frequency)) {
      ec = ERR_INVALID;
      break;
    }
    configuration_.psel_sck = psel_sck;
    configuration_.psel_mosi = psel_mosi;
    configuration_.psel_miso = psel_miso;
    configuration_.frequency = frequency;
    configuration_.config = config;
  } while (0);
  return error_encoded(ec);
}

SPI::error_type
SPI::set_enabled_ (bool enabled)
{
  error_type rv{};
  spi_->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
  if (enabled) {
    if (0 > configuration_.psel_sck) {
      rv = ERR_INVALID;
    } else {
      spi_->PSEL.SCK = configuration_.psel_sck;
      spi_->PSEL.MOSI = configuration_.psel_mosi;
      spi_->PSEL.MISO = configuration_.psel_miso;
      spi_->FREQUENCY = configuration_.frequency;
      spi_->CONFIG = configuration_.config;
      spi_->EVENTS_READY = 0;
      spi_->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
    }
  }
  return rv;
}

SPI::ssize_type
SPI::tx_rx (const uint8_t* tx_data,
            size_type tx_len,
            size_type rx_len,
            uint8_t* rx_data,
            uint8_t tx_dummy)
{
  size_type transaction_length = tx_len + rx_len;
  size_type i{};

  uint8_t* rxp = rx_data;
  bool need_flush{0 < transaction_length};

  /* This is a little tricky because the peripheral has two slots in
   * each of TXD and RXD.  We write up to two values to start, write
   * one more each time we get a READY and there's more data, and wait
   * for the final READY after last byte written (the "flush"). */
  while ((i < transaction_length)
         || need_flush) {
    if (i < transaction_length) {
      uint8_t txb = tx_dummy;
      if (i < tx_len) {
        txb = tx_data[i];
      }
      spi_->TXD = txb;
      if (1 == ++i) {
        // First transmission: bypass wait for space in TXB
        continue;
      }
    } else {
      // Already wrote the last byte, now we're waiting for its ack.
      need_flush = false;
    }

    /* Busy-wait until there's space in the TXB FIFO (signalled by
     * there being data in the RXD FIFO).  Clear the signal so we
     * detect the one that's generated when the next octet shifts into
     * RXD.  Then read the value to release the peripheral for the
     * next octet. */
    while (!spi_->EVENTS_READY) {
    }
    spi_->EVENTS_READY = 0;
    uint8_t rxb = spi_->RXD;
    if (rxp) {
      *rxp++ = rxb;
    }
  }
  return transaction_length;
}

} // namespace periph
} // namespace nrfcxx

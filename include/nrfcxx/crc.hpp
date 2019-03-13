/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Module that declares various useful checksums.
 *
 * @file */

#ifndef NRFCXX_CRC_HPP
#define NRFCXX_CRC_HPP
#pragma once

#include <pabigot/crc.hpp>

#include <nrfcxx/core.hpp>

namespace nrfcxx {

/** Namespace providing pre-defined checksum algorithms.
 * @see http://users.ece.cmu.edu/~koopman/crc/index.html
 */
namespace crc {

/** CRC-16/DNP.
 *
 * This is optimized for small packets, significantly outperforming
 * CRC-16/CCITT and its variants.
 *
 * @see http://reveng.sourceforge.net/crc-catalogue/16.htm#crc.cat.crc-16-dnp
 * @see http://users.ece.cmu.edu/~koopman/crc/index.html
 */
using crc16dnp_type = pabigot::crc::crc<16, 0x3d65, true, true, 0, -1>;

/** The tabler to use for CRC-16/DNP. */
extern crc16dnp_type::tabler_type crc16dnp;

/** CRC-32 for standard operations.
 *
 * This matches the libarchive/pkzip crc32 utility.
 *
 * @see http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.32
 */
using crc32_type = pabigot::crc::CRC32; // crc<32, 0x04c11db7, true, true, -1, -1>;

/** The tabler to use for CRC-32. */
extern crc32_type::tabler_type crc32;

} // ns crc
} // ns nrfcxx

#endif /* NRFCXX_CRC_HPP */

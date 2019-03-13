// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <nrfcxx/crc.hpp>

namespace nrfcxx {
namespace crc {

crc16dnp_type::tabler_type crc16dnp{crc16dnp_type::instantiate_tabler()};
crc32_type::tabler_type crc32{crc32_type::instantiate_tabler()};

} // ns crc
} // ns nrfcxx

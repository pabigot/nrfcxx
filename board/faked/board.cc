// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/led.hpp>
#include <array>

namespace nrfcxx {
namespace led {

namespace {

led_type led0;
led_type led1;
led_type led2;
led_type led3;
led_type led4;

std::array<led_type*, 5> leds = {&led0, &led1, &led2, &led3, &led4};

} // ns anonymous

led_type** const led_type::ledps_ = &leds[0];
uint8_t const led_type::nleds_ = leds.max_size();

} // ns led
} // ns nrfcxx

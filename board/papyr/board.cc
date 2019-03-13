// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <array>

#include <nrfcxx/led.hpp>

namespace nrfcxx {
namespace board {

// NB: Pin assignment reordered from vendor assignment to produce
// standard RGB ordering.
gpio::gpio_pin led0pin{14};     // Red, very weak
gpio::gpio_pin led1pin{13};     // Green, very strong
gpio::gpio_pin led2pin{15};     // Blue, slightly weak

} // ns board

namespace led {
namespace {

generic_led<board::led_active_low> led0{board::led0pin};
generic_led<board::led_active_low> led1{board::led1pin};
generic_led<board::led_active_low> led2{board::led2pin};

std::array<led_type*, 3> leds = {&led0, &led1, &led2};

} // ns anonymous

led_type** const led_type::ledps_ = &leds[0];
uint8_t const led_type::nleds_ = leds.max_size();

} // ns led

} // ns nrfcxx

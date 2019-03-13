// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/led.hpp>
#include <array>

namespace nrfcxx {
namespace board {

gpio::gpio_pin led0pin{16};     // red
gpio::gpio_pin led1pin{12};     // green
gpio::gpio_pin led2pin{15};     // blue
gpio::gpio_pin led3pin{24};

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

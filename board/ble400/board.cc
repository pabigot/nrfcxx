// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <array>

#include <nrfcxx/led.hpp>

namespace nrfcxx {
namespace board {

gpio::gpio_pin led0pin{18};
gpio::gpio_pin led1pin{19};
gpio::gpio_pin led2pin{20};
gpio::gpio_pin led3pin{21};
gpio::gpio_pin led4pin{22};

} // ns board

namespace led {
namespace {

generic_led<board::led_active_low> led0{board::led0pin};
generic_led<board::led_active_low> led1{board::led1pin};
generic_led<board::led_active_low> led2{board::led2pin};
generic_led<board::led_active_low> led3{board::led3pin};
generic_led<board::led_active_low> led4{board::led4pin};

std::array<led_type*, 5> leds = {&led0, &led1, &led2, &led3, &led4};

} // ns anonymous

led_type** const led_type::ledps_ = &leds[0];
uint8_t const led_type::nleds_ = leds.max_size();

} // ns led
} // ns nrfcxx

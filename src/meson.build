# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

subdir('common')

lib_src = [
  common_src,
]

if meson.is_cross_build()
  # Initialize series_src with series-specific material
  if device.startswith('NRF51')
    subdir('nrf51')
  elif device.startswith('NRF52')
    subdir('nrf52')
  endif

  # Add the common series source files
  series_src += [
    'clock.cc',
    'gpio.cc',
    'gpiote.cc',
    'periph.cc',
    'ppi.cc',
    'rng.cc',
    'twi.cc',
    'spi.cc',
    'uart.cc',
  ]
  # Add nRF52840 peripheral source files
  if 'NRF52840' == device
    series_src += [
      'qspi.cc',
    ]
  endif

  subdir('misc')
  subdir('newlib')
  subdir('sensor')
  if 'blank' != softdevice
    subdir('sd')
  endif

  lib_src += [
    board_src,
    series_src,
    startup_s,
    system_c,
    series_src,
    'core.cc',
    'crc.cc',
    'led.cc',
    'lpm.cc',
    'utility.cc',
    misc_src,
    sensor_src,
  ]

  nrfcxx_lib = static_library('nrfcxx', lib_src,
                              dependencies: pabigot_dep,
                              include_directories: nrfcxx_inc,
                              implicit_include_directories: false,
                              pic: false)

else
  nrfcxx_lib = library('nrfcxx', lib_src,
                       include_directories: nrfcxx_inc,
                       dependencies: pabigot_dep,
                       implicit_include_directories: false)
endif

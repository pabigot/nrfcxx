# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

sensor_src = files('adc.cc',
                   'button.cc',
                   'ccs811.cc',
                   'contact.cc',
                   'hts221.cc',
                   'lps22hb.cc',
                   'sdp8xx.cc',
                   'sht21.cc',
                   'sht31.cc',
                   'utils.cc')

sensor_bme280_lib = static_library('sensor_bme280', 'bme280.cc',
                                   dependencies: [
                                     pabigot_dep,
                                     bme280_dep,
                                   ],
                                   include_directories: nrfcxx_inc,
                                   implicit_include_directories: false,
                                   pic: false)

# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

ex_sensor_bases = [
  'adc',
  'button',
  'caladc',
  'ccs811',
  'ccs811-fw',
  'contact',
  'ntcThermistor',
  'sdp8xx',
  'sht21',
  'sht31',
  'vdd',
]
if board == 'thingy52'
  ex_sensor_bases += [
    'hts221',
    'lps22hb',
    'vdivider',
  ]
endif
if board == 'xenon'
  ex_sensor_bases += [
    'vdivider',
  ]
endif
if board == 'xenon' or board == 'pca10040'
  ex_sensor_bases += [
    'saadc-burst',
  ]
endif

foreach base : ex_sensor_bases
  executable(base, base + '.cc',
             link_args: [
               nosd_memory_link_arg,
               ldargs_for_exe,
               '-Wl,-Map,' + join_paths(meson.current_build_dir(), base + '.map'),
             ],
             dependencies: nrfcxx_dep,
             implicit_include_directories: false,
             name_suffix: 'elf')
endforeach

executable('bme280', 'bme280.cc',
           link_args: [
             nosd_memory_link_arg,
             ldargs_for_exe,
             '-Wl,-Map,' + join_paths(meson.current_build_dir(), base + '.map'),
           ],
           link_with: sensor_bme280_lib,
           dependencies: [nrfcxx_dep, bme280_dep],
           implicit_include_directories: false,
           name_suffix: 'elf')

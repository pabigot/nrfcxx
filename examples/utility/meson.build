# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

ex_utility_bases = [
  'display',
  'i2cdetect',
  'memory_pool',
]
if board == 'xenon'
  ex_utility_bases += [
    'sfdp',
  ]
endif
if board == 'pca10056'
  ex_utility_bases += [
    'sfdp',
  ]
endif

foreach base : ex_utility_bases
  executable(base, base + '.cc',
             link_args: [nosd_memory_link_arg, ldargs_for_exe],
             dependencies: nrfcxx_dep,
             implicit_include_directories: false,
             name_suffix: 'elf')
endforeach

executable('persist', 'persist.cc',
           link_args: [
             '-Wl,-T,' + meson.current_source_dir() + '/persist-' + device + '_XX' + variant + '.mld',
             ldargs_for_exe,
           ],
           dependencies: nrfcxx_dep,
           implicit_include_directories: false,
           name_suffix: 'elf')

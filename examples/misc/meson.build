# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

# Examples that do depend on libnosys
ex_misc_bases = [
  'calsaadc',
  'randio',
  'sx1509b',
]
if board == 'thingy52'
  ex_misc_bases += [
    'lipomon',
  ]
endif
if board == 'xenon'
  ex_misc_bases += [
    'lipomon',
  ]
endif
if bfdata.get('spi1', false)
endif

foreach base : ex_misc_bases
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

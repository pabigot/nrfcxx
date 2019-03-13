# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

# Examples that don't depend on libnosys
ex_bare_bases = [
  'led',
  'nop',
  'uart',
]
foreach base : ex_bare_bases
  executable(base, base + '.cc',
             link_args: [
               nosd_memory_link_arg,
               ldargs_for_exe,
               '-Wl,-Map,' + join_paths(meson.current_build_dir(), base + '.map'),
             ],
             dependencies: nrfcxx_bare_dep,
             implicit_include_directories: false,
             name_suffix: 'elf')
endforeach

# Examples that do depend on libnosys
ex_nosys_bases = [
  'alarm',
  'clock',
  'cstdio',
  'delay',
  'devinfo',
  'events',
  'extwdog',
  'gpiote',
  'instr_psel',
  'led_pattern',
  'off',
  'playground',
  'power',
  'rng',
  'sbrk',
  'systemstate',
]
if board == 'thingy52'
  ex_nosys_bases += 'thingy52'
endif

foreach base : ex_nosys_bases
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

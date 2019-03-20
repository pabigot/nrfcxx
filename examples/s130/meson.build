# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

foreach base : [
  'nop',
]
  executable('sd-' + base, base + '.cc',
             link_args: [
               sd_memory_link_arg,
               ldargs_for_exe,
               '-Wl,-Map,' + join_paths(meson.current_build_dir(), base + '.map'),
             ],
             dependencies: [
               softdevice_dep,
               nrfcxx_dep,
             ],
             implicit_include_directories: false,
             name_suffix: 'elf')
endforeach

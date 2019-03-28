# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

# Broadcaster implementations
foreach base : [
  'beacon',
  'stdbeacon',
]
  executable(base, base + '.cc',
           link_with: nrfsdbc_lib,
           link_args: [
             sd_memory_link_arg,
             ldargs_for_exe,
             '-Wl,-Map,' + join_paths(meson.current_build_dir(), base + '.map'),
           ],
           dependencies: [
             softdevice_dep,    # first to override weak defn in nrfcxx
             nrfcxx_dep,
           ],
           implicit_include_directories: false,
           name_suffix: 'elf')
endforeach

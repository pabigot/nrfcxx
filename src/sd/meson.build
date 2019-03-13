# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

sd_src = files('beacon.cc')

nrfsd_lib = static_library('nrfsd', sd_src,
                           dependencies: [
                             pabigot_dep,
                           ],
                           include_directories: [
                             nrfcxx_inc,
                             softdevice_inc,
                           ],
                           implicit_include_directories: false,
                           pic: false)

nrfsdbc_lib = static_library('nrfsdbc', 'broadcaster.cc',
                             dependencies: [
                               pabigot_dep,
                             ],
                             include_directories: [
                               nrfcxx_inc,
                               softdevice_inc,
                             ],
                             implicit_include_directories: false,
                             pic: false)

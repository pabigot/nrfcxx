# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

sd_src = files(
  'beacon.cc',
  'core.cc',
)

# When linking with sd/core we need to tell the build system to
# discard the weak nrfcxx::systemState::die_temperature_ that comes
# from libnrfcxx.a.  Otherwise that takes priority and we get a
# soft device fault when sensor:adc::calibrator checks for things.
nrfsd_link_args = [
  '-Wl,--undefined=_ZN6nrfcxx11systemState16die_temperature_Ev',
]

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

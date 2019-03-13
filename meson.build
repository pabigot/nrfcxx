# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

project('nrfcxx', ['cpp', 'c'],
        version: '0.1.0',
        default_options: [
          'c_std=c11',
          'cpp_std=c++17',
          'werror=true',
        ])

# Material used to generate files.
cdata = configuration_data()
cdata.set('PROJECT', meson.project_name())
cdata.set('PROJECT_UC', meson.project_name().to_upper())
version_split = meson.project_version().split('-')
cdata.set('VERSION', meson.project_version())
version_components = version_split[0].split('.')
cdata.set('VERSION_MAJOR', version_components[0])
cdata.set('VERSION_MINOR', version_components[1])
cdata.set('VERSION_PATCH', version_components[2])
if version_split.length() == 1
  cdata.set('VERSION_RELEASE', 'RELEASE')
  cdata.set('VERSION_DEVEL', 1)
else
  version_dev = version_split[1].split('.')
  if version_dev[0] != 'dev'
    error('In the project version attribute: @0@ should be dev'.format(version_dev[0]))
  endif
  cdata.set('VERSION_RELEASE', 'DEVEL')
  cdata.set('VERSION_DEVEL', version_dev[1])
  version_components += version_dev[1]
endif

doxygen = find_program('doxygen', required: false)
if find_program('dot', required: false).found()
  cdata.set('HAVE_DOT', 'YES')
else
  cdata.set('HAVE_DOT', 'NO')
endif

# Extends then uses cdata.
subdir('doc')

bfdata = configuration_data()
board = get_option('board')
if 0 == run_command(find_program('test'), '-f', board + '/meson.build').returncode()
  message('Reading board config from ' + board)
  subdir(board)
else
  message('Using standard board ' + board)
  subdir('board/' + board)
endif

# Set up the global arguments based on target platform
if meson.is_cross_build()
  # Now device, variant, board_inc, and board_src are available

  # Confirm that the device is compatible with the cross
  # configuration.
  if not device.startswith(meson.get_cross_property('series').to_upper())
    error('cross ' + meson.get_cross_property('series')
          + ' incompatible with board ' + board
          + ' device ' + device)
  endif

  subdir('softdevices')

  mflags_for_target = meson.get_cross_property('mflags')
  cflags_for_target = ['-D' + device,
                       '-DNRFCXX_BOARD_IS_' + board.to_upper(),
                       '-DNRFCXX_SOFTDEVICE_IS_' + softdevice.to_upper(),
                       '-ffunction-sections',
                       '-fdata-sections',
                       '-Os']
  # nrfx mdk device identifiers include variant information, which has
  # no effect on non-linker operations.  That header is referenced by
  # the mdk system source, so we need a define for it.
  if device.startswith('NRF52')
    cflags_for_target += '-D' + device + '_XX' + variant
  endif
  ldflags_for_target = ['-Wl,-gc-sections']

  add_global_arguments(mflags_for_target,
                       cflags_for_target,
                       language: 'c')
  add_global_arguments(mflags_for_target,
                       cflags_for_target,
                       '-fno-exceptions',
                       '-fno-rtti',
                       language: 'cpp')
  # Required linker flags for cross-compile
  ldargs_for_target = [ldflags_for_target,
                       mflags_for_target]

  # link args when building executables.  Must be preceded by
  # memory_link_arg or substitute.
  ldargs_for_exe = ['-specs=nano.specs',
                    '-specs=nosys.specs',
                    '-Wl,-T,' + meson.source_root() + '/toolchain/GCC/gcc_arm.ld',
                    '-Wl,--undefined=_sbrk',
                    '-Wl,--undefined=_read',
                    '-Wl,--undefined=_write',
                   ]
  add_global_link_arguments(ldargs_for_target,
                            language: 'c')
  add_global_link_arguments(ldargs_for_target,
                            language: 'cpp')

  if 'blank' != softdevice
    # link_args to specify default memory map
    sd_memory_link_arg = ''.join(['-Wl,-T,',
                                  meson.source_root(),
                                  '/toolchain/GCC/' + device.to_lower(),
                                  '/' + softdevice,
                                  '/xx' + variant.to_lower(),
                                  '/memory.ld'])
  endif

  # link_args to specify default memory map
  nosd_memory_link_arg = ''.join(['-Wl,-T,',
                                  meson.source_root(),
                                  '/toolchain/GCC/' + device.to_lower(),
                                  '/blank/xx' + variant.to_lower(),
                                  '/memory.ld'])

  # Dependencies that affect included meson.build content
  bme280_dep = subproject('bme280').get_variable('bme280_dep')

else
  device = 'NRF51'

  cflags_for_host = [
    '-D' + device,
    '-DNRFCXX_CROSS_COMPILING=0',
    '-DNRFCXX_FAKED=1',
  ]
  add_global_arguments(cflags_for_host,
                       language: 'c')
  add_global_arguments(cflags_for_host,
                       language: 'cpp')
endif

pabigot_dep = subproject('pabigot-cxx',
                          default_options: [
                            'googletest=false',
                            'fullcpp=' + (meson.is_cross_build() ? 'false' : 'true'),
                            'support=false',
                          ]).get_variable('pabigot_dep')
nrfx_sp = subproject('nrfx', default_options: ['device=' + device])
mdk_inc = nrfx_sp.get_variable('mdk_inc')

# Build the generated include files
subdir('include/nrfcxx')

# Include files for all sources.  This is in decreasing priority
# order, though there should be no conflicts.
nrfcxx_inc = [
  board_inc,
  include_directories('include',
                      'toolchain/CMSIS'),
  mdk_inc,
]

# Determines startup_s based on device
subdir('toolchain/GCC')
system_c = nrfx_sp.get_variable('system_c')

subdir('src')
nrfcxx_bare_dep = declare_dependency(link_with: nrfcxx_lib,
                                     dependencies: pabigot_dep,
                                     include_directories: nrfcxx_inc)
# Flags and components that are target-specific
if meson.is_cross_build()
  nrfcxx_dep = declare_dependency(link_with: [
                                  nosys_lib,
                                  nrfcxx_lib,
                                ],
                                 dependencies: pabigot_dep,
                                 include_directories: nrfcxx_inc)

  gmock_dep = disabler()
  gtest_dep = disabler()

  if 'blank' == softdevice
    softdevice_dep = disabler()
  else
    softdevice_dep = declare_dependency(include_directories: softdevice_inc,
                                        link_with: nrfsd_lib)
  endif

  subdir('examples')
else
  gtest_sp = subproject('gtest')
  gmock_dep = gtest_sp.get_variable('gmock_main_dep')
  gtest_dep = gtest_sp.get_variable('gtest_main_dep')
  softdevice_dep = disabler()

  subdir('tests')
endif


# These run commands assume a single JLink device.  If multiple JLink
# devices are available the command will be directed to the one with a
# serial number provided by the JSN environment variable, or to the
# one with the lowest serial number of JSN is not defined.
#
# For program pass the name of the image to store as a path relative
# to the build root via the ELF environment variable.
#
# NOTE: The order of declaration reflects the order of execution if
# multiple targets are provided to ninja.  This should support:
#
#     ninja erase flash_softdevice program
#
# as a single command.
run_target('erase',
           command: [
             'scripts/erase',
             device,
           ])
if softdevice_dep.found()
  run_target('flash_softdevice',
             command: [
               'scripts/flash_softdevice',
               device,
               softdevice_hex,
             ])
endif
run_target('program',
           command: [
             'scripts/program',
             device,
             meson.build_root(),
           ])
run_target('reset',
           command: [
             'scripts/reset',
             device,
           ])

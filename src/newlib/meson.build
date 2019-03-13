# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

nosys_lib = static_library('nosys', [
                            'cstdio.cc',
                            'sbrk.c',
                            'nosys.c',
                           ],
                           dependencies: pabigot_dep,
                           include_directories: nrfcxx_inc,
                           implicit_include_directories: false,
                           pic: false)

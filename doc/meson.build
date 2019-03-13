# SPDX-License-Identifier: Apache-2.0
# Copyright 2018 Peter A. Bigot

cdata.set('TOP_SRCDIR', meson.source_root())
cdata.set('TOP_BUILDDIR', meson.build_root())

datadir = join_paths(get_option('datadir'), 'doc')
cdata.set('DOCDIR', 'doc')
cdata.set('PROJECT_UC', meson.project_name().to_upper())

if doxygen.found() and not meson.is_cross_build()
  doxycfg = configure_file(input: 'doxygen.cfg.in',
                           output: 'doxygen.cfg',
                           configuration: cdata,
                           install: false)

  html_target = custom_target('docs',
                              input: doxycfg,
                              output: 'html',
                              command: [doxygen, doxycfg],
                              install: true,
                              install_dir: datadir)
endif

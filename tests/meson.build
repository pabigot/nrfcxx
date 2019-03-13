# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

test_names = [
  'clock',
  'sensor_utils',
  'stub',
]

foreach base: test_names
  gt = 'gt_' + base
  test(base, executable(gt, gt + '.cc',
                        dependencies: [
                          nrfcxx_bare_dep,
                          gtest_dep,
                        ]))
endforeach

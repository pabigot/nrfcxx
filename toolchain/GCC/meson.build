# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

# See meson #2404
if device.startswith('NRF51')
  startup_s = files('startup_ARMCM0.S')
elif device.startswith('NRF52')
  startup_s = files('startup_ARMCM4.S')
else
  startup_s = []
endif

# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

message('Identifying soft device for ' + device)
if ('NRF51' == device)
  softdevice = 's130'
  softdevice_pfx = softdevice + '_nrf51_2.0.1_'
elif 'NRF52832' == device
  softdevice = 's132'
  softdevice_pfx = softdevice + '_nrf52_6.1.1_'
elif 'NRF52840' == device
  softdevice = 's140'
  softdevice_pfx = softdevice + '_nrf52_6.1.1_'
else
  softdevice = 'blank'
endif
if softdevice != 'blank'
  message('Using soft device ' + softdevice)
  softdevice_inc = include_directories(softdevice_pfx + 'API/include')
  softdevice_hex = files(softdevice_pfx + 'softdevice.hex')
else
  message('No soft device identified')
endif
  

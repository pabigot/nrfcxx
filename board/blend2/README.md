Board home page: https://github.com/redbear/nRF5x/tree/master/nRF52832

Device: nRF52832

openocd -f daplink.cfg -c 'nrf5 mass_erase'
openocd -f daplink.cfg -c 'program blend2/examples/bootstrap/cstdio.elf reset exit'

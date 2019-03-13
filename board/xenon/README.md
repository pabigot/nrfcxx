Board home page: https://docs.particle.io/xenon/

Device: nRF52840
External 32 Mb SPI/QSPI flash MX25L3233F
LEDS: P0.13 (red LED0), P0.14 (green LED1), P0.15 (blue LED2)
Buttons: P0.11 (mode), P0.18 (reset)
SDA: P0.26, SCL P0.27 # match Feather

Note that the MX25L3233F is placed into deep power-down mode by
board::initialize(), reducing idle current by about 9 uA (25%).

# NRFCXX

NRFCXX is a build environment and development framework for Nordic nRF5
Cortex<sup>&reg;</sup>-M microcontrollers using C++.

Characteristics of this framework include:
* Requires C++17 or later;
* Designed for ultra-low-power wireless sensor applications using
  Bluetooth beacon technology;
* Supports both nRF51 and nRF52 processors;
* Implemented for use with GNU Compiler;
* Uses the [nrfx][] mdk CMSIS headers with C++ templates for optimized
  control through peripheral registers without HAL or other intervening
  driver API;
* Provides an event-driven control model with state machine
  infrastructure for sensors;
* Leverages Nordic soft devices for Bluetooth 4.2 (nRF51) and 5.0
  (nRF52) compatibility;
* Full API documentation;
* Multiple examples;
* Support for multiple easily obtained nRF5-based development boards.

Characteristics the framework currently lacks:
* Support for Bluetooth peripheral and central roles;
* Support for OTA firmware update;
* Detailed high-level documentation and a roadmap through examples;
* Ability to build applications out-of-tree;
* Script support on non-POSIX systems.

Source code and issue management are at:
https://github.com/pabigot/nrfcxx

Documentation is at: https://pabigot.github.io/nrfcxx

## Build Area Setup

NRFCXX uses the [Meson build system][meson] for build management.  There
are subprojects, so start with:

    meson subprojects download

You also must download and patch the Nordic soft devices; see
instructions in the `softdevice` subdirectory.

Each
build area is configured for a specific target board, or by default for
the host to build documentation and run unit tests.  In-tree boards are
provided in the `board` subdirectory, and can be selected like this:

    scripts/mkbuild pca10056

which will configure the `pca10056` directory to build for the
nRF52840-DK.  The libraries and all example applications can then be
built with:

    ninja -C pca10056

Applications that do not require soft devices can be programmed onto the
board with the [nRF5 Command Line Tools][nrf5cli]:

    ELF=examples/bootstrap/devinfo ninja -C pca10056 program

## Host-based test suite

When the `--cross` option is not used to configure the build area a
subset of the framework is built for host-based unit testing.  You may
also set `board`.

Consider adding `-Db_coverage=true` when configuring a host build area.
This allows:

    ninja -C build test
    ninja -C build coverage-html

The coverage results will be in `build/meson-logs/coveragereport/`.

## Controlling Dev Boards

Several run targets are provided that can be accessed as ninja commands.
If there is only one JLink device on the system the board attached to
that device will be programmed.

If multiple JLink devices are present the serial number of the desired
device should be set in the `JSN` environment variable.  Otherwise the
device with the lowest serial number will be used.

At meson 0.45 presenting multiple commands works and they are executed
in the order specified in the meson.build script (not necessarily
command-line order).

Example:

    JSN=681982376 ninja erase
    ELF=examples/s130/sd-beacon ninja erase flash_softdevice program

### erase

This command invokes the `--eraseall` nrfjprog action.

### program

This command looks in the `ELF` environment variable for a path,
relative to the build root, to an elf file that should be programmed.
The `.elf` suffix present on the constructed images will be added if
necessary.  It then converts the image to Intel hex in a temporary file,
programs it with the nrfjprog `--program` action and `--sectorerase`,
then resets the board with the nrfjprog `--reset` action.

Example:

    ELF=examples/bootstrap/alarm ninja program

### flash_softdevice

This command flashes the soft device onto the board.  It neither erases
nor resets.

### reset

This command issues a hard reset to the board.

[meson]: https://mesonbuild.com/
[nrf5cli]: https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF5-Command-Line-Tools

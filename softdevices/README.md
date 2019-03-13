# Nordic SoftDevices Support under meson

To get a Nordic soft device you must go to
https://www.nordicsemi.com/Software-and-Tools/Software and scroll down
to the Bluetooth SoftDevices section.

For nRF51 you want version 2.0.1 from
https://www.nordicsemi.com/Software-and-Tools/Software/S130

For nRF52832 you want version 6.1.1 from
https://www.nordicsemi.com/Software-and-Tools/Software/S132

For nRF52840 you want version 6.1.1 from
https://www.nordicsemi.com/Software-and-Tools/Software/S140

For each download the zip file, then unpack it to get a second zip file
that has the soft device and API files.  The files you want are:

```
-rw-r--r-- 1 user user  311014 Mar  5 23:44 s130nrf51201.zip
-rw-r--r-- 1 user user  370870 Mar  5 23:48 s132nrf52611.zip
-rw-r--r-- 1 user user  389907 Mar  5 23:48 s140nrf52611.zip
```

Unpack each of those into this directory.

The s130 and s140 soft device requires a couple patches to work
correctly with C++ and the cleaned nrfx mdk used in nrfcxx.  Apply them
like this:

```
for f in PATCHES/*.patch ; do
  patch -p1 < ${f}
done
```

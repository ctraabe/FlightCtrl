University of Tokyo FlightCtrl
--

This is alternative firmware for the MikroKopter FlitghtCtrl board developed by Chris Raabe at the University of Tokyo.

Status
--

**Not yet plug and play**

**USE AT YOUR OWN RISK**

This version of the software currently employs constants that have been pre-computed for the weight and inertia of a small X-quad. These constants would have to be recomputed for different vehicles. In the future, all constants will be computed at startup given the following parameters: weight, inertia, propeller thrust constant, propeller drag constant, propeller locations, propeller rotations, and motor time constants.

To do
--

- Review and improve diagnostics and error handling
- Implement communication protocol to update configuration (in EEPROM)
- Compute constants (control gains, Kalman filter coefficients, angular dynamics model coefficients) based on vehicle measurements
- Receive attitude commands via SPI or UART
- Limit thrust based on voltage to prevent battery damage
- Implement altitude hold based on barometric pressure
- Implement disturbance (acceleration) rejection
- Implement bootloader

Intent
--

This firmware is intended to wholly replace the MikroKopter ecosystem. It may or may not be compatible with the MikroKopter tool in the future, but is not as of now.

The primary goal of the firmware is to provide a very good inner-loop controller for any configuration, up to 8 propellers.

The inner-lop controller takes commands for thrust, heading, and the direction of the gravity vector in the body axis and forms motor commands to achieve those commands.

Build and install
--

This software was built using Binutils 2.25 and GCC 5.2

The software is built using the standard `make`

Currently there are several ways to upload the resulting HEX file to the FlightCtrl board

1. Upload using the MikroKopter tool (may have to remove and restore power to the board after initiating upload)

2. Upload using MKProgrammer Linux tool, which can be found here: https://github.com/ctraabe/MKProgrammer  (may have to remove and restore power to the board after initiating upload)

3. Upload using an AVR programmer (requires erasing the chip, which will also erase the proprietary MK bootloader)

GCC for AVR can easily be built and installed on linux. The following are the steps that were used for Ubuntu 15.04:

##### Build and install binutils and avr-gcc

Download binutils and gcc from http://ftp.jaist.ac.jp/pub/GNU/

Get prerequisites (GMP and MPFR and MPC):

`sudo apt-get install texinfo libgmp3-dev libmpfr-dev libmpc-dev`

In `binutils-<version>` directory:

```
mkcd obj-avr
../configure --target=avr --disable-nls
make -j4
sudo make install
```

In `gcc-<version>` directory:

```
mkcd obj-avr
../configure --target=avr --enable-languages=c,c++ --disable-nls --disable-libssp --with-dwarf2
make -j4
sudo make install
```

##### Get AVR LIBC

Download avr-libc from http://savannah.nongnu.org/download/avr-libc/

**OR** use the following repository (for bleeding edge):

```
svn co http://svn.savannah.nongnu.org/svn/avr-libc/trunk
sudo apt-get install automake
cd avr-libc/trunk/avr-libc
./bootstrap
```

In `avr-libc-<version>` **OR** `trunk/avr-libc` directory:

```
./configure --build=`./config.guess` --host=avr
make -j4
sudo make install
```

Control design
--

The inner-loop controller has the following features:

- Quaternion-based
- Pole-placed state-feedback
- Kalman filter to smooth angular rate and acceleration measurements about the x and y body axes
- Model of angular dynamics to enable high-gain integral control without overshoot (ideally)

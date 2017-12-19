ttyebus
===================

ttyebus is a linux kernel module that will provide a serial device /dev/ttyebus with almost no latency upon receiving characters. It is dedicated to the PL011 UART of the Raspberry Pi.

Theory of Operation
-------------------
The PrimeCell UART PL011 found in the Broadcom BCM2835 on the Rasperry Pi is a variation of the popular 16C650 UART. This kind of UART features a 32 byte depth FIFO for receive and transmit. One can select the FIFO trigger level that will specify the number of received bytes in the FIFO before the UART will issue an interrupt to the system. The values for this trigger level are 4, 8, 16, 24 and 28 characters. Unfortunately, it cannot be set to 1 character.

This means that when FIFO is enabled, a minimum of 4 characters has to be received (or some timeout has to occur) until an interrupt is issued and the system can react. This is far too long for the eBus where the time between receiving a SYN character and sending an address byte is specified to be in the range of 100us at a baudrate of 2400 Baud.

This driver disables the FIFO completely and so an interrupt will be issued immediately after receiving any single character. To prevent from receiver overrun, a software FIFO (ring-buffer) is implemented and filled directly from the interrupt service routine.

Since the transmit FIFO is also disabled when disabling the receive FIFO, a software buffer for sending characters is also implemented. The user can put characters into that buffer and the characters are transferred to the transmitter by use of the interrupt service routine. 

Limitations
-----------
This driver was created to be used along with the eBus. Therefore, the UART parameters are fixed to 2400 Baud, 8 Bit, 1 Stop Bit, no parity. It is not guaranteed that it will work with any higher baudrate.

This driver will manipulate the PL011 UART hardware and the GPIO pins 14 and 15 directly. There is no check if this will collide with any other software in the system. The user has to take care that the driver has exclusive access to this hardware. Especially at the Raspberry Pi / Raspbian, the device /dev/ttyAMA0 has to be removed completely before using this driver.

Since this driver is provided as a kernel module, it has to be compiled at the target system, using the kernel header files of the actual kernel version. When upgrading the kernel to a higher version, the new kernel header files must also be fetched and the driver must be re-compiled with this matching headers.  

Install
--------
* Before using this software, the resources of the PL011 UART normally allocated by the ttyAMA0 device must be freed.
 - Type "cat /sys/firmware/devicetree/base/model" to see what kind of hardware you have.
 - On ***Rasperry Pi 3***, append a line "dtoverlay=pi3-miniuart-bt" to /boot/config.txt. This will exchange the UART and the Mini-UART so the Mini-UART is connected to the bluetooth and the UART to the GPIO pins.
    > sudo echo "dtoverlay=pi3-miniuart-bt" >> /boot/config.txt 
 - On ***all*** hardware, call "sudo raspi-config" - Interfacing Options - Serial - and disable the login shell and the serial port hardware. Press finish and the system should reboot.

  - You may verify this by typing "ls -l /dev". The "ttyAMA0" should no longer be listed.
 
* Update your raspbian linux to the latest version
    > sudo apt-get update  
    > sudo apt-get -y upgrade

* Download the kernel headers that fit to your kernel version:
    > sudo apt-get install raspberrypi-kernel-headers

    The header files should now recede in /usr/src/linux-headers-xxxxx. You may want to cross-check your kernel version by typing "uname -r".
* Download the latest ttyebus release package from the repository to your working directory.
    > cd ~  
    > git clone https://github.com/ebus/ttyebus.git

* Build the ttyebus module
    > cd ~/ttyebus  
    > make
    
    On success, you should find a file "ttyebus.ko" in your working directory.
* Install the ttyebus module. This includes copying the module file to its target directory, inserting the module at the kernel and registering the module for autostart at boot time.
    > sudo make install
* Reboot. Now the module should be automatically loaded and can be shown with
    > lsmod  
    > modinfo ttyebus

    and the device "ttyebus" should be listed with
    > ls -l /dev

Uninstall
---------
* If you want to uninstall the module you can do this with:

    > sudo make uninstall

* If uninstall fails because the module ttyebus is in use, you may consider stopping the user of the module first, namely the ebusd daemon, see the [ebusd Wiki](https://github.com/john30/ebusd/wiki/2.-Run):
    > sudo service ebusd stop

Configuration
-------------
To be used with the ebusd, the ebusd configuration must be adapted.  
In file
/etc/default/ebusd, change or add
> -d /dev/ttyebus

at the EBUSD_OPTS statement. For more details, see the [ebusd Wiki](https://github.com/john30/ebusd/wiki/2.-Run).

Warning
-------
Do not connect the eBus (via an eBus Adapter) to the Rasperry Pi GPIO pins unless you have deallocated the Raspbian console (ttyAMA0) from the GPIO pins, as mentioned above. If you connect to the eBus with the console active, you may not only freeze or crash the Raspbian system, but you may also influence the eBus with unexpected results.

Disclaimer
----------
This software is distributed in the hope that it will be useful, but without any warranty. You use the software at your own risk and the author is not responsible for any failure, malfunction or damage of any parts of the system. 

Contact
-------
For bugs and missing features use github issue system.

The author can be contacted at galileo53@gmx.at .


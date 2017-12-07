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

Installation
------------
* Before using this software, the resources of the PL011 UART normally allocated by the ttyAMA0 device must be freed. This can be done by
 - calling "sudo raspi-config" and disabling the serial interface
 - appending a line "dtoverlay=pi3-miniuart-bt" to /boot/config.txt (RASPI 3 only)
 - sudo systemctl stop serial-getty@ttyAMA0.service  
 sudo systemctl disable serial-getty@ttyAMA0.service  

--> Todo: Not clear if one or all of the above is necessary. Should verify this for different RASPIs.  

* You may verify this by typing "ls -l /dev". The "ttyAMA0" should no longer be listed.
 
* Create a working directory on your RasPi:
    > mkdir ~/ttyebus  
    > cd ~/ttyebus
* Pick the [latest release package](https://github.com/ebus/ttyebus/releases/latest) from the repository and move it to your working directory. The package will consist just of a Makefile and the ttyebus.c.

* Download the kernel headers that fit to your kernel version:
    > sudo apt-get install raspberrypi-kernel-headers

    The header files should now recede in /usr/src/linux-headers-xxxxx. You may want to cross-check your kernel version by typing "uname -r".
* Build the ttyebus
    > make
    
    On success, you should find a file "ttyebus_module.ko" in your working directory.
* Copy the ttyebus module to its target directory
    > sudo mkdir /lib/modules/$(uname -r)/kernel/drivers/ttyebus  
    > sudo cp ttyebus_module.ko /lib/modules/$(uname -r)/kernel/drivers/ttyebus
* Insert the module to the kernel
    > sudo insmod /lib/modules/$(uname -r)/kernel/drivers/ttyebus/ttyebus_module.ko
* Create the list of dependencies
    > sudo depmod -a
* Load the module
    > sudo modprobe ttyebus_module
* For autostart of the module, append a line "ttyebus_module" into "/etc/modules". -i will update the file, $ is regex to match end-of-file, a appends the following text
    > sudo sed -i '$a ttyebus_module' /etc/modules
* Reboot. Now the module should be loaded and can be shown with
    > lsmod  
    > modinfo ttyebus_module

    and the device "ttyebus" should be listed with
    > ls -l /dev


Documentation
-------------



Configuration
-------------
To be used with the ebusd, the ebusd configuration must be adapted.  
In file
/etc/default/ebusd, change or add "-d ttyebusd" at the EBUSD_OPTS statement. For more details, see the [ebusd Wiki](https://github.com/john30/ebusd/wiki/2.-Run).


Contact
-------
For bugs and missing features use github issue system.

The author can be contacted at galileo53@gmx.at .

#!/bin/bash
###################################################################
#
# make file to build and install/uninstall the ttyebus module
# 
#
###################################################################
TARGET_MODULE:=ttyebus
TARGET_DIR:=/lib/modules/$(shell uname -r)/kernel/drivers/tty/serial

# If we running by kernel building system
ifneq ($(KERNELRELEASE),)
	$(TARGET_MODULE)-objs := $(TARGET_MODULE)m.o
	obj-m := $(TARGET_MODULE).o

# If we are running without kernel build system
else
	BUILDSYSTEM_DIR?=/lib/modules/$(shell uname -r)/build
	PWD:=$(shell pwd)


all : 
# run kernel build system to make module
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules

clean:
# run kernel build system to cleanup in current directory
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) clean

load:
	modprobe $(TARGET_DIR)/$(TARGET_MODULE)

unload:
	modprobe -r $(TARGET_MODULE)
	
install:
	cp $(TARGET_MODULE).ko $(TARGET_DIR)/$(TARGET_MODULE).ko
	depmod -a
	cd $(TARGET_DIR)/
	modprobe $(TARGET_MODULE)
	sed -i "s/$(TARGET_MODULE)//g" /etc/modules
	echo "$(TARGET_MODULE)" >> /etc/modules
	
uninstall:
	modprobe -r $(TARGET_MODULE)
	rm $(TARGET_DIR)/$(TARGET_MODULE).ko
	sed -i "s/$(TARGET_MODULE)//g" /etc/modules

endif

TARGET_MODULE:=ttyebus_module

# If we running by kernel building system
ifneq ($(KERNELRELEASE),)
	$(TARGET_MODULE)-objs := ttyebus.o
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
	insmod ./$(TARGET_MODULE).ko

unload:
	rmmod ./$(TARGET_MODULE).ko
	
install:
	cp ttyebus_module.ko /lib/modules/$(shell uname -r)/kernel/drivers/tty/serial/ttyebus.ko
	# -i will update the file, $ is regex to match end-of-file, a appends the following text
	sed -i '$a ttyebus_module' /etc/modules
	
uninstall:
	rm /lib/modules/$(shell uname -r)/kernel/drivers/tty/serial/ttyebus.ko
	sed -i 's/ttyebus//g' /etc/modules

endif

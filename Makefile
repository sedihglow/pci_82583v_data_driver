KERNEL_DIR = /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

TARGET = pciLED
pciLED-objs := pciDev_main.o pciDev.o ledTimer.o

obj-m += $(TARGET).o

all:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean

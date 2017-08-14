obj-m += usbdisplay.o 
usbdisplay-objs := src/usbdisplay.o

ccflags-y := -std=gnu99

KVERSION ?= $(shell uname -r)
KERNEL_SOURCE ?= /lib/modules/$(KVERSION)/build

all:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD) clean

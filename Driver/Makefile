# kbuild obj
obj-m := dma-proxy.o

# Try the paths Yocto sets; fall back to running-kernel headers
KERNEL_SRC ?= $(KERNEL_PATH)
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# NOTE: the lines after each target MUST start with a real TAB
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean


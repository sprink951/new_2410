ifeq ($(KERNELRELEASE),)
KERN_DIR=/home/liu/old_2410/linux-2.6.35_farsight
PWD=$(shell pwd)
modules:
	$(MAKE) -C $(KERN_DIR) M=$(PWD) modules
clean:
	rm *.o *.ko *.mod.c mo* Mo* -rf
else
	obj-m=cs8900.o
endif


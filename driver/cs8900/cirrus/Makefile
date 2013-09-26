ifeq ($(KERNELRELEASE),)
KERN_DIR=/home/liu/linux-3.10.12
PWD=$(shell pwd)
modules:
	$(MAKE) -C $(KERN_DIR) M=$(PWD) modules
clean:
	rm -rf *.o *.ko Modu* modu* *.mod.c
else
	obj-m=cs89x0.o
endif

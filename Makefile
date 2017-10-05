XENO_CONFIG=/usr/xenomai/bin/xeno-config

XENOMAI_SKIN=native
prefix := $(shell $(XENO_CONFIG) --prefix)
ifeq ($(prefix),)
$(error Please add <xenomai-install-path>/bin to your PATH variable)
endif
CC=gcc -no-pie -fno-pie
PWD:= $(shell pwd)
KDIR := /lib/modules/$(shell uname -r)/build

STD_CFLAGS  := $(shell $(XENO_CONFIG) --skin=$(XENOMAI_SKIN) --skin=rtdm --cflags) -I. -g -DXENOMAI_SKIN_$(XENOMAI_SKIN)
STD_LDFLAGS := $(shell $(XENO_CONFIG) --skin=$(XENOMAI_SKIN) --skin=rtdm --ldflags) -g 
EXTRA_CFLAGS += $(shell $(XENO_CONFIG) --skin=$(XENOMAI_SKIN) --skin=rtdm --cflags)
EXTRA_CFLAGS += $(CFLAGS) 

MODULE_NAME=rtdm_pruss_irq
TEST=$(MODULE_NAME)-test
obj-m := $(MODULE_NAME).o 

all: $(MODULE_NAME).ko $(TEST)
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules Q=

$(MODULE_NAME).ko: $(MODULE_NAME).c

test: $(TEST)
$(TEST): $(TEST).c $(TEST)_bin.h
	$(CC) -o $@ $< $(STD_CFLAGS) $(STD_LDFLAGS) -I/root/Bela/include /root/Bela/lib/libprussdrv.a

install:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules_install
	depmod -a

clean:
	rm -f *~ Module.markers Module.symvers modules.order
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

m: all
	rmmod $(MODULE_NAME) 2> /dev/null || true
	insmod ./$(MODULE_NAME).ko

$(TEST)_bin.h: $(TEST).p
	pasm -V2 $(TEST).p > /dev/null

t: test
	./$(TEST)

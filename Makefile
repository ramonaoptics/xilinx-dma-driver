all:
	make -C xdma
	make -C tools

install:
	make -C xdma install
	depmod
	modprobe xdma

.PHONY: all install

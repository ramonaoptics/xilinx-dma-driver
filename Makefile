all:
	make -C driver
	make -C tests

install: uninstall
	install -d /usr/include/xdma
	install -m 644 include/*.h /usr/include/xdma/.
	insmod driver/xdma.ko

uninstall:
	- rmmod xdma
	rm -rf /usr/include/xdma

.PHONY: all, install, uninstall


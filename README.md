# AR 65444 Xilinx PCI Express DMA Drivers and Software Guide

This is mostly a dump of [AR 65444](https://www.xilinx.com/support/answers/65444.html) as a github repo to track my changes


# Changelog

The tag `rel20180420` basically includes a straight dump of Xilinx's files

1. You will need to copy the files in the `etc` directory from their "old deprecated" linux install.
   * I've done this here in the master branch

2. Their install instructions miss a few key components for people that don't understand kernels very well:
   a. You will after running the `sudo make install` command from the `xdma` directory, you will
      see an output message like:

```
  INSTALL /home/mark2/git/xilinx/xilinx-dma-driver/xdma/xdma.ko
At main.c:160:
- SSL error:02001002:system library:fopen:No such file or directory: ../crypto/bio/bss_file.c:74
- SSL error:2006D080:BIO routines:BIO_new_file:no such file: ../crypto/bio/bss_file.c:81
sign-file: certs/signing_key.pem: No such file or directory
  DEPMOD  4.15.0-45-generic
```

     This is OK, nothing to worry about. I think it is because it is an unsigned due to the fact
     that you likely lack a signing key at this point in the development.

  b. You might need to issue the command `sudo depmod` after installing the driver. Technically,
     it is spwed out during the `make install` command, but I've found that my kernel wasn't about to find the driver
  c. You should then install the kernel module with `sudo modprobe xdma`. To check that it is
     installed, you can use the `dmesg | grep xmod`. I have an output that looks like:

```
[  958.391237] xdma:xdma_mod_init: Xilinx XDMA Reference Driver xdma v2017.1.47
[  958.391242] xdma:xdma_mod_init: desc_blen_max: 0xfffffff/268435455, sgdma_timeout: 10 sec.
```

3. Unfortunately, the following commands will require you to run
```
modprobe xdma
```

   on every boot. Kinda annoying.
   The instructions on the manpage `modules-load.d` are kinda wrong.
   You need to add the line `xdma` t o the file `/etc/modules`. My file now looks like:

```
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.

fpga
xdma
```

You can check that things loaded correctly by issuing the command

```
<F28>mark2@xps ~
$ systemctl status systemd-modules-load.service
● systemd-modules-load.service - Load Kernel Modules
   Loaded: loaded (/lib/systemd/system/systemd-modules-load.service; static; ven
   Active: active (exited) since Thu 2019-02-07 10:29:49 EST; 9min ago
     Docs: man:systemd-modules-load.service(8)
           man:modules-load.d(5)
  Process: 287 ExecStart=/lib/systemd/systemd-modules-load (code=exited, status=
 Main PID: 287 (code=exited, status=0/SUCCESS)

Warning: Journal has been rotated since unit was started. Log output is incomple

mark2@xps ~
$ journalctl -b _PID=287
-- Logs begin at Mon 2018-10-22 23:39:25 EDT, end at Thu 2019-02-07 10:38:58 EST
Feb 07 10:29:49 xps systemd-modules-load[287]: Inserted module 'lp'
Feb 07 10:29:49 xps systemd-modules-load[287]: Inserted module 'ppdev'
Feb 07 10:29:49 xps systemd-modules-load[287]: Inserted module 'parport_pc'
Feb 07 10:29:49 xps systemd-modules-load[287]: Inserted module 'fpga'
Feb 07 10:29:49 xps systemd-modules-load[287]: Inserted module 'xdma'
```

Notice that the PID of `journalctl` command matches the output of the `systemctl` command.

The command `lsmod | grep xdma` should also indicate that the module was loaded correctly.

## TODO

1. Make the kernel module load automatically depending on a rule from `/etc/udev/rules.d`
2. Remove that `xdma-udev-command.sh` script

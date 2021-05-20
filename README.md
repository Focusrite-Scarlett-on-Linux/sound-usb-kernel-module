## Development

To actually try modifying the driver source code, and build your modification against your current running kernel, do this (don't worry - the outcome stays
in this directory):

```
make -C /lib/modules/`uname -r`/build M=`pwd`/sound/usb clean
make -C /lib/modules/`uname -r`/build M=`pwd`/sound/usb modules
```

*Untested* Something like this would install the newly built modules in your system against your current kernel.
You might want to do `make -n` to see where the command put the modules:

```
make -C /lib/modules/`uname -r`/build M=`pwd`/sound/usb modules_install
```

The linux kernel prefers to load modules under `/lib/modules/*/updates` and `/lib/modules/*/extra` over elsewhere under `/lib/modules/*`,
so it is safer to manually copy new `*.ko` into those
directories, and run `depmod -a` to let the kernel rescan them, rather than doing `make ... modules_install`.

If your kernel has `CONFIG_MMODULE_SIG_FORCE=y` (most secure-boot distribution kernels are), you will need to have a pair of `certs/signing_key.pem`
and `certs/signing_key.x509`, acceptable to your current BIOS, to sign the kernel modules.


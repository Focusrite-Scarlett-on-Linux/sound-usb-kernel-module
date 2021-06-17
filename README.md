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

The default branch (`v5.12-dev`) tracks Vladimir / sadko4u 's. The `v5.12-scarlett-gen3` branch tracks Geoffrey
Bennett's `scarlett-gen3` development, and can be accessed by:

```
git clone -b v5.12-scarlett-gen3 https://github.com/Focusrite-Scarlett-on-Linux/sound-usb-kernel-module.git
cd sound-usb-kernel-module
make -C /lib/modules/`uname -r`/build M=`pwd`/sound/usb clean
make -C /lib/modules/`uname -r`/build M=`pwd`/sound/usb modules
# copy all the *.ko from under sound/usb into /lib/modules/`uname -r`/updates ; see the README
depmod -a
```

You should see `Focusrite Scarlett Gen 2/3 Mixer Driver enabled [sadko4u mod] ...` for one,
and `Focusrite Scarlett Gen 2/3 Mixer Driver enabled v5.12.9s1 ...`
for the other.

If your kernel has `CONFIG_MMODULE_SIG_FORCE=y` (most secure-boot distribution kernels are), you will need to have a pair of `certs/signing_key.pem`
and `certs/signing_key.x509`, acceptable to your current BIOS, to sign the kernel modules.

Add a single line `#define DEBUG 1` to the top of `sound/usb/mixer_scarlett_gen2.c`, to enable some rather noisy debug messaging code, if needed.

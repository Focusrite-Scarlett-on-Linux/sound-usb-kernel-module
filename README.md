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

`depmod -v -a` (with `-v`) gives rather verbose messages about kernel modules. You should find those new ones being
processed.

Also remember to create a `/etc/modprobe.d/scarlett.conf` with `device_setup=1` option for the device as in the
[detailed instruction](https://github.com/geoffreybennett/scarlett-gen2/releases), duplicated further below.

After reboot, if you run `dmesg`,
you should see `Focusrite Scarlett Gen 2/3 Mixer Driver enabled [sadko4u mod] ...` for one,
and `Focusrite Scarlett Gen 2/3 Mixer Driver enabled v5.12.9s1 ...`
for the other.

If your kernel has `CONFIG_MMODULE_SIG_FORCE=y` (most secure-boot distribution kernels are), you will need to have a pair of `certs/signing_key.pem`
and `certs/signing_key.x509`, acceptable to your current BIOS, to sign the kernel modules.

## Enabling new functionality at load time

You need to enable the driver at module load time with the `device_setup=1` option to insmod/modprobe. Create a new file `/etc/modprobe.d/scarlett.conf`, with
the following one-line content:

Gen 2:

- 6i6: `options snd_usb_audio vid=0x1235 pid=0x8203 device_setup=1`
- 18i8: `options snd_usb_audio vid=0x1235 pid=0x8204 device_setup=1`
- 18i20: `options snd_usb_audio vid=0x1235 pid=0x8201 device_setup=1`

Gen 3:

- Solo: `options snd_usb_audio vid=0x1235 pid=0x8211 device_setup=1`
- 2i2: `options snd_usb_audio vid=0x1235 pid=0x8210 device_setup=1`
- 4i4: `options snd_usb_audio vid=0x1235 pid=0x8212 device_setup=1`
- 8i6: `options snd_usb_audio vid=0x1235 pid=0x8213 device_setup=1`
- 18i8: `options snd_usb_audio vid=0x1235 pid=0x8214 device_setup=1`
- 18i20: `options snd_usb_audio vid=0x1235 pid=0x8215 device_setup=1`

Or you can use the sledgehammer: `options snd_usb_audio device_setup=1,1,1,1` to pass that option to the first 4 USB audio devices.

# DKMS modules for sound/usb from the latest Linux Kernel development

Initially, this enables the usage of bleeding-edge driver code for USB sound
devices against a mainline kernel (5.2.x). By "bleeding-edge", I mean pre-Linus, destined for the 5.4 merge window,
a few months in the future.

This is assembled from the Linux Kernel's sound sub-system maintainer, 
Takashi Iwai's tree (`git://git.kernel.org/pub/scm/linux/kernel/git/tiwai/sound.git`). Initially, it was mainly for trying out the 
work of Geoffrey D. Bennett (`git://github.com/geoffreybennett/scarlett-gen2.git`) regarding the Focusrite Scarlett 18i20 Gen 2.
However, it is likely useful for other USB sound devices too.

## Installation

For those who merely wants to try it out, from [mainline 5.2.8](https://kernel.ubuntu.com/~kernel-ppa/mainline/v5.2.8/),
[mainline 5.2.9](https://kernel.ubuntu.com/~kernel-ppa/mainline/v5.2.9/) , grab at least the `image-unsigned` and 
`modules` Ubuntu packages, plus the corresponding binary packages in release. e.g.

```
dpkg -i linux-image-unsigned-5.2.8-050208-generic_5.2.8-050208.201908091630_amd64.deb
dpkg -i linux-modules-5.2.8-050208-generic_5.2.8-050208.201908091630_amd64.deb
dpkg -i sound-usb-modules-5.2.8-050208-generic_5.4.0rc0_amd64.deb
```

For those who actually may like to modify the driver code, grab the `headers-…-all` and `headers-…-amd64` packages, too, of
one of the [mainline 5.2.x series](https://kernel.ubuntu.com/~kernel-ppa/mainline/). You also need to install the `dkms` package
from Ubuntu, and the `sound-usb-dkms` package from release :
  
```
apt install dkms
dpkg -i sound-usb-dkms_5.4.0rc0-1ubuntu1_all.deb
```

This combination will try to build the USB sound modules, from source code, for every compatible kernel image you
have installed.

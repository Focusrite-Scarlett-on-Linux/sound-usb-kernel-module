# DKMS modules for sound/usb from the latest Linux Kernel development

Initially, this enables the usage of bleeding-edge driver code for USB sound
devices against a mainline kernel (5.2.x). By "bleeding-edge", I mean pre-Linus, destined for the 5.4 merge window,
a few months in the future.

This is assembled from the Linux Kernel's sound sub-system maintainer, 
Takashi Iwai's tree (`git://git.kernel.org/pub/scm/linux/kernel/git/tiwai/sound.git`). Initially, it was mainly for trying out the 
work of Geoffrey D. Bennett (`git://github.com/geoffreybennett/scarlett-gen2.git`) regarding the Focusrite Scarlett 18i20 Gen 2.
However, it is likely useful for other USB sound devices too.

## Installation

#### Ubuntu

For those who merely wants to try it out, from [mainline 5.2.8](https://kernel.ubuntu.com/~kernel-ppa/mainline/v5.2.8/),
[mainline 5.2.9](https://kernel.ubuntu.com/~kernel-ppa/mainline/v5.2.9/),
[mainline 5.2.11](https://kernel.ubuntu.com/~kernel-ppa/mainline/v5.2.11/), grab at least the `image-unsigned` and
`modules` Ubuntu packages, plus the corresponding binary packages in release. e.g.

```
dpkg -i linux-image-unsigned-5.2.8-050208-generic_5.2.8-050208.201908091630_amd64.deb
dpkg -i linux-modules-5.2.8-050208-generic_5.2.8-050208.201908091630_amd64.deb
dpkg -i sound-usb-modules-5.2.8-050208-generic_5.3.0rc7.14312_amd64.deb
```

For those who actually may like to modify the driver code, grab the `headers-…-all` and `headers-…-amd64` packages, too, of
one of the [mainline 5.2.x series](https://kernel.ubuntu.com/~kernel-ppa/mainline/). You also need to install the `dkms` package
from Ubuntu, and the `sound-usb-dkms` package from release :
  
```
apt install dkms
dpkg -i sound-usb-dkms_5.3.0rc7.14312-1ubuntu1_all.deb
```

This combination will try to build the USB sound modules, from source code, for every compatible kernel image you
have installed.

#### Fedora

```
dnf install -y dkms
dnf install -y sound-usb-5.3.0rc7.14312-1dkms.noarch.rpm
```

### Manual installation on Non-Ubuntu/Debian Systems

For Non-Ubuntu/Debian Systems,

* You could just copy the whole of the `sound-usb` sub-directory to `/usr/src/sound-usb-<module-version>/` e.g. `<module-version>` = `5.3.0rc7.14312` in the above
is auto-generated by Ubuntu's `dpkg-buildpackage`.

* Then modify `debian/sound-usb-dkms.dkms` slightly by replacing `#MODULE_VERSION#` near the top by the same number
(`5.3.0rc7.14312`), and copy to `/usr/src/sound-usb-<module-version>/dkms.conf`.

* execute `dkms add sound-usb/5.3.0rc7.14312` to register the module.

Then most dkms commands should work, such as:

```
dkms status
dkms build --verbose sound-usb/5.3.0rc7.14312
dkms install --verbose sound-usb/5.3.0rc7.14312
```

The above build and install against your running kernel. Using `modprobe -r ...` to unload the old modules, and `modprobe -v ...` to reload. Reboot is probably simpler.

#### Manual Fedora Installation example

As an example, the `noarch.rpm` was built in a Fedora 30 docker instance (`docker pull fedora:30`) with:

```
dnf install dkms kernel-devel kernel-modules make rpm-build
cd ... # where
cp -ipr sound-usb /usr/src/sound-usb-5.3.0rc7.14312

# copy debian/sound-usb-dkms.dkms to dkms.conf and s/#MODULE_VERSION#/5.3.0rc7.14312/
cp -p dkms.conf /usr/src/sound-usb-5.3.0rc7.14312

dkms add sound-usb/5.3.0rc7.14312
dkms status

dkms build --verbose sound-usb/5.3.0rc7.14312 -k 5.2.9-200.fc30.x86_64
dkms mkrpm --verbose --source-only sound-usb/5.3.0rc7.14312 -k 5.2.9-200.fc30.x86_64
```

# kbd-backlight

This module is for Intel-T2-Macs running Linux, specifically Arch-Linux and if by default Keyboard Backlight is not working.

## Features

<!-- vim-markdown-toc GFM -->

* [Handling brightness with Touchbar controls](#changing-brightness)
* [Changing defaults with kernel settings](#changing-defaults)
* [Recovering from any sleep operation](#recovering-from-sleep)

<!-- vim-markdown-toc -->

### Changing brightness

The brightness levels can be changed by using the already available brightness touch controls on the Touchbar. The module will just map those controls to the systems default brightness-up and brightness-down key presses.

Brightness can be changed by holding those toggles or by just pressing them.

When brightness level is changed, it will increase or decrease in steps over time.

When being inactive for 1 Minute, the brightness level will be decreased to 50% from the current level. After waiting another 15 seconds, the brightness level will fall to 0. This resembles the timing, that can also be seen for the Touchbar backlight module.

If the module is loaded for the first time at boot or unloaded on shutdown, it will again try to step between the brightness values. On boot it will go to the default value and on shutdown it will try to go to zero.

### Changing defaults

This module will create files whilst being active, that allow for custom configuration.

The files should be located at /sys/kbd-backlight/.

Some of the available configuration options haven’t been tested yet, so there still might be some bugs, or they just won’t work.

Currently available configurations:
* Current brightness
* Minimum brightness
* Maximum brightness
* Default brightness
* Brightness change delay
* Brightness change steps
* Current brightness
* First keyboard dimming delay
* Second keyboard dimming delay

### Recovering from sleep

When recovering from any sleep operation, this module will use its default values to restore the brightness.

## Requirements

* gcc
* make

## Installation

To install this module, you may choose between different ways, but here’s the one I used (You can get the .ko file by building from source):

```sh
# === CONFIG ===

MODULE_NAME=“kbd-backlight”
RAMFS_CONF=“/etc/mkinitcpio.conf”

# === INSTALL ===

# Create directory for the module
mkdir /lib/modules/$(uname -r)/extra

# Move module to the kernel
mv kbd-backlight.ko /lib/modules/$(uname -r)/extra/kbd-backlight.ko

# Install the module
insmod /lib/modules/$(uname -r)/extra/kbd-backlight.ko

# Configure the module to load on boot
if ! grep -qE "^MODULES=.*\b$MODULE_NAME\b" "$HOOK_CONF"; then
  sed -i "s/^MODULES=(/MODULES=($MODULE_NAME /" "$HOOK_CONF"
fi

# Rebuild initramfs
mkinitcpio -P
```

## Building

Building should be relatively simple, just run:

```sh
make
```

## Attention

Be aware that this Readme might be incomplete or inaccurate. I'll try to keep it updated whenever I've got time.

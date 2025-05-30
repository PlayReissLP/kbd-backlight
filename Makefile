PROJECT_NAME := HID-KBD-BACKLIGHT

obj-m += hid-kbd-backlight.o

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

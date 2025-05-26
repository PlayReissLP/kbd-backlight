#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/workqueue.h>

#include "hid-kbd-brightness.h"

/* Product Definitions */
#define PRODUCT_ID_TB 0x8302 // Apple Inc. Touch Bar Display
/* Check Definitions - Brightness */
#define CHECK_BRIGHTNESS_LED 0x1
#define CHECK_BRIGHTNESS_LED_FUNC (CHECK_BRIGHTNESS_LED << 1)
#define CHECK_OK (CHECK_BRIGHTNESS_LED | CHECK_BRIGHTNESS_LED_FUNC)
/* Check Definitions - Key State */
#define CHECK_KEY_STUCK (1 << 4)

/* Macros */
#define FLAGS_LOWER_NIBBLE(flags) ((flags) & 0xF)
#define FLAGS_UPPER_NIBBLE(flags) (((flags) >> 4) & 0xF)

/* Externs */
extern struct list_head leds_list;

/* Declarations */
enum counter_type_e counter_type_e;
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value);
static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id);
static void kbd_disconnect(struct input_handle* handle);
static int __init kbd_init(void);
static void __exit kbd_exit(void);
static void brightness_init(void);
static void send_brightness_work_func(struct work_struct* work);
static void brightness_update(void);
static void brightness_counter(const enum counter_type_e type);
static void check_brightness_steps(void);
static struct led_classdev* find_led_by_name(const char* led_name);
static void toggle_hold_work_func(struct work_struct* work);
static void brightness_unload(void);

/* Enums */
enum key_state_e {
    EV_KEY_UP,
    EV_KEY_DOWN,
    EV_KEY_LONG
};

enum counter_type_e {
    COUNTER_INC,
    COUNTER_DEC
};

enum key_type_e {
    KBDILLUMUP = 1,
    KBDILLUMDOWN = 2
};

/* Structs */
static const struct input_device_id kbd_ids[] = {
    {
        .flags = INPUT_DEVICE_ID_MATCH_PRODUCT,
        .product = PRODUCT_ID_TB
    },
    { },
};

static struct input_handler kbd_handler = {
    .event = kbd_event,
    .connect = kbd_connect,
    .disconnect = kbd_disconnect,
    .name = "hid-kbd-brightness",
    .id_table = kbd_ids
};

static struct {
    struct led_classdev* led;
    unsigned char brightness;
    unsigned char max;
    unsigned char check_flags;
} brightness = {
    .brightness = 50, // Default brightness value
    .max = 100, // Max default brightness
    .check_flags = 0 // Default check is false
};

static struct work_struct send_brightness_work;

static struct delayed_work toggle_hold_work;

/* Variables */
static const unsigned char BRIGHTNESS_STEPS = 5;

/* Event Functions */
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value) {
    // pr_info("[hid-kbd-brightness]: Event key: %d", value);

    if (value == EV_KEY_DOWN
        || value == EV_KEY_LONG) {
        // pr_info("[hid-kbd-brightness]: Pressing .. value: %d\n", value);

        if(value == EV_KEY_DOWN) {
            switch(code) {
                case KEY_KBDILLUMUP:
                    pr_info("[hid-kbd-brightness]: Brightness up ..\n");
                    
                    brightness_counter(COUNTER_INC);
    
                    brightness_update();
                    return;
                case KEY_KBDILLUMDOWN:
                    pr_info("[hid-kbd-brightness]: Brightness down ..\n");
    
                    brightness_counter(COUNTER_DEC);
    
                    brightness_update();
                    return;
            }

            brightness.check_flags |= CHECK_KEY_STUCK;
        } else if(brightness.check_flags & CHECK_KEY_STUCK) {
            pr_info("[hid-kbd-brightness]: Key down ..");

            switch(code) {
                case KEY_KBDILLUMUP:
                    pr_info("[hid-kbd-brightness]: Setting illum up ..");

                    brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMUP);
                    break;
                case KEY_KBDILLUMDOWN:
                    pr_info("[hid-kbd-brightness]: Setting illum down ..");
                    
                    brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMDOWN);
                    break;
            }

            pr_info("[hid-kbd-brightness]: Brightness_check_flags: %d", brightness.check_flags);

            schedule_delayed_work(&toggle_hold_work, msecs_to_jiffies(100));
        }
    } else if(value == EV_KEY_UP) {
        if(brightness.check_flags & CHECK_KEY_STUCK) {
            pr_info("[hid-kbd-brightness]: State before release: %d", brightness.check_flags);

            pr_info("[hid-kbd-brightness]: State val: %d", CHECK_KEY_STUCK
                | (CHECK_KEY_STUCK << KBDILLUMUP)
                | (CHECK_KEY_STUCK << KBDILLUMDOWN)
            );
            
            // Resetting check key flags
            brightness.check_flags &= ~(
                CHECK_KEY_STUCK
                    | (CHECK_KEY_STUCK << KBDILLUMUP)
                    | (CHECK_KEY_STUCK << KBDILLUMDOWN)
            );

            pr_info("[hid-kbd-brightness]: State after release: %d", brightness.check_flags);
        }

        pr_info("[hid-kbd-brightness]: Released key ..");

        cancel_delayed_work(&toggle_hold_work);
    }
}

static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id) {
    struct input_handle* handle;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle) {
        return -ENOMEM;
    }

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "hid-kbd-brightness";

    if (input_register_handle(handle)) {
        kfree(handle);
        return -1;
    }

    if (input_open_device(handle)) {
        pr_err("[hid-kbd-brightness]: Failed to open device: %s\n", dev_name(&dev->dev));
        input_unregister_handle(handle);
        kfree(handle);
        return -1;
    }

    pr_info("[hid-kbd-brightness]: Connected to device: %s with id: %lx\n", dev_name(&dev->dev), id->evbit[0]);
    return 0;
}

static void kbd_disconnect(struct input_handle* handle) {
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
    pr_info("[hid-kbd-brightness]: Disconnected.\n");
}

static int __init kbd_init(void) {
    // Init workers
    INIT_WORK(&send_brightness_work, send_brightness_work_func);
    INIT_DELAYED_WORK(&toggle_hold_work, toggle_hold_work_func);

    // Init brightness stuff and load default value
    brightness_init();
    brightness_update();

    return input_register_handler(&kbd_handler);
}

static void __exit kbd_exit(void) {
    brightness_unload(); // Turn off backlight when unloading module
    input_unregister_handler(&kbd_handler);
}

/* Functions */
static void brightness_init(void) {
    if(!brightness.led) {
        brightness.led = find_led_by_name("apple::kbd_backlight");
    }

    if(brightness.led) {
        brightness.check_flags |= CHECK_BRIGHTNESS_LED;

        if(brightness.led->brightness_set_blocking) {
            brightness.check_flags |= CHECK_BRIGHTNESS_LED_FUNC;
        }
    }
}

static void send_brightness_work_func(struct work_struct* work) {
    brightness.led->brightness_set_blocking(brightness.led, (enum led_brightness)brightness.brightness);
}

static void brightness_update(void) {
    // pr_info("[hid-kbd-brightness]: Current checking flags value: %d", brightness.check_flags);
    // pr_info("[hid-kbd-brightness]: LED brightness: %d", (unsigned char)(brightness.led->brightness & 0xFF));
    // pr_info("[hid-kbd-brightness]: Current brightness value: %d", brightness.brightness);

    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        unsigned char current_brightness;

        pr_info("[hid-kbd-brightness]: Getting current brightness.. ");

        // Getting current brightness
        current_brightness = (unsigned char)(brightness.led->brightness & 0xFF);

        pr_info("[hid-kbd-brightness]: Got the following brightness: %d", current_brightness);

        // Checking if brightness has changed
        if(brightness.brightness != current_brightness) {
            pr_info("[hid-kbd-brightness]: Setting brightness to: %d", brightness.brightness);

            // Setting the current brightness value (This notifies the device)
            schedule_work(&send_brightness_work);
        }
    }
}

static void brightness_counter(const enum counter_type_e type) {
    check_brightness_steps();

    pr_info("[hid-kbd-brightness]: Type: %d | Brightness: %d | Brightness_max: %d", type, brightness.brightness, brightness.max);

    switch(type) {
        case COUNTER_INC:
            if(brightness.brightness < brightness.max) {
                brightness.brightness += 5;
            }
            break;
        case COUNTER_DEC:
            if(brightness.brightness >= BRIGHTNESS_STEPS) {
                brightness.brightness -= 5;
            }
            break;
    }
}

static void check_brightness_steps(void) {
    brightness.brightness = (unsigned char)(brightness.brightness / BRIGHTNESS_STEPS) * BRIGHTNESS_STEPS;
    pr_info("[hid-kbd-brightness]: Calculation result: %d", brightness.brightness);
}

static struct led_classdev* find_led_by_name(const char* led_name) {
    struct led_classdev* led;

    list_for_each_entry(led, &leds_list, node) {
        if(strcmp(led->name, led_name) == 0) {
            return led;
        }
    }

    return NULL;
}

static void toggle_hold_work_func(struct work_struct* work) {
    unsigned char key_state = FLAGS_UPPER_NIBBLE(brightness.check_flags) >> 1; // Three key state flags (upper most bits)

    pr_info("[hid-kbd-brightness]: Key state value: %d", key_state);

    if(key_state & KBDILLUMUP) {
        pr_info("[hid-kbd-brightness]: Holding up ..");

        brightness_counter(COUNTER_INC);
        brightness_update();
    } else if(key_state & KBDILLUMDOWN) {
        pr_info("[hid-kbd-brightness]: Holding down ..");

        brightness_counter(COUNTER_DEC);
        brightness_update();
    }
}

static void brightness_unload(void) {
    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        brightness.brightness = 0;
        brightness_update();
    }
}

/* Module Stuff */
module_init(kbd_init);
module_exit(kbd_exit);

MODULE_AUTHOR("PlayReissLP");
MODULE_DESCRIPTION("Translates kernel brightness key events to system commands.");
MODULE_DESCRIPTION("Kernel drivers for T2 Macs currently don't support changing brightness over Touchbar controls.");
MODULE_VERSION("a0.0.1");
MODULE_LICENSE("GPL");

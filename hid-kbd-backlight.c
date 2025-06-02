// SPDX-License-Identifier: GPL-2.0
/**
 * Apple T2 Mac - Keyboard Backlight Driver
 * 
 * Copyright (c) 2025 Marcel Berlinger
 */

#include "include/hid-kbd-backlight.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

/**
 * === Product Definitions ===
 */

// Apple Inc. Touch Bar Display.
#define VENDOR_ID_TB 0x5ac // TODO
#define PRODUCT_ID_TB 0x8302

/**
 * === Check Definitions - Brightness ===
 */

#define CHECK_BRIGHTNESS_LED 0x1
#define CHECK_BRIGHTNESS_LED_FUNC (CHECK_BRIGHTNESS_LED << 1)
#define CHECK_OK (CHECK_BRIGHTNESS_LED | CHECK_BRIGHTNESS_LED_FUNC)

/**
 * === Check Definitions - Key State ===
 */

#define CHECK_KEY_STUCK (1 << 4) // TODO: This may be removed.

/**
 * === Check Definitions - Unloading ===
 */

#define CHECK_UNLOAD (1 << 7)

/**
 * === Macros ===
 */

#define FLAGS_LOWER_NIBBLE(flags) ((flags) & 0xF)
#define FLAGS_UPPER_NIBBLE(flags) (((flags) >> 4) & 0xF)

/**
 * === Externs ===
 */

extern struct list_head leds_list;

/**
 * === Declarations ===
 */

enum counter_type_e counter_type_e;
enum key_type_e key_type_e;
enum key_state_e key_state_e;
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value);
static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id);
static void kbd_disconnect(struct input_handle* handle);
static int __init kbd_init(void);
static void __exit kbd_exit(void);
static void send_brightness_work_func(struct work_struct* work);
static void toggle_hold_work_func(struct work_struct* work);
static ssize_t brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t min_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t min_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t max_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t max_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t brightness_default_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t brightness_default_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t brightness_delay_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t brightness_delay_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t brightness_steps_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t brightness_steps_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static int brightness_set_blocking_hook(struct led_classdev* led_cdev, enum led_brightness brghtns);
static void brightness_init(void);
static void brightness_update(void);
static void brightness_counter(const enum counter_type_e type);
static void check_brightness_steps(void);
static struct led_classdev* find_led_by_name(const char* led_name);
static void brightness_store_steps(const unsigned short val);
static void schedule_brightness_store_steps(const unsigned short val, enum key_type_e key_type);
static bool schedule_delayed_work_fn(void);
static void kbd_illum_up(const enum key_state_e key_state);
static void kbd_illum_down(const enum key_state_e key_state);
static void reset_check_key_flags(void);
static void brightness_unload(void);

/**
 * === Enums ===
 */

/// @brief Event key states.
enum key_state_e {
    EV_KEY_UP,
    EV_KEY_DOWN,
    EV_KEY_LONG
};

/// @brief Determines if counter should count up or down.
enum counter_type_e {
    COUNTER_INC,
    COUNTER_DEC
};

/// @brief Touchbar illumination keys.
enum key_type_e {
    KBDILLUMUP = 1,
    KBDILLUMDOWN = 2
};

/**
 * === Structs ===
 */

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
    .name = "hid-kbd-backlight",
    .id_table = kbd_ids
};

/**
 * Represents keyboard backlight brightness control.
 * 
 * @led: Keyboard LED-device.
 * @brightness_set_blocking: Function pointer from the led_classdev struct
 * @brightnes: Brightness value of the keyboard backlight.
 * @min: Minimum brightness.
 * @max: Maximum brightness.
 * @default_brightness: The default brightness to set when module is loaded.
 * @delay: The delay between brightness changes when holding down keys. (in ms)
 * @steps: Steps to increase the brightness with.
 * @check_flags: Flags for brightness checks. // TODO: -- Add flag desciption table here --
 */
static struct {
    struct led_classdev* led;
    int (*brightness_set_blocking)(struct led_classdev* led_cdev, enum led_brightness brightness);
    unsigned short brightness;
    unsigned short min, max;
    unsigned short default_brightness;
    unsigned short delay;
    unsigned char steps;
    unsigned char check_flags;
} brightness = {
    .brightness = 0,
    .min = 32,
    .max = 512,
    .default_brightness = 256,
    .delay = 10,
    .steps = 8,
    .check_flags = 0
};

/**
 * File System Attributes.
 * 
 * Modes are: (OCTAL CONSTANT) -> [Owner, Group, Others].
 * 
 * Available modes: Write: [2] - Read: [4].
 */
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute brightness_attr =
    __ATTR(brightness_value, 0664, brightness_show, brightness_store);
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute min_brightness_attr =
    __ATTR(min_brightness_value, 0664, min_brightness_show, min_brightness_store);
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute max_brightness_attr =
    __ATTR(max_brightness_value, 0664, max_brightness_show, max_brightness_store);
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute brightness_default_attr =
    __ATTR(brightness_default_value, 0664, brightness_default_show, brightness_default_store);
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute brightness_delay_attr =
    __ATTR(brightness_delay, 0664, brightness_delay_show, brightness_delay_store);
/// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute brightness_steps_attr =
    __ATTR(brightness_steps, 0664, brightness_steps_show, brightness_steps_store);

/// @brief Worker for sending brightness blocking.
static struct work_struct send_brightness_work;
/// @brief Worker for holding down brightness keys.
static struct delayed_work toggle_hold_work;
/// @brief File System object.
static struct kobject* kbd_obj;

/**
 * === Variables ===
 */

/// @brief Logging tag.
static const unsigned char* LOG_TAG = "[hid-kbd-backlight]: ";

/**
 * === Event Functions ===
 */

/// @brief Linux Event: Handles keyboard events.
///
/// We use this to redirect Touchbar key events to our own advantage.
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value) {
    pr_debug("%stype: %d .. code: %d .. value: %d", LOG_TAG, type, code, value);

    if (value == EV_KEY_DOWN
        || value == EV_KEY_LONG) {
        switch(code) {
            case KEY_KBDILLUMUP:
                kbd_illum_up(value);
                break;
            case KEY_KBDILLUMDOWN:
                kbd_illum_down(value);
                break;
        }

        pr_debug("%sBrightness_check_flags: %d", LOG_TAG, brightness.check_flags);
    } else if(value == EV_KEY_UP) {
        pr_debug("%sState before release: %d", LOG_TAG, brightness.check_flags);
        pr_debug("%sState val: %d", LOG_TAG, CHECK_KEY_STUCK
            | (CHECK_KEY_STUCK << KBDILLUMUP)
            | (CHECK_KEY_STUCK << KBDILLUMDOWN)
        );
        
        // Resetting check key flags
        reset_check_key_flags();

        pr_debug("%sState after release: %d", LOG_TAG, brightness.check_flags);
        pr_debug("%sReleased key ..", LOG_TAG);

        cancel_delayed_work(&toggle_hold_work);
    }
}

/// @brief Linux Event: Initialize the handle on connection.
static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id) {
    struct input_handle* handle;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle) {
        return -ENOMEM;
    }

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "hid-kbd-backlight";

    if (input_register_handle(handle)) {
        kfree(handle);
        return -1;
    }

    if (input_open_device(handle)) {
        pr_err("%sFailed to open device: %s.", LOG_TAG, dev_name(&dev->dev));
        input_unregister_handle(handle);
        kfree(handle);
        return -1;
    }

    pr_debug("%sConnected to device: %s with id: %lx.", LOG_TAG, dev_name(&dev->dev), id->evbit[0]);
    pr_info("%sConnected to Touchbar device.", LOG_TAG);
    return 0;
}

/// @brief Linux Event: Clean up handle stuff on diconnect.
static void kbd_disconnect(struct input_handle* handle) {
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
    pr_info("%sDisconnected from Touchbar device.", LOG_TAG);
}

/// @brief Linux Event: Initialize keyboard backlight brightness stuff.
static int __init kbd_init(void) {
    // Init workers
    INIT_WORK(&send_brightness_work, send_brightness_work_func);
    INIT_DELAYED_WORK(&toggle_hold_work, toggle_hold_work_func);

    // Init brightness stuff and load default value
    brightness_init();
    brightness_store_steps(brightness.default_brightness);

    // Init filesystem
    kbd_obj = kobject_create_and_add("kbd_backlight", kernel_kobj);
    if(!kbd_obj) {
        return -ENOMEM;
    }

    return input_register_handler(&kbd_handler)
        & sysfs_create_file(kbd_obj, &brightness_attr.attr)
        & sysfs_create_file(kbd_obj, &min_brightness_attr.attr)
        & sysfs_create_file(kbd_obj, &max_brightness_attr.attr)
        & sysfs_create_file(kbd_obj, &brightness_default_attr.attr)
        & sysfs_create_file(kbd_obj, &brightness_delay_attr.attr)
        & sysfs_create_file(kbd_obj, &brightness_steps_attr.attr);
}

/// @brief Linux Event: Unload keyboard backlight brightness stuff on exit. 
static void __exit kbd_exit(void) {
    // Turn off backlight when unloading module
    brightness_unload();

    // Unload keyboard handler
    input_unregister_handler(&kbd_handler);

    // Unload filesystem
    sysfs_remove_file(kbd_obj, &brightness_attr.attr);
    sysfs_remove_file(kbd_obj, &min_brightness_attr.attr);
    sysfs_remove_file(kbd_obj, &max_brightness_attr.attr);
    sysfs_remove_file(kbd_obj, &brightness_default_attr.attr);
    sysfs_remove_file(kbd_obj, &brightness_delay_attr.attr);
    sysfs_remove_file(kbd_obj, &brightness_steps_attr.attr);
    kobject_put(kbd_obj);

    // (Unhooking): Reset brightness function pointer
    if(brightness.brightness_set_blocking) {
        brightness.led->brightness_set_blocking = brightness.brightness_set_blocking;
    }
}

/**
 * === Function Workers ===
 */

/// @brief Worker: Sets brightness blocking.
static void send_brightness_work_func(struct work_struct* work) {
    brightness.led->brightness = brightness.brightness;
    brightness.led->brightness_set_blocking(brightness.led, (enum led_brightness)brightness.brightness);
}

/// @brief Worker: Called when keys are held down.
///
/// Gets brightness flags and calls increment or decrement functions based on set values.
static void toggle_hold_work_func(struct work_struct* work) {
    // Three key state flags (upper most bits)
    unsigned char key_state = FLAGS_UPPER_NIBBLE(brightness.check_flags) >> 1;

    pr_debug("%sKey state value: %d", LOG_TAG, key_state);

    if(key_state & KBDILLUMUP) {
        pr_debug("%sHolding up ..", LOG_TAG);

        brightness_counter(COUNTER_INC);
        brightness_update();
    } else if(key_state & KBDILLUMDOWN) {
        pr_debug("%sHolding down ..", LOG_TAG);

        brightness_counter(COUNTER_DEC);
        brightness_update();
    }
}

/**
 * === File System Functions ===
 */

/// @brief File System: Called when written to the brightness_value file in the system kernel.
static ssize_t brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness_store_steps(val);
    }

    return count;
}

/// @brief File System: Called when read from the brightness_value file in the system kernel.
static ssize_t brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return sprintf(buf, "%d\n", brightness.brightness);
}

/// @brief File System: Called when written to the min_brightness_value file in the system kernel.
static ssize_t min_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness.min = (unsigned short)val;
    }

    return count;
}

/// @brief File System: Called when read from the min_brightness_value file in the system kernel.
static ssize_t min_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return brightness.min;
}

/// @brief File System: Called when written to the max_brightness_value file in the system kernel.
static ssize_t max_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness.max = (unsigned short)val;
    }

    return count;
}

/// @brief File System: Called when read from the max_brightness_value file in the system kernel.
static ssize_t max_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return brightness.max;
}

/// @brief File System: Called when written to the brightness_default file in the system kernel.
static ssize_t brightness_default_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness.default_brightness = (unsigned short)val;
    }

    return count;
}

/// @brief File System: Called when read from the brightness_default file in the system kernel.
static ssize_t brightness_default_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return brightness.default_brightness;
}

/// @brief File System: Called when written to the brightness_delay file in the system kernel.
static ssize_t brightness_delay_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness.delay = (unsigned short)val;
    }

    return count;
}

/// @brief File System: Called when read from the brightness_delay file in the system kernel.
static ssize_t brightness_delay_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return brightness.delay;
}

/// @brief File System: Called when written to the brightness_steps file in the system kernel.
static ssize_t brightness_steps_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        brightness.steps = (unsigned char)val;
    }

    return count;
}

/// @brief File System: Called when read from the brightness_steps file in the system kernel.
static ssize_t brightness_steps_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return brightness.steps;
}

/**
 * === Hooks ===
 */

/// @brief This function will be called as a hook for brightness_set_blocking.
/// If any other function call tries to use a different brightness value than in this module,
/// it will ignore its actions.
static int brightness_set_blocking_hook(struct led_classdev* led_cdev, enum led_brightness brghtns) {
    pr_debug("%sTrying to set brightness value from hook: %d", LOG_TAG, brghtns);

    if(brghtns == brightness.brightness) {
        brightness.brightness_set_blocking(led_cdev, brghtns);
    }

    return 0;
}

/**
 * === Functions ===
 */

/// @brief Initializes brightness led and checks function availability.
static void brightness_init(void) {
    if(!brightness.led) {
        brightness.led = find_led_by_name("apple::kbd_backlight");
    }

    if(brightness.led) {
        brightness.check_flags |= CHECK_BRIGHTNESS_LED;

        if(brightness.led->brightness_set_blocking) {
            brightness.check_flags |= CHECK_BRIGHTNESS_LED_FUNC;

            brightness.brightness_set_blocking = brightness.led->brightness_set_blocking;
            brightness.led->brightness_set_blocking = brightness_set_blocking_hook;
        }
    }
}

/// @brief Updates brightness when changed based on the current value. 
static void brightness_update(void) {
    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        unsigned short current_brightness;

        pr_debug("%sGetting current brightness ..", LOG_TAG);

        // Getting current brightness
        current_brightness = (unsigned short)(brightness.led->brightness);

        pr_debug("%sGot the following brightness: %d", LOG_TAG, current_brightness);

        // Checking if brightness has changed
        if(brightness.brightness != current_brightness) {
            pr_debug("%sSetting brightness to: %d", LOG_TAG, brightness.brightness);

            // Setting the current brightness value (This notifies the device)
            schedule_work(&send_brightness_work);
        }
    }
}

/// @brief Counts brightness level. (Increment or Decrement)
/// @param type The counter type.
static void brightness_counter(const enum counter_type_e type) {
    check_brightness_steps();

    switch(type) {
        case COUNTER_INC:
            if(brightness.brightness < brightness.max) {
                brightness.brightness += brightness.steps;
            }
            break;
        case COUNTER_DEC:
            if(brightness.brightness >= (brightness.min + brightness.steps)) {
                brightness.brightness -= brightness.steps;
            }
            break;
    }

    if((brightness.check_flags & CHECK_UNLOAD)
        && brightness.brightness == brightness.min) {
        pr_debug("%sSetting brightness to unloading state ..", LOG_TAG);

        brightness.brightness = 0;
    }

    pr_debug("%sType: %d | Brightness: %d | Brightness_max: %d", LOG_TAG, type, brightness.brightness, brightness.max);
}

/// @brief Calculates and sets brightness level based on the steps.
///
/// For example: When BRIGHTNESS_STEPS is set to 5, the brightness value will increase or decrease by a factor of 5. 
static void check_brightness_steps(void) {
    brightness.brightness = (unsigned short)(brightness.brightness / brightness.steps) * brightness.steps;
    pr_debug("%sCalculation result: %d", LOG_TAG, brightness.brightness);
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

/// @brief Calls brightness steps scheduler functions based on the specified value.
///
/// Increases or decreases brightness value by current brightness.steps value.
/// Should fade between current and specified value.
///
/// @param val The brightness value to step to (fade).
static void brightness_store_steps(const unsigned short val) {
    if(val != brightness.brightness) {
        if(val > brightness.brightness) {
            schedule_brightness_store_steps(val, KBDILLUMUP);
        } else {
            schedule_brightness_store_steps(val, KBDILLUMDOWN);
        }
    }
}

/// @brief Calculates brightness steps based on the specified brightness value.
///
/// Calls delayed work scheduler after calculation.
/// Waits until each work scheduler has finished execution.
/// Acts as a fading animation between brightness values.
///
/// @param val The brightness value to step to (fade).
/// @param key_type Increase or decrease illumination.
static void schedule_brightness_store_steps(const unsigned short val, enum key_type_e key_type) {
    unsigned short count = 0;
    
    // Calculating delta and dividing by steps
    if(key_type == KBDILLUMUP) {
        count = (val - brightness.brightness) / brightness.steps;
    } else {
        count = (brightness.brightness - val) / brightness.steps;
    }

    if(count) {
        // Resetting check key state
        cancel_delayed_work(&toggle_hold_work);
        reset_check_key_flags();

        brightness.check_flags |= (CHECK_KEY_STUCK << key_type);

        pr_debug("%sScheduler brightness: %d | val: %d | count: %d", LOG_TAG, brightness.brightness, val, count);

        for(unsigned short i = 0; i < count; ++i) {
            pr_debug("%sScheduling work until ..", LOG_TAG);

            while(!schedule_delayed_work_fn());
        }

        pr_debug("%sBrightness steps count: %d", LOG_TAG, count);
    }
}

/// @brief Calls the toggle_hold_work scheduler when holding down illumination keys.
///
/// This helps to set a specific msecs delay between calls when callinc this function.
///
/// @return If the work has finished execution.
static bool schedule_delayed_work_fn(void) {
    return schedule_delayed_work(&toggle_hold_work, msecs_to_jiffies(brightness.delay));
}

/// @brief Handles increasing keyboard illumination.
///
/// Either applies normal increase or scheduled increase.
/// @param key_state The current key event state.
static void kbd_illum_up(const enum key_state_e key_state) {
    switch(key_state) {
        case EV_KEY_DOWN:
            pr_debug("%sBrightness up ..", LOG_TAG);

            brightness_counter(COUNTER_INC);
            brightness_update();
            break;
        case EV_KEY_LONG:
            pr_debug("%sSetting illum up ..", LOG_TAG);

            brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMUP);
            schedule_delayed_work_fn();
            break;
        default:
            break;
    }
}

/// @brief Handles decreasing keyboard illumination.
///
/// Either applies normal decrease or scheduled decrease.
/// @param key_state The current key event state.
static void kbd_illum_down(const enum key_state_e key_state) {
    switch(key_state) {
        case EV_KEY_DOWN:
            pr_debug("%sBrightness down ..", LOG_TAG);
            
            brightness_counter(COUNTER_DEC);
            brightness_update();
            break;
        case EV_KEY_LONG:
            pr_debug("%sSetting illum down ..", LOG_TAG);
                        
            brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMDOWN);
            schedule_delayed_work_fn();
            break;
        default:
            break;
    }
}

/// @brief Resets key check flags for the brightness construct. 
static void reset_check_key_flags(void) {
    brightness.check_flags &= ~(
        CHECK_KEY_STUCK
            | (CHECK_KEY_STUCK << KBDILLUMUP)
            | (CHECK_KEY_STUCK << KBDILLUMDOWN)
    );
}

/// @brief Checks if brighness is 'locked an loaded'.
///
/// Sets check flags and calls brightness function to fade keyboard backlight brightness value to 0.
static void brightness_unload(void) {
    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        pr_info("%sUnloading brightness ..", LOG_TAG);
        
        brightness.check_flags |= CHECK_UNLOAD;
        brightness_store_steps(0);

        pr_info("%sBrightness unloaded.", LOG_TAG);
    }
}

/**
 * === Module Stuff ===
 */

module_init(kbd_init);
module_exit(kbd_exit);

MODULE_AUTHOR("Marcel Berlinger");
MODULE_DESCRIPTION("Translates Kernel Touchbar key events to System Commands.");
MODULE_DESCRIPTION(
    "Kernel drivers for T2 Macs currently don't support changing keyboard backlight brightness over Touchbar controls."
);
MODULE_VERSION("a0.0.1");
MODULE_LICENSE("GPL");

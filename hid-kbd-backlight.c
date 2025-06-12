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
#include <linux/pm.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/delay.h>

/**
 * === Module Definitions ===
 */

#define MODULE_NAME "kbd-backlight"

/**
 * === Product Definitions ===
 */

#define INPUT_DEVICE_ID_MATCH_VENDOR_PRODUCT ( \
    INPUT_DEVICE_ID_MATCH_VENDOR               \
        | INPUT_DEVICE_ID_MATCH_PRODUCT        \
)
// Apple Inc. Touch Bar Display.
#define VENDOR_ID_TB 0x5ac
#define PRODUCT_ID_TB 0x8302
// Apple Inc. Apple Internal Keyboard / Trackpad.
#define VENDOR_ID_KBD 0x5ac
#define PRODUCT_ID_KBD 0x27c

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
#define PR_INFO(fmt, ...) pr_info("%s" fmt, LOG_TAG, ##__VA_ARGS__)
#define PR_ERR(fmt, ...) pr_err("%s" fmt, LOG_TAG, ##__VA_ARGS__)
#define PR_DEBUG(fmt, ...) pr_debug("%s" fmt, LOG_TAG, ##__VA_ARGS__)
#define PR_DONE(void) PR_INFO("Done.")
#define SHOW_NUM_FN(buf_ptr, ...) sprintf(buf_ptr, "%d\n", __VA_ARGS__)
#define RET_CHECK_ERROR(val) do { \
    if((val) != 0) {              \
        return val;               \
    }                             \
} while(0)
#define RET_CHECK_NO_MEM(val) do { \
    if(!(val)) {                   \
        return -ENOMEM;            \
    }                              \
} while(0)
#define RET_CHECK_DELAY_CANCELLED(flags_ptr) do {       \
    if((is_and_set_timer_delay_cancelled(flags_ptr))) { \
        return;                                         \
    }                                                   \
} while(0)
#define FREE(val_ptr) do { \
    if((val_ptr)) {        \
        kfree((val_ptr));  \
    }                      \
} while(0)

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
enum fs_type_e fs_type_e; 
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value);
static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id);
static void kbd_disconnect(struct input_handle* handle);
static int __init kbd_init(void);
static void __exit kbd_exit(void);
static void send_brightness_work_fn(struct work_struct* work);
static void toggle_hold_work_fn(struct work_struct* work);
static void timer_delayed_dimm_work_fn(struct work_struct* work);
static void find_led_by_name_retry_work_fn(struct work_struct* work);
static void resume_work_fn(struct work_struct* work);
static void reset_dimm_work_fn(struct work_struct* work);
static void send_power_mgmnt_work_fn(struct work_struct* work);
static void schedule_brightness_store_steps_work_fn(struct work_struct* work);
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
static ssize_t timer_delay_first_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t timer_delay_first_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static ssize_t timer_delay_second_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count);
static ssize_t timer_delay_second_show(struct kobject* obj, struct kobj_attribute* attr, char* buf);
static int brightness_set_blocking_hook(struct led_classdev* led_cdev, enum led_brightness brghtns);
static int pm_state_notifier(struct notifier_block* nb, unsigned long action, void* data);
static void brightness_init(void);
static void brightness_update(void);
static void brightness_counter(const enum counter_type_e type);
static void check_brightness_steps(void);
static struct led_classdev* find_led_by_name(const char* led_name);
static void brightness_store_steps(const unsigned short val);
static void schedule_brightness_store_steps(const unsigned short val, enum key_type_e key_type);
static inline bool schedule_delayed_work_fn(void);
static void kbd_illum_up(const enum key_state_e key_state);
static void kbd_illum_down(const enum key_state_e key_state);
static inline unsigned short get_brightness_half(void);
static inline void schedule_delayed_dimm_fn_async(const unsigned int delay);
static inline struct work_struct* schedule_brightness_steps_fn_async(const unsigned short val);
static inline void schedule_brightness_steps_fn_sync(const unsigned short val);
static inline struct work_struct* schedule_find_led_by_name_retry_fn_async(char* led_name);
static inline void schedule_find_led_by_name_retry_fn_sync(char* led_name);
static inline unsigned char is_and_set_timer_delay_cancelled(unsigned char* flags);
static inline unsigned char reset_timer_delay_flags(unsigned char* flags);
static inline size_t store_fn(const enum fs_type_e type, const char* buf, const size_t count);
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

/// @brief Timer delay dimming flags.
enum timer_delayed_dimm_flags_e {
    TIMER_DELAY_STATE_CANCELLED = 1
};

/// @brief Active file system types.
enum fs_type_e {
    FSTYPE_SCHEDULE_BRIGHTNESS,
    FSTYPE_MIN_BRIGHTNESS,
    FSTYPE_MAX_BRIGHTNESS,
    FSTYPE_DEFAULT_BRIGHTNESS,
    FSTYPE_DELAY_BRIGHTNESS,
    FSTYPE_STEPS_BRIGHTNESS,
    FSTYPE_TIMER_DELAY_FIRST,
    FSTYPE_TIMER_DELAY_SECOND
};

/**
 * === Structs ===
 */

static const struct input_device_id kbd_ids[] = {
    {
        .flags = INPUT_DEVICE_ID_MATCH_VENDOR_PRODUCT,
        .vendor = VENDOR_ID_TB,
        .product = PRODUCT_ID_TB
    },
    {
        .flags = INPUT_DEVICE_ID_MATCH_VENDOR_PRODUCT,
        .vendor = VENDOR_ID_KBD,
        .product = PRODUCT_ID_KBD
    },
    { },
};

static struct input_handler kbd_handler = {
    .event = kbd_event,
    .connect = kbd_connect,
    .disconnect = kbd_disconnect,
    .name = MODULE_NAME,
    .id_table = kbd_ids
};

static struct notifier_block pm_notify_block = {
    .notifier_call = pm_state_notifier
};

struct schedule_brightness_steps {
    struct work_struct steps_work;
    unsigned short val;
};

struct find_led_by_name {
    struct work_struct led_work;
    char* led_name;
};

struct timer_delayed_dimm {
    struct delayed_work timer_dimm_work;
    unsigned char flags;
};

/**
 * Represents the first and second delay for dimming down the keyboard brightness.
 * 
 * Current default is the following:
 * 
 * first -> 1 Minute (60000ms)
 * second -> 15 Seconds (15000ms)
 * 
 * @first: The first dimming delay, it will set the current brightness to half.
 * @second: The second dimming delay, it will set the current brightness to 0
 */
static struct {
    unsigned int first;
    unsigned int second;
} timer_dimm_delay = {
    .first = 60000,
    .second = 15000
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
 * @check_flags: Flags for brightness checks.
 * 
 * @check_flags: -- Table of Contents --
 * 
 *      Check flags are 8 bits in length and should look like the following:
 *      (LOWER NIBBLE) [ 0 | 0 | CHECK_BRIGHTNESS_LED_FUNC | CHECK_BRIGHTNESS_LED ]
 *      (UPPER NIBBLE) [ CHECK_UNLOAD | 0 | 0 | CHECK_KEY_STUCK ]
 * 
 *      The Zero values in the Upper Nibble will be shifted one to the right later on.
 *      When shifted, those two values will represent the following: [ 0 | 0 | KBDILLUMDOWN | KBDILLUMUP ]
 * 
 * @check_flags: -- Table of Contents --
 * 
 */
static struct {
    struct led_classdev* led;
    int (*brightness_set_blocking)(struct led_classdev* led_cdev, enum led_brightness brightness);
    unsigned short brightness;
    unsigned short min, max;
    unsigned short default_brightness;
    unsigned short delay;
    unsigned short previous;
    unsigned char steps;
    unsigned char check_flags;
} brightness = {
    .brightness = 0,
    .min = 32,
    .max = 512,
    .default_brightness = 256,
    .delay = 10,
    .previous = 0,
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
// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute timer_delay_first_attr =
    __ATTR(timer_delay_first, 0664, timer_delay_first_show, timer_delay_first_store);
// Current mode set: [0] | [Read + Write] | [Read + Write] | [Read].
static struct kobj_attribute timer_delay_second_attr =
    __ATTR(timer_delay_second, 0664, timer_delay_second_show, timer_delay_second_store);

/// @brief Worker for sending brightness blocking.
static struct work_struct send_brightness_work;
/// @brief Worker for handling states on power management.
static struct work_struct send_power_mgmnt_work;
/// @brief Worker for handling resume actions.
static struct work_struct resume_work;
/// @brief Worker for resetting dimming actions.
static struct work_struct reset_dimm_work;
/// @brief Delayed Worker for holding down brightness keys.
static struct delayed_work toggle_hold_work;
/// @brief File System object.
static struct kobject* kbd_obj;
/// @brief Custom Worker for scheduling brightness steps.
static struct schedule_brightness_steps* schedule_brightness_store_steps_work;
/// @brief Custom Worker for retrying to find led by name.
static struct find_led_by_name* find_led_by_name_retry_work;
/// @brief Customer Delayed Worker for time based dimming delay.
static struct timer_delayed_dimm* timer_delayed_dimm_work;

/**
 * === Variables ===
 */

/// @brief Logging tag.
static const unsigned char* LOG_TAG = "[" MODULE_NAME "]: ";

/**
 * === Event Functions ===
 */

/// @brief LINUXEVENT: Handles keyboard events.
///
/// We use this to redirect Touchbar key events to our own advantage.
static void kbd_event(struct input_handle* handle, unsigned int type, unsigned int code, int value) {
    PR_DEBUG("type: %d .. code: %d .. value: %d", type, code, value);

    // Reset the dimming timer upon receiving any other key presses from specified devices
    if(brightness.previous) {
        schedule_brightness_steps_fn_async(brightness.previous);
        schedule_work(&reset_dimm_work);
    }

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

        PR_DEBUG("Brightness_check_flags: %d", brightness.check_flags);
    } else if(value == EV_KEY_UP) {
        PR_DEBUG("State before release: %d", brightness.check_flags);
        PR_DEBUG("State val: %d", CHECK_KEY_STUCK
            | (CHECK_KEY_STUCK << KBDILLUMUP)
            | (CHECK_KEY_STUCK << KBDILLUMDOWN)
        );
        
        // Resetting check key flags
        // reset_check_key_flags();

        PR_DEBUG("State after release: %d", brightness.check_flags);
        PR_DEBUG("Released key ..");
    }
}

/// @brief LINUXEVENT: Initialize the handle on connection.
static int kbd_connect(struct input_handler* handler, struct input_dev* dev, const struct input_device_id* id) {
    struct input_handle* handle;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle) {
        return -ENOMEM;
    }

    handle->dev = dev;
    handle->handler = handler;
    handle->name = MODULE_NAME;

    if (input_register_handle(handle)) {
        kfree(handle);
        return -1;
    }

    if (input_open_device(handle)) {
        PR_ERR("Failed to open device: %s.", dev_name(&dev->dev));
        input_unregister_handle(handle); 
        kfree(handle);
        return -1;
    }

    PR_INFO("Connected to device: %s with id: %lx.", dev_name(&dev->dev), id->evbit[0]);
    return 0;
}

/// @brief LINUXEVENT: Clean up handle stuff on disconnect.
static void kbd_disconnect(struct input_handle* handle) {
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);

    PR_INFO("Disconnected from device: %s.", dev_name(&handle->dev->dev));
}

/// @brief LINUXEVENT: Initialize keyboard backlight brightness stuff.
static int __init kbd_init(void) {
    pr_info("%sRunning init ..", LOG_TAG);

    RET_CHECK_NO_MEM(
        schedule_brightness_store_steps_work = kmalloc(sizeof(*schedule_brightness_store_steps_work), GFP_KERNEL)
    );
    
    RET_CHECK_NO_MEM(
        find_led_by_name_retry_work = kmalloc(sizeof(*find_led_by_name_retry_work), GFP_KERNEL)
    );
    find_led_by_name_retry_work->led_name = "apple::kbd_backlight";

    RET_CHECK_NO_MEM(
        timer_delayed_dimm_work = kmalloc(sizeof(*timer_delayed_dimm_work), GFP_KERNEL)
    );
    timer_delayed_dimm_work->flags = 0;

    // Init workers
    INIT_WORK(&send_brightness_work, send_brightness_work_fn);
    INIT_WORK(&send_power_mgmnt_work, send_power_mgmnt_work_fn);
    INIT_WORK(&resume_work, resume_work_fn);
    INIT_WORK(&reset_dimm_work, reset_dimm_work_fn);
    INIT_WORK(&find_led_by_name_retry_work->led_work, find_led_by_name_retry_work_fn);
    INIT_WORK(&schedule_brightness_store_steps_work->steps_work, schedule_brightness_store_steps_work_fn);
    INIT_DELAYED_WORK(&toggle_hold_work, toggle_hold_work_fn);
    INIT_DELAYED_WORK(&timer_delayed_dimm_work->timer_dimm_work, timer_delayed_dimm_work_fn);

    // Init brightness stuff and load default value
    // TODO: This may need to be a retry based action in the future
    brightness_init();
    schedule_brightness_steps_fn_async(brightness.default_brightness);

    // Init filesystem
    RET_CHECK_NO_MEM(
        kbd_obj = kobject_create_and_add("kbd_backlight", kernel_kobj)
    );

    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &brightness_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &min_brightness_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &max_brightness_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &brightness_default_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &brightness_delay_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &brightness_steps_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &timer_delay_first_attr.attr));
    RET_CHECK_ERROR(sysfs_create_file(kbd_obj, &timer_delay_second_attr.attr));

    // Init power management notifier
    RET_CHECK_ERROR(register_pm_notifier(&pm_notify_block));
    
    // Init input handler
    RET_CHECK_ERROR(input_register_handler(&kbd_handler));

    // Scheduling delayed dimming
    schedule_delayed_dimm_fn_async(timer_dimm_delay.first);

    return 0;
}

/// @brief LINUXEVENT: Unload keyboard backlight brightness stuff on exit. 
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
    sysfs_remove_file(kbd_obj, &timer_delay_first_attr.attr);
    sysfs_remove_file(kbd_obj, &timer_delay_second_attr.attr);
    kobject_put(kbd_obj);

    // Flush and Free brightness store steps work
    flush_work(&schedule_brightness_store_steps_work->steps_work);
    FREE(schedule_brightness_store_steps_work);

    // Flush and free finding led name retry work
    flush_work(&find_led_by_name_retry_work->led_work);
    FREE(find_led_by_name_retry_work);

    // Cancel and free timer delayed dimm work
    cancel_delayed_work(&timer_delayed_dimm_work->timer_dimm_work);
    FREE(timer_delayed_dimm_work);
    
    // Remove power management notifier
    unregister_pm_notifier(&pm_notify_block);

    PR_INFO("Unhooking ..");
    // (Unhooking): Reset brightness function pointer
    if(brightness.brightness_set_blocking) {
        brightness.led->brightness_set_blocking = brightness.brightness_set_blocking;
    }
    PR_DONE();
}

/**
 * === Worker Functions ===
 */

/// @brief WORKER: Sets brightness blocking.
static void send_brightness_work_fn(struct work_struct* work) {
    PR_DEBUG("Sending info | led: %d", brightness.led ? 1 : 0);

    if(!brightness.led) {
        brightness_init();

        if(brightness.led) {
            PR_INFO("Brightness init successfull!");

            brightness.led->brightness = brightness.brightness = 0;
        } else {
            PR_ERR("Brightness init failed!");
        }
    }

    if(brightness.led
        && brightness.led->brightness_set_blocking) {
        brightness.led->brightness_set_blocking(brightness.led, (enum led_brightness)brightness.brightness);
        
        brightness.led->brightness = brightness.brightness;

        PR_DEBUG("Success.");
    } else {
        PR_ERR("Error setting brightness!");
    }
}

/// @brief WORKER: Called when keys are held down.
///
/// Gets brightness flags and calls increment or decrement functions based on set values.
static void toggle_hold_work_fn(struct work_struct* work) {
    // Three key state flags (upper most bits)
    unsigned char key_state = FLAGS_UPPER_NIBBLE(brightness.check_flags) >> 1;

    PR_DEBUG("Key state value: %d", key_state);

    if(key_state & KBDILLUMUP) {
        PR_DEBUG("Holding up ..");

        brightness_counter(COUNTER_INC);
        brightness_update();
    } else if(key_state & KBDILLUMDOWN) {
        PR_DEBUG("Holding down ..");

        brightness_counter(COUNTER_DEC);
        brightness_update();
    }
}

/// @brief WORKER: Responsible for the dimming and timing process. 
static void timer_delayed_dimm_work_fn(struct work_struct* work) {
    PR_DEBUG("Running delayed work ..");

    PR_DEBUG("Flags before reset: %d", timer_delayed_dimm_work->flags);
    reset_timer_delay_flags(&timer_delayed_dimm_work->flags);
    PR_DEBUG("Flags after reset: %d", timer_delayed_dimm_work->flags);

    RET_CHECK_DELAY_CANCELLED(&timer_delayed_dimm_work->flags);

    if(brightness.brightness) {
        // Saving previous brightness value
        if(!brightness.previous) {
            brightness.previous = brightness.brightness;
        }

        // Set brightness to half and waiting for the brightness change
        schedule_brightness_steps_fn_sync(get_brightness_half());

        RET_CHECK_DELAY_CANCELLED(&timer_delayed_dimm_work->flags);

        if(brightness.brightness) {
            // Sleeping for the second part of the dimming
            msleep(timer_dimm_delay.second);

            RET_CHECK_DELAY_CANCELLED(&timer_delayed_dimm_work->flags);

            // Set brightness to 0 with unload check and wait for brightness change
            brightness.check_flags |= CHECK_UNLOAD;
            schedule_brightness_steps_fn_sync(0);

            // Reset unload check flag
            brightness.check_flags ^= CHECK_UNLOAD;
        }
    }
}

/// @brief WORKER: Looks for the led and retries a set amount of times until found.
static void find_led_by_name_retry_work_fn(struct work_struct* work) {
    struct find_led_by_name* led_name;
    int retry_count;
    struct led_classdev* led;

    led_name = container_of(work, struct find_led_by_name, led_work);
    if(!led_name) {
        PR_ERR("Finding led name worker failed!");
        return;
    }

    if(!led_name->led_name) {
        PR_ERR("Led name empty!");
        return;
    }
    
    PR_INFO("Looking for led .. retry ..");

    retry_count = 10;
    while(retry_count-- > 0) {
        led = find_led_by_name(led_name->led_name);
        if(led) {
            PR_DEBUG("Led found.");
            break;
        }

        PR_DEBUG("Looping .. Led not found.");

        msleep(1000);
    }

    if(led) {
        brightness.check_flags |= CHECK_BRIGHTNESS_LED;

        // Setting current led to result
        brightness.led = led;

        if(brightness.led->brightness_set_blocking) {
            brightness.check_flags |= CHECK_BRIGHTNESS_LED_FUNC;

            PR_INFO("Hooking ..");
            brightness.brightness_set_blocking = brightness.led->brightness_set_blocking;
            brightness.led->brightness_set_blocking = brightness_set_blocking_hook;
            PR_DONE();
        }
    }
}

/// @brief WORKER: Responsible for the resuming process. (coming from suspend, hibernate etc.)
static void resume_work_fn(struct work_struct* work) {
    // Clearing and looking for led device
    brightness.brightness = 0;
    brightness.led = NULL;
    // Scheduling and waiting for led work
    schedule_find_led_by_name_retry_fn_sync(NULL);

    // Checking if led device found
    if(brightness.led) {
        // Sending and waiting for brightness update work
        schedule_brightness_steps_fn_sync(brightness.default_brightness);
    } else {
        PR_ERR("Failed to restore previous brightness value. LED device not found!");
    }

    // When led is available start dimming work
    if(brightness.led) {
        schedule_delayed_dimm_fn_async(timer_dimm_delay.first);
    }
}

/// @brief WORKER: Responsible for ressetting dimming actions.
static void reset_dimm_work_fn(struct work_struct* work) {
    // Wait until brightness steps is finished
    flush_work(&schedule_brightness_store_steps_work->steps_work);

    timer_delayed_dimm_work->flags |= TIMER_DELAY_STATE_CANCELLED;
    cancel_delayed_work(&timer_delayed_dimm_work->timer_dimm_work);

    // // Revert brightness to the privious one
    // flush_work(
    //     brightness_steps_scheduler_work_fn(brightness.previous)
    // );

    // cancel_delayed_work_sync(&timer_delayed_dimm_work->timer_dimm_work);

    // Restart dimming delay
    schedule_delayed_dimm_fn_async(timer_dimm_delay.first);

    // timer_delayed_dimm_work->flags ^= TIMER_DELAY_STATE_CANCELLED;
}

/// @brief WORKER: Responsible for sending work from power management.
static void send_power_mgmnt_work_fn(struct work_struct* work) {
    schedule_brightness_steps_fn_async(0);

    // schedule_brightness_store_steps_work->val = 0;
    // schedule_work(&schedule_brightness_store_steps_work->steps_work);

    // brightness.brightness = 0;
    // brightness_store_steps(brightness.default_brightness);
}

/// @brief WORKER: Responsible for sending the current brightness steps to the next stepping function.
static void schedule_brightness_store_steps_work_fn(struct work_struct* work) {
    PR_DEBUG("Sending work ..");
    PR_DEBUG("Work: %d", work ? 1 : 0);

    struct schedule_brightness_steps* container = container_of(work, struct schedule_brightness_steps, steps_work);
    if(container) {
        PR_DEBUG("Current brightness: %d | Setting brightness: %d", brightness.brightness, container->val);

        brightness_store_steps(container->val);
    } else {
        PR_ERR("Sending failed!");
    }
}

/**
 * === File System Functions ===
 */

/// @brief FILESYSTEM: Called when written to the brightness_value file in the system kernel.
static ssize_t brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_SCHEDULE_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the brightness_value file in the system kernel.
static ssize_t brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.brightness);
}

/// @brief FILESYSTEM: Called when written to the min_brightness_value file in the system kernel.
static ssize_t min_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_MIN_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the min_brightness_value file in the system kernel.
static ssize_t min_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.min);
}

/// @brief FILESYSTEM: Called when written to the max_brightness_value file in the system kernel.
static ssize_t max_brightness_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_MAX_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the max_brightness_value file in the system kernel.
static ssize_t max_brightness_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.max);
}

/// @brief FILESYSTEM: Called when written to the brightness_default file in the system kernel.
static ssize_t brightness_default_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_DEFAULT_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the brightness_default file in the system kernel.
static ssize_t brightness_default_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.default_brightness);
}

/// @brief FILESYSTEM: Called when written to the brightness_delay file in the system kernel.
static ssize_t brightness_delay_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_DELAY_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the brightness_delay file in the system kernel.
static ssize_t brightness_delay_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.delay);
}

/// @brief FILESYSTEM: Called when written to the brightness_steps file in the system kernel.
static ssize_t brightness_steps_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_STEPS_BRIGHTNESS, buf, count);
}

/// @brief FILESYSTEM: Called when read from the brightness_steps file in the system kernel.
static ssize_t brightness_steps_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, brightness.steps);
}

/// @brief FILESYSTEM: Called when written to the timer_delay_first file in the system kernel.
static ssize_t timer_delay_first_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_TIMER_DELAY_FIRST, buf, count);
}

/// @brief FILESYSTEM: Called when read from the timer_delay_first file in the system kernel.
static ssize_t timer_delay_first_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, timer_dimm_delay.first);
}

/// @brief FILESYSTEM: Called when written to the timer_delay_second file in the system kernel.
static ssize_t timer_delay_second_store(struct kobject* obj, struct kobj_attribute* attr, const char* buf, size_t count) {
    return store_fn(FSTYPE_TIMER_DELAY_SECOND, buf, count);
}

/// @brief FILESYSTEM: Called when read from the timer_delay_second file in the system kernel.
static ssize_t timer_delay_second_show(struct kobject* obj, struct kobj_attribute* attr, char* buf) {
    return SHOW_NUM_FN(buf, timer_dimm_delay.second);
}

/**
 * === Hooks ===
 */

/// @brief HOOK: This function will be called as a hook for brightness_set_blocking.
///
/// If any other function call tries to use a different brightness value than in this module,
/// it will ignore its actions.
static int brightness_set_blocking_hook(struct led_classdev* led_cdev, enum led_brightness brghtns) {
    PR_DEBUG("Trying to set brightness value from hook: %d", brghtns);

    if(brghtns == brightness.brightness) {
        brightness.brightness_set_blocking(led_cdev, brghtns);
    }

    return 0;
}

/**
 * === Notifiers ===
 */

/// @brief NOTIFIER: This notifies suspend or hibernate events on prepare or post.
///
/// Handles reset and initialization of this module after following events.
static int pm_state_notifier(struct notifier_block* nb, unsigned long action, void* data) {
    PR_DEBUG("Inside power management fn ..");

    switch(action) {
        case PM_HIBERNATION_PREPARE:
        case PM_SUSPEND_PREPARE:
            PR_INFO("Preparing power management ..");

            // Flush and Free brightness store steps
            cancel_delayed_work_sync(&toggle_hold_work);
            cancel_work_sync(&schedule_brightness_store_steps_work->steps_work);
            cancel_work_sync(&send_brightness_work);
            cancel_delayed_work_sync(&timer_delayed_dimm_work->timer_dimm_work);
            cancel_work_sync(&reset_dimm_work);

            PR_INFO("Unhooking ..");
            // (Unhooking): Reset brightness function pointer
            if(brightness.brightness_set_blocking) {
                brightness.led->brightness_set_blocking = brightness.brightness_set_blocking;
            }
            PR_DONE();
            break;

        case PM_POST_HIBERNATION:
        case PM_POST_SUSPEND:
            PR_DEBUG("Getting the following action id: %ld", action);
            PR_DEBUG("Previous brightness: %d", brightness.brightness);
            PR_DEBUG("LED available: %d", brightness.led ? 1 : 0);

            PR_INFO("Running init ..");
            
            PR_DEBUG("Current flags when resumed: %d", brightness.check_flags);

            // Resetting brightness flags
            brightness.check_flags = 0;

            PR_DEBUG("Current default brightness: %d", brightness.default_brightness);

            // Init brightness stuff and load default value
            schedule_work(&resume_work);
            break;
    }

    return NOTIFY_OK;
}

/**
 * === Functions ===
 */

/// @brief FN: Initializes brightness led and checks function availability.
static void brightness_init(void) {
    if(!brightness.led) {
        PR_INFO("Looking for led ..");
        brightness.led = find_led_by_name(find_led_by_name_retry_work->led_name);
    }

    if(brightness.led) {
        brightness.check_flags |= CHECK_BRIGHTNESS_LED;

        if(!brightness.brightness_set_blocking) {
            brightness.check_flags |= CHECK_BRIGHTNESS_LED_FUNC;

            PR_INFO("Hooking ..");
            brightness.brightness_set_blocking = brightness.led->brightness_set_blocking;
            brightness.led->brightness_set_blocking = brightness_set_blocking_hook;
            PR_DONE();
        }
    }
}

/// @brief FN: Updates brightness when changed based on the current value. 
static void brightness_update(void) {
    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        unsigned short current_brightness = 0;

        PR_DEBUG("Getting current brightness ..");

        // Getting current brightness
        if(brightness.led) {
            current_brightness = (unsigned short)(brightness.led->brightness);
        }

        PR_DEBUG("Got the following brightness: %d", current_brightness);

        // Checking if brightness has changed
        if(brightness.brightness != current_brightness) {
            PR_DEBUG("Setting brightness to: %d", brightness.brightness);

            // Setting the current brightness value (This notifies the device)
            schedule_work(&send_brightness_work);
        }
    }
}

/// @brief FN: Counts brightness level. (Increment or Decrement)
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
        PR_DEBUG("Setting brightness to unloading state ..");

        brightness.brightness = 0;
    }

    // if(brightness.previous
    //     && brightness.previous == brightness.brightness) {
    //     // Resetting previous value
    //     brightness.previous = 0;
    // }

    PR_DEBUG("Type: %d | Brightness: %d | Brightness_max: %d", type, brightness.brightness, brightness.max);
}

/// @brief FN: Calculates and sets brightness level based on the steps.
///
/// For example: When BRIGHTNESS_STEPS is set to 5, the brightness value will increase or decrease by a factor of 5. 
static void check_brightness_steps(void) {
    brightness.brightness = (unsigned short)(brightness.brightness / brightness.steps) * brightness.steps;
    PR_DEBUG("Calculation result: %d", brightness.brightness);
}

/// @brief FN: Tries to find led by its name.
/// 
/// @param led_name The name of the led to look for.
///
/// @return Returns led when found, but NULL when not.
static struct led_classdev* find_led_by_name(const char* led_name) {
    struct led_classdev* led;

    list_for_each_entry(led, &leds_list, node) {
        if(strcmp(led->name, led_name) == 0) {
            return led;
        }
    }

    return NULL;
}

/// @brief FN: Calls brightness steps scheduler functions based on the specified value.
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

/// @brief FN: Calculates brightness steps based on the specified brightness value.
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
        cancel_delayed_work_sync(&toggle_hold_work);
        reset_check_key_flags();

        brightness.check_flags |= (CHECK_KEY_STUCK << key_type);

        // pr_debug("%sScheduler brightness: %d | val: %d | count: %d", LOG_TAG, brightness.brightness, val, count);

        for(unsigned short i = 0; i < count; ++i) {
            PR_DEBUG("Scheduling work until ..");

            while(!schedule_delayed_work_fn());
        }

        // Resetting check key flags
        reset_check_key_flags();

        PR_DEBUG("Brightness steps count: %d", count);
    }
}

/// @brief FN: Calls the toggle_hold_work scheduler when holding down illumination keys.
///
/// This helps to set a specific msecs delay between calls when callinc this function.
///
/// @return If the work has finished execution.
static inline bool schedule_delayed_work_fn(void) {
    return schedule_delayed_work(&toggle_hold_work, msecs_to_jiffies(brightness.delay));
}

/// @brief FN: Handles increasing keyboard illumination.
///
/// Either applies normal increase or scheduled increase.
/// @param key_state The current key event state.
static void kbd_illum_up(const enum key_state_e key_state) {
    switch(key_state) {
        case EV_KEY_DOWN:
            PR_DEBUG("Brightness up ..");

            brightness_counter(COUNTER_INC);
            brightness_update();
            break;
        case EV_KEY_LONG:
            PR_DEBUG("Setting illum up ..");

            brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMUP);
            schedule_delayed_work_fn();
            break;
        default:
            break;
    }
}

/// @brief FN: Handles decreasing keyboard illumination.
///
/// Either applies normal decrease or scheduled decrease.
///
/// @param key_state The current key event state.
static void kbd_illum_down(const enum key_state_e key_state) {
    switch(key_state) {
        case EV_KEY_DOWN:
            PR_DEBUG("Brightness down ..");
            
            brightness_counter(COUNTER_DEC);
            brightness_update();
            break;
        case EV_KEY_LONG:
            PR_DEBUG("Setting illum down ..");
                        
            brightness.check_flags |= (CHECK_KEY_STUCK << KBDILLUMDOWN);
            schedule_delayed_work_fn();
            break;
        default:
            break;
    }
}

/// @brief FN: Returns half of the current brightness value. 
static inline unsigned short get_brightness_half(void) {
    return (unsigned short)((brightness.brightness * 5) / 10);
}

/// @brief FN: Re-Schedules and modifies delayed brightness dimming work asynchronous.
///
/// @param delay The delay to use.
static inline void schedule_delayed_dimm_fn_async(const unsigned int delay) {
    mod_delayed_work(system_wq, &timer_delayed_dimm_work->timer_dimm_work, msecs_to_jiffies(delay));
}

/// @brief FN: Schedules brightness steps asynchronous.
///
/// @param val The brightness value to use.
///
/// @return Current work.
static inline struct work_struct* schedule_brightness_steps_fn_async(const unsigned short val) {
    schedule_brightness_store_steps_work->val = val;
    schedule_work(&schedule_brightness_store_steps_work->steps_work);

    return &schedule_brightness_store_steps_work->steps_work;
}

/// @brief FN: Schedules brightness steps synchronous.
///
/// Calls its asynchronous counterpart.
///
/// @param val The brightness value to use.
static inline void schedule_brightness_steps_fn_sync(const unsigned short val) {
    flush_work(
        schedule_brightness_steps_fn_async(val)
    );
}

/// @brief FN: Schedules finding led by name retry asynchronous.
///
/// @param led_name The led name to look for.
///
/// @return Current work.
static inline struct work_struct* schedule_find_led_by_name_retry_fn_async(char* led_name) {
    if(led_name) {
        find_led_by_name_retry_work->led_name = led_name;
        schedule_work(&find_led_by_name_retry_work->led_work);
    } else {
        schedule_work(&find_led_by_name_retry_work->led_work);
    }

    return &find_led_by_name_retry_work->led_work;
}

/// @brief FN: Schedules finding led by name retry synchronous.
///
/// Calls its asynchronous counterpart.
///
/// @param led_name The led name to look for.
static inline void schedule_find_led_by_name_retry_fn_sync(char* led_name) {
    flush_work(
        schedule_find_led_by_name_retry_fn_async(led_name)
    );
}

/// @brief FN: Checks flags if timer delay is cancelled and resets flags
/// and cancels brightness steps work.
/// 
/// @param flags Flags of the timer delay work.
/// 
/// @return Is the timer delay cancelled.
static inline unsigned char is_and_set_timer_delay_cancelled(unsigned char* flags) {
    // unsigned char is_cancelled = (*flags & TIMER_DELAY_STATE_CANCELLED);

    if(reset_timer_delay_flags(flags)) {
        cancel_work(&schedule_brightness_store_steps_work->steps_work);

        PR_DEBUG("======== Timer delay cancelled: 1 ========");
        return 1;
    }

    PR_DEBUG("======== Timer delay cancelled: 0 ========");
    return 0;
}

/// @brief FN: Resets timer delay flags if present.
/// @param flags The flags pointer to check.
/// @return If reset flag was set before calling this function.
static inline unsigned char reset_timer_delay_flags(unsigned char* flags) {
    if(*flags & TIMER_DELAY_STATE_CANCELLED) {
        *flags ^= TIMER_DELAY_STATE_CANCELLED;

        return 1;
    }

    return 0;
}

/// @brief FN: Maps received values to the correct types for brightness struct.
static inline size_t store_fn(const enum fs_type_e type, const char* buf, const size_t count) {
    int val;
    if(!kstrtoint(buf, 10, &val)) {
        switch(type) {
            case FSTYPE_SCHEDULE_BRIGHTNESS:
                schedule_brightness_steps_fn_async((unsigned short)val);
                schedule_delayed_dimm_fn_async(timer_dimm_delay.first);
                // brightness_store_steps(val);
                break;
            case FSTYPE_MIN_BRIGHTNESS:
                brightness.min = (unsigned short)val;
                break;
            case FSTYPE_MAX_BRIGHTNESS:
                brightness.max = (unsigned short)val;
                break;
            case FSTYPE_DEFAULT_BRIGHTNESS:
                brightness.default_brightness = (unsigned short)val;
                break;
            case FSTYPE_DELAY_BRIGHTNESS:
                brightness.delay = (unsigned short)val;
                break;
            case FSTYPE_STEPS_BRIGHTNESS:
                brightness.steps = (unsigned char)val;
                break;
            case FSTYPE_TIMER_DELAY_FIRST:
                timer_dimm_delay.first = (unsigned int)val;
                break;
            case FSTYPE_TIMER_DELAY_SECOND:
                timer_dimm_delay.second = (unsigned int)val;
                break;
        }
    }

    return count;
}

/// @brief FN: Resets key check flags for the brightness construct. 
static void reset_check_key_flags(void) {
    brightness.check_flags &= ~(
        CHECK_KEY_STUCK
            | (CHECK_KEY_STUCK << KBDILLUMUP)
            | (CHECK_KEY_STUCK << KBDILLUMDOWN)
    );
}

/// @brief FN: Checks if brighness is 'locked an loaded'.
///
/// Sets check flags and calls brightness function to fade keyboard backlight brightness value to 0.
static void brightness_unload(void) {
    if(FLAGS_LOWER_NIBBLE(brightness.check_flags) == CHECK_OK) {
        PR_INFO("Unloading brightness ..");
        
        brightness.check_flags |= CHECK_UNLOAD;
        if(schedule_brightness_store_steps_work) {
            // Schedule work
            schedule_brightness_store_steps_work->val = 0;
            schedule_work(&schedule_brightness_store_steps_work->steps_work);
        }
        // brightness_store_steps(0);

        PR_DONE();
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

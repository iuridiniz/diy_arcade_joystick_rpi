//#define DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/circ_buf.h>
#include <linux/input.h>


/*
https://github.com/recalbox/mk_arcade_joystick_rpi
https://www.raspberrypi.org/documentation/usage/gpio/
https://www.disk91.com/2015/technology/systems/rf433-raspberry-pi-gpio-kernel-driver-for-interrupt-management/
https://bitbucket.org/disk_91-admin/rfrpi/src/625129fecef974059a4f8f4353aa31e1ca48b8c4/rfrpi_src/krfrpi/krfrpi.c?at=master
https://www.kernel.org/doc/htmldocs/kernel-api/
https://www.kernel.org/doc/html/latest/driver-api/gpio/consumer.html?highlight=gpio
https://docs.huihoo.com/doxygen/linux/kernel/3.7/drivers_2gpio_2gpiolib_8c.html#adbee45b45d890e04dc3b7f642d2dddba
https://www.linuxquestions.org/questions/programming-9/correct-circular-buffer-api-usage-4175644449/
https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html
https://lwn.net/Articles/302043/
https://docs.huihoo.com/doxygen/linux/kernel/3.7/kernel_2irq_2manage_8c.html

https://lwn.net/Articles/577608/

http://www.cs.fsu.edu/~cop4610t/lectures/project2/kthreads/kthreads.pdf
https://lwn.net/Articles/65178/

Button layout (retroarch)

          ⨂ ⨁        [start / select]
     ↑    Ⓧ Ⓨ Ⓛ     [left analog up / X / Y / L shoulder]
   ←   →  Ⓐ Ⓑ Ⓡ     [left analog left / left analog right / A / B / R shoulder]
     ↓                [left analog down]

*/


#define RPI_MIN_GPIO_NUMBER (4)
#define RPI_MAX_GPIO_NUMBER (27)

/* GPIO high/low variations inside this time window (in μs - microsecond) will be ignored */
#define EVENT_THRESHOLD (5000) /* 5ms */

/* How much time thread to process events will sleep if there's no event */
#define PROCESS_EVENTS_SLEEP_TIME (HZ/100) /* 1ms */
//#define PROCESS_EVENTS_SLEEP_TIME (HZ/1) /* 1 second */

#define BUFFER_SIZE (32) /* max number of events to store on buffer */
#define MODULE_MARK "diy_arcade_joystick_rpi: " /* printk prefix */

struct state_data {
    struct gpio* gpio;
    u8 index;
    u8 last;
    u16 ignored;
    u64 ignored_total;
    struct timespec last_time;
};

struct event_data {
    u8 key;
    u8 state;
    const char* key_name;
};

struct thread_pad_data {
    struct circ_buf* events;
    struct input_dev* dev;
    const char* phys;
    char* pad_name;
    u8 pad_number;
    //const short btns[BTNS_ARRAY_SIZE];
};

static const short input_dev_keys[] = {BTN_START, BTN_SELECT, BTN_A, BTN_B, BTN_TR, BTN_Y, BTN_X, BTN_TL};

enum {
    GPIO_P1_UP            = 4,    /* PIN 07 of RPI 2+ */
    GPIO_P1_DOWN          = 17,   /* PIN 11 of RPI 2+ */
    GPIO_P1_LEFT          = 27,   /* PIN 13 of RPI 2+ */
    GPIO_P1_RIGHT         = 22,   /* PIN 15 of RPI 2+ */
    GPIO_P1_BUTTON_L      = 14,   /* PIN 08 of RPI 2+ */
    GPIO_P1_BUTTON_X      = 15,   /* PIN 10 of RPI 2+ */
    GPIO_P1_BUTTON_Y      = 18,   /* PIN 12 of RPI 2+ */
    GPIO_P1_BUTTON_R      = 23,   /* PIN 16 of RPI 2+ */
    GPIO_P1_BUTTON_A      = 25,   /* PIN 22 of RPI 2+ */
    GPIO_P1_BUTTON_B      = 24,   /* PIN 18 of RPI 2+ */
    GPIO_P1_BUTTON_START  = 10,   /* PIN 19 of RPI 2+ */
    GPIO_P1_BUTTON_SELECT = 9,    /* PIN 21 of RPI 2+ */

    GPIO_P2_UP            = 11,   /* PIN 23 of RPI 2+ */
    GPIO_P2_DOWN          = 5,    /* PIN 29 of RPI 2+ */
    GPIO_P2_LEFT          = 6,    /* PIN 31 of RPI 2+ */
    GPIO_P2_RIGHT         = 13,   /* PIN 33 of RPI 2+ */
    GPIO_P2_BUTTON_L      = 8,    /* PIN 24 of RPI 2+ */
    GPIO_P2_BUTTON_X      = 7,    /* PIN 26 of RPI 2+ */
    GPIO_P2_BUTTON_Y      = 12,   /* PIN 32 of RPI 2+ */
    GPIO_P2_BUTTON_R      = 16,   /* PIN 36 of RPI 2+ */
    GPIO_P2_BUTTON_A      = 21,   /* PIN 40 of RPI 2+ */
    GPIO_P2_BUTTON_B      = 20,   /* PIN 38 of RPI 2+ */
    GPIO_P2_BUTTON_START  = 19,   /* PIN 35 of RPI 2+ */
    GPIO_P2_BUTTON_SELECT = 26,   /* PIN 37 of RPI 2+ */
};


static struct gpio _gpio_keys[] = {
    { GPIO_P1_UP,            GPIOF_IN, "P1 ↑ - left analog up"},
    { GPIO_P1_DOWN,          GPIOF_IN, "P1 ↓ - left analog down"},
    { GPIO_P1_LEFT,          GPIOF_IN, "P1 ← - left analog left"},
    { GPIO_P1_RIGHT,         GPIOF_IN, "P1 → - left analog right"},
    { GPIO_P1_BUTTON_L,      GPIOF_IN, "P1 Ⓛ - button L"},
    { GPIO_P1_BUTTON_X,      GPIOF_IN, "P1 Ⓧ - button X"},
    { GPIO_P1_BUTTON_Y,      GPIOF_IN, "P1 Ⓨ - button Y"},
    { GPIO_P1_BUTTON_R,      GPIOF_IN, "P1 Ⓡ - button R"},
    { GPIO_P1_BUTTON_A,      GPIOF_IN, "P1 Ⓐ - button A"},
    { GPIO_P1_BUTTON_B,      GPIOF_IN, "P1 Ⓑ - button B"},
    { GPIO_P1_BUTTON_START,  GPIOF_IN, "P1 ⨂ - button start"},
    { GPIO_P1_BUTTON_SELECT, GPIOF_IN, "P1 ⨁ - button select"},

    { GPIO_P2_UP,            GPIOF_IN, "P2 ↑ - left analog up"},
    { GPIO_P2_DOWN,          GPIOF_IN, "P2 ↓ - left analog down"},
    { GPIO_P2_LEFT,          GPIOF_IN, "P2 ← - left analog left"},
    { GPIO_P2_RIGHT,         GPIOF_IN, "P2 → - left analog right"},
    { GPIO_P2_BUTTON_L,      GPIOF_IN, "P2 Ⓛ - button L"},
    { GPIO_P2_BUTTON_X,      GPIOF_IN, "P2 Ⓧ - button X"},
    { GPIO_P2_BUTTON_Y,      GPIOF_IN, "P2 Ⓨ - button Y"},
    { GPIO_P2_BUTTON_R,      GPIOF_IN, "P2 Ⓡ - button R"},
    { GPIO_P2_BUTTON_A,      GPIOF_IN, "P2 Ⓐ - button A"},
    { GPIO_P2_BUTTON_B,      GPIOF_IN, "P2 Ⓑ - button B"},
    { GPIO_P2_BUTTON_START,  GPIOF_IN, "P2 ⨂ - button start"},
    { GPIO_P2_BUTTON_SELECT, GPIOF_IN, "P2 ⨁ - button select"},
};

static struct state_data pad_keys[] = {
    { &_gpio_keys[0], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[1], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[2], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[3], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[4], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[5], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[6], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[7], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[8], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[9], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[10], 0, 1, 0, 0, {0, 0}},
    { &_gpio_keys[11], 0, 1, 0, 0, {0, 0}},

    { &_gpio_keys[12], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[13], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[14], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[15], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[16], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[17], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[18], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[19], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[20], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[21], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[22], 1, 1, 0, 0, {0, 0}},
    { &_gpio_keys[23], 1, 1, 0, 0, {0, 0}},
};

static struct circ_buf p1_events = {
    .buf = NULL,
    .head = 0,
    .tail = 0,
};

static struct circ_buf p2_events = {
    .buf = NULL,
    .head = 0,
    .tail = 0,
};


static struct task_struct* thread_p1 = NULL;
static struct task_struct* thread_p2 = NULL;
static struct thread_pad_data thread_p1_data = {
    .events = &p1_events,
    .dev = NULL,
    //.btns = BTNS_ARRAY,
    .phys = "input0",
    .pad_name = "GPIOArcade PAD1",
    .pad_number = 1
};
static struct thread_pad_data thread_p2_data = {
    .events = &p2_events,
    .dev = NULL,
    //.btns = BTNS_ARRAY,
    .phys = "input1",
    .pad_name = "GPIOArcade PAD2",
    .pad_number = 2
};

static void fire_key(struct input_dev* dev, struct event_data* ev) {
    // start    , select    , a    ,     b,     tr,     y,     x,     tl
    // BTN_START, BTN_SELECT, BTN_A, BTN_B, BTN_TR, BTN_Y, BTN_X, BTN_TL

    int is_abs = 0;
    int button = -1;
    int abs = -1;
    int value = ev->state;
    switch(ev->key) {
        case GPIO_P1_UP:
        case GPIO_P2_UP:
            is_abs = 1;
            abs = ABS_Y;
            if (ev->state == 1) {
                value = -1;
            } else {
                value = 0;
            }
            break;
        case GPIO_P1_DOWN:
        case GPIO_P2_DOWN:
            is_abs = 1;
            abs = ABS_Y;
            if (ev->state == 1) {
                value = +1;
            } else {
                value = 0;
            }
            break;
        case GPIO_P1_LEFT:
        case GPIO_P2_LEFT:
            is_abs = 1;
            abs = ABS_X;
            if (ev->state == 1) {
                value = -1;
            } else {
                value = 0;
            }
            break;
        case GPIO_P1_RIGHT:
        case GPIO_P2_RIGHT:
            is_abs = 1;
            abs = ABS_X;
            if (ev->state == 1) {
                value = +1;
            } else {
                value = 0;
            }
            break;
        case GPIO_P1_BUTTON_A:
        case GPIO_P2_BUTTON_A:
            button = BTN_A;
            break;
        case GPIO_P1_BUTTON_B:
        case GPIO_P2_BUTTON_B:
            button = BTN_B;
            break;
        case GPIO_P1_BUTTON_X:
        case GPIO_P2_BUTTON_X:
            button = BTN_X;
            break;
        case GPIO_P1_BUTTON_Y:
        case GPIO_P2_BUTTON_Y:
            button = BTN_Y;
            break;
        case GPIO_P1_BUTTON_L:
        case GPIO_P2_BUTTON_L:
            button = BTN_TL;
            break;
        case GPIO_P1_BUTTON_R:
        case GPIO_P2_BUTTON_R:
            button = BTN_TR;
            break;
        case GPIO_P1_BUTTON_SELECT:
        case GPIO_P2_BUTTON_SELECT:
            button = BTN_SELECT;
            break;
        case GPIO_P1_BUTTON_START:
        case GPIO_P2_BUTTON_START:
            button = BTN_START;
            break;
        default:
            BUG();
    }
    if (is_abs) {
        input_report_abs(dev, abs, value);
    } else {
        input_report_key(dev, button, value);
    }
}

static int thread_pad(void *data) {
    struct thread_pad_data* pad = (struct thread_pad_data*) data;
    struct event_data cur_ev = { .key = 0, .state = 0};
    pr_debug("pad/%d: thread '%s' started\n", pad->pad_number, pad->pad_name);
    while (!kthread_should_stop()) {
        //pr_debug("pad/%d: thread '%s' running\n", pad->pad_number, pad->pad_name);
        /* consume event */
        int head = smp_load_acquire(&pad->events->head);
        int tail = pad->events->tail;
        struct event_data* events = (struct event_data*) pad->events->buf;
        if (CIRC_CNT(head, tail, BUFFER_SIZE) >= 1) {
            struct event_data _ev = events[tail];
            memcpy(&cur_ev, &_ev, sizeof(struct event_data));
            smp_store_release(&pad->events->tail, (tail + 1) & (BUFFER_SIZE - 1));

            pr_debug("pad/%d: key '%s' (%d) is %d\n", pad->pad_number, cur_ev.key_name, cur_ev.key, cur_ev.state);
            fire_key(pad->dev, &cur_ev);
            continue;
        }
        input_sync(pad->dev);
        //pr_debug("pad/%d: thread '%s' running ... no events\n", pad->pad_number, pad->pad_name);
        schedule_timeout_interruptible(PROCESS_EVENTS_SLEEP_TIME);
    }
    pr_debug("pad/%d: thread '%s' stopped\n", pad->pad_number, pad->pad_name);
    return 0;
}

static irqreturn_t _gpio_hard_isr(int irq, void *data)
{

    struct state_data* pad_key = (struct state_data*) data;
    struct circ_buf* buf;
    static struct task_struct* consumer;
    struct event_data* events;

   	struct timespec current_time;
    struct timespec delta;
    unsigned long long delta_microssec;

    int head;
    int tail;

    u8 cur_value = (u8) gpio_get_value(pad_key->gpio->gpio);

    getnstimeofday(&current_time);
    delta = timespec_sub(current_time, pad_key->last_time);
    delta_microssec = ((unsigned long long)delta.tv_sec * 1000000)+(delta.tv_nsec/1000);

    /* key not changed ignore this event */
    if (cur_value == pad_key->last) {
        pad_key->ignored += 1;
        pad_key->ignored_total += 1;
        //pr_debug("'%s' is %d (was %d) --> IGNORING (UNCHANGED)\n", pad_key->gpio->label, cur_value, pad_key->last);
        return IRQ_HANDLED;
    }

    /* When a key is pressed, GPIO set it to low and become unstable (high/low) for some microsseconds, */
    /* ignoring theese events. */
    if ((delta.tv_sec == 0 && delta_microssec < (EVENT_THRESHOLD))){
        pad_key->ignored += 1;
        pad_key->ignored_total += 1;
        //pr_debug("'%s' is %d (was %d) --> IGNORING (FAST, delta: %04llu μs)\n", pad_key->gpio->label, cur_value, pad_key->last, delta_microssec);

        /* save state */
        pad_key->last = cur_value;
        memcpy(&pad_key->last_time, &current_time, sizeof(struct timespec));

        return IRQ_HANDLED;
    }

    //pr_debug("'%s' is %d (was %d)\n", pad_key->gpio->label, cur_value, pad_key->last);

    /* reset  */
    pad_key->ignored = 0;
    pad_key->last = cur_value;
    memcpy(&pad_key->last_time, &current_time, sizeof(struct timespec));

    /* which gamepad ? */
    if (pad_key->index == 0) {
        buf = &p1_events;
        consumer = thread_p1;
    } else if (pad_key->index == 1) {
        buf = &p2_events;
        consumer = thread_p2;
    } else {
        BUG();
    }

    /* produce event */
    head = buf->head;
    tail = READ_ONCE(buf->tail);
    events = (struct event_data*) buf->buf;
    if (CIRC_SPACE(head, tail, BUFFER_SIZE) >= 1) {
        struct event_data* ev = &events[head];
        ev->key = pad_key->gpio->gpio;
        ev->state = ! cur_value; /* for GPIO's, high is key up; for keys, high is key down */
        ev->key_name = pad_key->gpio->label;
        smp_store_release(&buf->head, (head + 1) & (BUFFER_SIZE - 1));

        /* wake up consumer */
        wake_up_process(consumer);
    } else {
        pr_err("Buffer events overflow on pad %d\n", pad_key->index);
    }

    return IRQ_HANDLED;
}

static int _pad_open(struct input_dev *dev) {
    return 0;
}

static void _pad_close(struct input_dev *dev) {
}

static void __exit _unconfigure_pad(struct thread_pad_data* pad) {
    if (pad->dev) {
        input_unregister_device(pad->dev);
    }
}

static int __init _configure_pad(struct thread_pad_data* pad) {
    struct input_dev* input_dev;
    int ret = 0;
    int i = 0;

    pr_debug("Allocating input_dev for pad %s\n", pad->pad_name);
    pad->dev = input_dev = input_allocate_device();
    
    if (input_dev == NULL) {
        pr_err("Cannot allocate input_dev for pad %s\n", pad->pad_name);
        ret = -ENOMEM;
        goto fail_input_allocate_device;
    }

    input_dev->name = pad->pad_name;
    input_dev->phys = pad->phys;
    input_dev->id.bustype = BUS_PARPORT;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0001;
    

    input_dev->open = _pad_open;
    input_dev->close = _pad_close;
    
    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

    for (i = 0; i < 2; i++) {
        input_set_abs_params(input_dev, ABS_X + i, -1, 1, 0, 0);
    }
    for (i = 0; i < ARRAY_SIZE(input_dev_keys); i++) {
         __set_bit(input_dev_keys[i], input_dev->keybit);
    }

    input_set_drvdata(input_dev, pad);

    pr_debug("Registering device for pad %s\n", pad->pad_name);
    ret = input_register_device(pad->dev);

    if (ret) {
        pr_err("Cannot register input_dev for pad %s\n", pad->pad_name);
        goto fail_input_register_device;
    }

    goto success;

fail_input_register_device:
    input_free_device(pad->dev);

fail_input_allocate_device:
success:
    return ret;
}

/* 
   I don't know how to reset GPIO 14 (PIN 8) aka TxD (Serial),
   but this code (from mk_arcade_joystick_rpi) reset that pin
   --> https://github.com/recalbox/mk_arcade_joystick_rpi/
*/
static void __init _gpio_reset(void) {
    #define GPIO_BASE (0x3F000000 + 0x200000)
    static volatile unsigned *gpio;
    /* pointer for direct access to gpio */
    if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
        pr_err("GPIO io remap failed\n");
        return;
    }
    #define GPIO_SET_PULL_UPS(pullUps) \
    do { \
        *(gpio + 37) = 0x02; \
        udelay(10); \
        *(gpio + 38) = (pullUps); \
        udelay(10); \
        *(gpio + 37) = 0x00; \
        *(gpio + 38) = 0x00; \
    } while (0)

    /* Pull Up */
    GPIO_SET_PULL_UPS(0x0FFFFFFC); /* All pins except GPIO 1,2 */

    #undef GPIO_SET_PULL_UPS
    #undef GPIO_BASE

    iounmap(gpio);
}

static int __init diy_arcade_joystick_rpi_init(void) {
    int ret = 0;
    int i;
    pr_info("GPIO Arcade Joystick Driver loaded\n");
    /* reset gpio */
    pr_info("Reset GPIO's (Fix GPIO 14 as TxD)\n");
    _gpio_reset();
    /* alocate memory */
    pr_debug("Allocing memory for event buffers: %d bytes\n", sizeof(struct event_data) * BUFFER_SIZE * 2);
    p1_events.buf = kzalloc(sizeof(struct event_data) * BUFFER_SIZE, GFP_KERNEL);
    p2_events.buf = kzalloc(sizeof(struct event_data) * BUFFER_SIZE, GFP_KERNEL);
    if (p1_events.buf == NULL || p2_events.buf == NULL) {
        pr_err("Not enough memory\n");
        ret = -ENOMEM;
        goto fail_memory;
    }

    /*******************************/
    /* allocate pads input devices */
    pr_debug("Configuring PADs");
    ret = _configure_pad(&thread_p1_data);
    if (ret) {
        goto fail_configure_pad;
    }
    _configure_pad(&thread_p2_data);
    if (ret) {
        goto fail_configure_pad;
    }

    /*******************************/
    /* start thread's */
    pr_debug("Starting %d threads for process event buffers\n", 2);
    thread_p1 = kthread_run(thread_pad, &thread_p1_data, thread_p1_data.pad_name);
    thread_p2 = kthread_run(thread_pad, &thread_p2_data, thread_p2_data.pad_name);

    if (IS_ERR(thread_p1) || IS_ERR(thread_p2)) {
        pr_err("Cannot create threads\n");
        ret = -EINVAL;
        goto fail_kthread;
    }

    /*******************************/
    /* register GPIO's PIN in use */
    pr_debug("Requesting GPIOs for pad_keys\n");
    if ((ret = gpio_request_array(_gpio_keys, ARRAY_SIZE(_gpio_keys))) != 0) {
        pr_err("Unable to request GPIOs for pad_keys: %d\n", ret);
        goto fail_gpio_request_array;
    }

    /* register IRQ's to GPIO */
    for(i = 0; i < ARRAY_SIZE(pad_keys); i++) {
        int irq_num;
        struct state_data* pad_key = &pad_keys[i];

        pr_debug("Requesting IRQ for GPIO %d\n", pad_key->gpio->gpio);
        irq_num = ret = gpio_to_irq(pad_key->gpio->gpio);
        if(ret < 0) {
            pr_err("Unable to map GPIO %d to IRQ: %d\n", pad_key->gpio->gpio, ret);
            goto fail_gpio_to_irq;
        }
        pr_debug("GPIO %d has IRQ %d\n", pad_key->gpio->gpio, irq_num);
        ret = request_threaded_irq(irq_num, 
                         _gpio_hard_isr, 
                         NULL,
                         IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NOBALANCING,
                         pad_key->gpio->label,
                         pad_key);
        if(ret < 0) {
            pr_err("Unable to resquest IRQ %d for GPIO %d: %d\n", irq_num, pad_key->gpio->gpio, ret);
            goto fail_gpio_to_irq;
        }
    }

    ret = 0;
    goto success;

fail_gpio_to_irq:
    while(i--) {
        int irq_num;
        struct state_data* pad_key = &pad_keys[i];
        irq_num = gpio_to_irq(pad_key->gpio->gpio);
        pr_debug("Freeing IRQ %d for GPIO %d\n", irq_num, pad_key->gpio->gpio);
        free_irq(irq_num, pad_key);
    }
    pr_debug("Freeing GPIOs for pad_keys\n");
    gpio_free_array(_gpio_keys, ARRAY_SIZE(_gpio_keys));

fail_gpio_request_array:
fail_kthread:
    if (!IS_ERR(thread_p1)) {
        kthread_stop(thread_p1);
    } 
    
    if (!IS_ERR(thread_p2)) {
        kthread_stop(thread_p2);
    }

fail_configure_pad:
    _unconfigure_pad(&thread_p1_data);
    _unconfigure_pad(&thread_p2_data);
fail_memory:
    if (p1_events.buf != NULL) {
        kfree(p1_events.buf);
        p1_events.buf = NULL;
    }
    if (p2_events.buf != NULL) {
        kfree(p2_events.buf);
        p2_events.buf = NULL;
    }
success:
    return ret;

}

static void __exit diy_arcade_joystick_rpi_exit(void) {
    int i = ARRAY_SIZE(pad_keys);
    while(i--) {
        int irq_num;
        struct state_data* pad_key = &pad_keys[i];
        irq_num = gpio_to_irq(pad_key->gpio->gpio);
        pr_debug("Freeing IRQ %d for GPIO %d\n", irq_num, pad_key->gpio->gpio);
        free_irq(irq_num, pad_key);
    }
    pr_debug("Freeing GPIOs for pad_keys\n");
    gpio_free_array(_gpio_keys, ARRAY_SIZE(_gpio_keys));

    pr_debug("Stopping threads\n");
    if (!IS_ERR(thread_p1)) {
        kthread_stop(thread_p1);
    } 

    if (!IS_ERR(thread_p2)) {
        kthread_stop(thread_p2);
    }

    pr_debug("Unconfiguring PADs");
    _unconfigure_pad(&thread_p1_data);
    _unconfigure_pad(&thread_p2_data);

    pr_debug("Freeing event buffers\n");
    if (p1_events.buf != NULL) {
        kfree(p1_events.buf);
        p1_events.buf = NULL;
    }
    if (p2_events.buf != NULL) {
        kfree(p2_events.buf);
        p2_events.buf = NULL;
    }
    pr_info("GPIO Arcade Joystick Driver unloaded\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Iuri Diniz <iuridiniz@gmail.com>");
MODULE_DESCRIPTION("GPIO Arcade Joystick Driver");

module_init(diy_arcade_joystick_rpi_init);
module_exit(diy_arcade_joystick_rpi_exit);

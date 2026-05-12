#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

extern uint8_t zmk_battery_state_of_charge(void);

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PIN_USB_DETECT 29 // P0.29
#define PIN_CHG_STATUS 30 // P0.30
#define PIN_BUTTON     5  // P0.05
#define PIN_LED_25     14 // P1.14
#define PIN_LED_50     12 // P1.12
#define PIN_LED_75     25 // P0.25

static const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

static bool blink_state = false;

static void system_work_handler(struct k_work *work) {
    if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) return;

    // Read battery percentage from ZMK native tracker
    uint8_t percentage = zmk_battery_state_of_charge();

    // Read digital pins
    int usb_connected = (gpio_pin_get_raw(gpio0, PIN_USB_DETECT) == 0);
    int fully_charged = (gpio_pin_get_raw(gpio0, PIN_CHG_STATUS) == 0);
    int button_pressed = (gpio_pin_get_raw(gpio0, PIN_BUTTON) == 1);

    if (button_pressed) {
        LOG_INF("BUTTON IS ON");
    }

    blink_state = !blink_state;

    // Default to turning all LEDs off
    int l25 = 0, l50 = 0, l75 = 0;

    if (usb_connected) {
        if (fully_charged) {
            l25 = 1; l50 = 1; l75 = 1;
        } else if (percentage >= 75) {
            l25 = 1; l50 = 1; l75 = blink_state;
        } else if (percentage >= 50) {
            l25 = 1; l50 = blink_state; l75 = 0;
        } else {
            l25 = blink_state; l50 = 0; l75 = 0;
        }
    } else {
        if (percentage >= 75) {
            l25 = 1; l50 = 1; l75 = 1;
        } else if (percentage >= 50) {
            l25 = 1; l50 = 1; l75 = 0;
        } else if (percentage >= 25) {
            l25 = 1; l50 = 0; l75 = 0;
        } else {
            l25 = 0; l50 = 0; l75 = 0;
        }
    }

    gpio_pin_set_raw(gpio1, PIN_LED_25, l25);
    gpio_pin_set_raw(gpio1, PIN_LED_50, l50);
    gpio_pin_set_raw(gpio0, PIN_LED_75, l75);
}

K_WORK_DEFINE(system_work, system_work_handler);

static void system_timer_handler(struct k_timer *timer) {
    k_work_submit(&system_work);
}

K_TIMER_DEFINE(system_timer, system_timer_handler, NULL);

static int holyiot_system_init(void) {
    if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) {
        LOG_ERR("Failed to get GPIO devices");
        return -ENODEV;
    }

    // Configure inputs
    gpio_pin_configure(gpio0, PIN_USB_DETECT, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpio0, PIN_CHG_STATUS, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpio0, PIN_BUTTON, GPIO_INPUT | GPIO_PULL_DOWN);

    // Configure outputs
    gpio_pin_configure(gpio1, PIN_LED_25, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, PIN_LED_50, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_LED_75, GPIO_OUTPUT_INACTIVE);

    // Start timer (every 500ms)
    k_timer_start(&system_timer, K_MSEC(500), K_MSEC(500));

    return 0;
}

SYS_INIT(holyiot_system_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

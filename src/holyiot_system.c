#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>

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

static bool blink_state   = false;
static bool system_is_off = false;

/* ── BT connection callback: kill any new connection immediately when OFF ── */
static void on_bt_connected(struct bt_conn *conn, uint8_t err) {
    if (err) return;
    if (system_is_off) {
        LOG_INF("Switch OFF — rejecting BT connection");
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

BT_CONN_CB_DEFINE(holyiot_conn_cb) = {
    .connected = on_bt_connected,
};

/* Drop every currently active BLE connection */
static void drop_all_connections(struct bt_conn *conn, void *data) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 &&
        info.state == BT_CONN_STATE_CONNECTED) {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

/* Called every 200ms tick while switch is OFF */
static void suppress_bluetooth(void) {
    bt_le_adv_stop();
    bt_conn_foreach(BT_CONN_TYPE_LE, drop_all_connections, NULL);
}

/* Restart connectable advertising so host can reconnect */
static void restart_bluetooth_adv(void) {
    static const struct bt_le_adv_param adv_param =
        BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
            BT_GAP_ADV_FAST_INT_MIN_2,
            BT_GAP_ADV_FAST_INT_MAX_2,
            NULL);

    int ret = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);
    if (ret < 0 && ret != -EALREADY) {
        LOG_WRN("Failed to restart BLE advertising: %d", ret);
    }
}

static void set_input_devices(enum pm_device_action action) {
    const struct device *kscan = DEVICE_DT_GET(DT_CHOSEN(zmk_kscan));
    if (device_is_ready(kscan)) {
        pm_device_action_run(kscan, action);
    }
    const struct device *trackpad = DEVICE_DT_GET(DT_NODELABEL(trackpad));
    if (device_is_ready(trackpad)) {
        pm_device_action_run(trackpad, action);
    }
}

static void on_power_off(void) {
    LOG_INF("Soft Power Switch: OFF");
    system_is_off = true;
    set_input_devices(PM_DEVICE_ACTION_SUSPEND);
    suppress_bluetooth();
}

static void on_power_on(void) {
    LOG_INF("Soft Power Switch: ON");
    system_is_off = false;
    set_input_devices(PM_DEVICE_ACTION_RESUME);
    restart_bluetooth_adv();
}

static void system_work_handler(struct k_work *work) {
    if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) return;

    int power_switch_off = (gpio_pin_get_raw(gpio0, PIN_BUTTON) == 1);

    if (power_switch_off && !system_is_off) {
        on_power_off();
    } else if (!power_switch_off && system_is_off) {
        on_power_on();
    } else if (power_switch_off && system_is_off) {
        /*
         * Switch still OFF — keep suppressing BLE every 200ms.
         * The bt_conn_cb will also kill any connection that sneaks through.
         */
        suppress_bluetooth();
    }

    /* ── Battery + LED logic (always runs) ── */

    uint8_t percentage = zmk_battery_state_of_charge();
    int usb_connected  = (gpio_pin_get_raw(gpio0, PIN_USB_DETECT) == 0);
    int fully_charged  = (gpio_pin_get_raw(gpio0, PIN_CHG_STATUS) == 0);

    blink_state = !blink_state;

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

    gpio_pin_configure(gpio0, PIN_USB_DETECT, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpio0, PIN_CHG_STATUS, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpio0, PIN_BUTTON,     GPIO_INPUT | GPIO_PULL_DOWN);

    gpio_pin_configure(gpio1, PIN_LED_25, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, PIN_LED_50, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_LED_75, GPIO_OUTPUT_INACTIVE);

    /* 200ms timer — fast enough to outrun ZMK's BLE restart attempts */
    k_timer_start(&system_timer, K_MSEC(200), K_MSEC(200));

    return 0;
}

SYS_INIT(holyiot_system_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * USB command + IMU stream + dual PWM motor for XIAO nRF52840 Sense Plus.
 *
 * Commands (case-insensitive):
 *   L on [%]   - left motor ON at duty cycle (0-100), default 100
 *   L off      - left motor OFF
 *   R on [%]   - right motor ON at duty cycle (0-100), default 100
 *   R off      - right motor OFF
 *   on [%]     - both motors ON
 *   off        - both motors OFF
 *   status     - motors + cal + led status
 *   cal        - calibrate IMU (10s, keep device still)
 *   cal off    - disable calibration, use raw values
 *   led on     - enable motor indicator LEDs (G=L, B=R)
 *   led off    - disable indicator LEDs
 *   help       - print commands
 *
 * L = D0 (P0.02), R = D1 (P0.03)
 * Red LED = heartbeat (500µs on / 2.5s off)
 * Green LED = L motor indicator
 * Blue LED = R motor indicator
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#define LED_R_NODE   DT_ALIAS(led0)
#define LED_G_NODE   DT_ALIAS(led1)
#define LED_B_NODE   DT_ALIAS(led2)
#define MOTOR_L_NODE DT_ALIAS(motor_left)
#define MOTOR_R_NODE DT_ALIAS(motor_right)
#define IMU_NODE     DT_NODELABEL(lsm6ds3tr_c)

#define CMD_BUF_SIZE    64
#define RX_RING_SIZE    128
#define IMU_PRINT_MS    10
#define RED_ON_MS       100
#define RED_OFF_MS      1400
#define CAL_DURATION_MS 10000
#define GRAVITY_STD     9.80665f

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(LED_G_NODE, gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(LED_B_NODE, gpios);
static const struct pwm_dt_spec motor_l = PWM_DT_SPEC_GET(MOTOR_L_NODE);
static const struct pwm_dt_spec motor_r = PWM_DT_SPEC_GET(MOTOR_R_NODE);
static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#if DT_NODE_EXISTS(IMU_NODE)
static const struct device *const imu_dev = DEVICE_DT_GET(IMU_NODE);
static bool imu_ready;
#endif

static uint8_t duty_l;
static uint8_t duty_r;
static bool led_indicate = true;

RING_BUF_DECLARE(rx_ring, RX_RING_SIZE);

/* ---- LED indicators ---- */
static void update_indicator_leds(void)
{
	if (led_indicate) {
		gpio_pin_set_dt(&led_g, duty_l > 0 ? 1 : 0);
		gpio_pin_set_dt(&led_b, duty_r > 0 ? 1 : 0);
	} else {
		gpio_pin_set_dt(&led_g, 0);
		gpio_pin_set_dt(&led_b, 0);
	}
}

/* ---- Calibration ---- */
enum cal_state { CAL_OFF, CAL_COLLECTING, CAL_ACTIVE };

static enum cal_state cal_state;
static int64_t cal_start_time;
static float cal_acc_sum[3];
static float cal_gyr_sum[3];
static uint32_t cal_n;

static float gyr_bias[3];
static float acc_scale = 1.0f;

static float sv_to_f(const struct sensor_value *sv)
{
	return (float)sv->val1 + (float)sv->val2 * 1e-6f;
}

static void cal_start(void)
{
	cal_state = CAL_COLLECTING;
	cal_start_time = k_uptime_get();
	cal_acc_sum[0] = 0; cal_acc_sum[1] = 0; cal_acc_sum[2] = 0;
	cal_gyr_sum[0] = 0; cal_gyr_sum[1] = 0; cal_gyr_sum[2] = 0;
	cal_n = 0;
	printk(">>> CAL started — keep device still for 10 seconds...\n");
}

static void cal_finalize(void)
{
	float n = (float)cal_n;

	for (int i = 0; i < 3; i++) {
		gyr_bias[i] = cal_gyr_sum[i] / n;
	}

	float avg_a[3];

	for (int i = 0; i < 3; i++) {
		avg_a[i] = cal_acc_sum[i] / n;
	}

	float mag = sqrtf(avg_a[0] * avg_a[0] +
			  avg_a[1] * avg_a[1] +
			  avg_a[2] * avg_a[2]);

	acc_scale = (mag > 0.1f) ? (GRAVITY_STD / mag) : 1.0f;

	cal_state = CAL_ACTIVE;
	printk(">>> CAL done: %u samples over 10s\n", cal_n);
	printk(">>> GYR bias: %f  %f  %f\n",
	       (double)gyr_bias[0], (double)gyr_bias[1], (double)gyr_bias[2]);
	printk(">>> ACC scale: %f (raw mag=%f, target=%f)\n",
	       (double)acc_scale, (double)mag, (double)GRAVITY_STD);
}

static void cal_reset(void)
{
	cal_state = CAL_OFF;
	gyr_bias[0] = 0; gyr_bias[1] = 0; gyr_bias[2] = 0;
	acc_scale = 1.0f;
	printk(">>> CAL off — using raw values\n");
}

/* ---- UART ISR ---- */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[32];
			int len = uart_fifo_read(dev, buf, sizeof(buf));

			if (len > 0) {
				ring_buf_put(&rx_ring, buf, len);
			}
		}
	}
}

/* ---- Motor control ---- */
static void motor_set(const struct pwm_dt_spec *pwm, uint8_t *cur_duty,
		      uint8_t pct, const char *name)
{
	if (pct > 100) {
		pct = 100;
	}
	*cur_duty = pct;

	uint32_t pulse = (pwm->period * (uint32_t)pct) / 100U;
	int ret = pwm_set_pulse_dt(pwm, pulse);

	printk(">>> OK %s=%u%% (ret=%d)\n", name, pct, ret);
	update_indicator_leds();
}

/* ---- Command parsing ---- */
static void normalize_cmd(char *cmd)
{
	for (size_t i = 0; cmd[i] != '\0'; i++) {
		cmd[i] = (char)tolower((unsigned char)cmd[i]);
	}
	size_t len = strlen(cmd);

	while (len > 0 && cmd[len - 1] == ' ') {
		cmd[--len] = '\0';
	}
	if (cmd[0] == ' ') {
		char *p = cmd;

		while (*p == ' ') {
			p++;
		}
		memmove(cmd, p, strlen(p) + 1);
	}
}

static uint8_t parse_duty(const char *s)
{
	long val = strtol(s, NULL, 10);

	if (val < 0) {
		val = 0;
	}
	if (val > 100) {
		val = 100;
	}
	return (uint8_t)val;
}

static void handle_motor_cmd(const char *args, const struct pwm_dt_spec *pwm,
			     uint8_t *cur_duty, const char *name)
{
	while (*args == ' ') {
		args++;
	}

	if (strncmp(args, "on", 2) == 0) {
		uint8_t pct = 100;
		const char *rest = args + 2;

		if (*rest == ' ' || *rest == '\t') {
			pct = parse_duty(rest);
		} else if (*rest != '\0') {
			printk(">>> ERR unknown '%s' cmd '%s'\n", name, args);
			return;
		}
		motor_set(pwm, cur_duty, pct, name);
	} else if (strncmp(args, "off", 3) == 0) {
		motor_set(pwm, cur_duty, 0, name);
	} else if (strncmp(args, "status", 6) == 0) {
		printk(">>> STATUS %s=%u%%\n", name, *cur_duty);
	} else {
		printk(">>> ERR unknown '%s' cmd '%s'\n", name, args);
	}
}

static void process_cmd(char *cmd)
{
	normalize_cmd(cmd);
	if (cmd[0] == '\0') {
		return;
	}

	printk(">>> RX '%s'\n", cmd);

	if (strncmp(cmd, "cal", 3) == 0) {
		const char *rest = cmd + 3;

		while (*rest == ' ') {
			rest++;
		}
		if (strncmp(rest, "off", 3) == 0) {
			cal_reset();
		} else {
			cal_start();
		}
	} else if (strncmp(cmd, "led", 3) == 0) {
		const char *rest = cmd + 3;

		while (*rest == ' ') {
			rest++;
		}
		if (strncmp(rest, "off", 3) == 0) {
			led_indicate = false;
			update_indicator_leds();
			printk(">>> LED indicators OFF\n");
		} else {
			led_indicate = true;
			update_indicator_leds();
			printk(">>> LED indicators ON\n");
		}
	} else if (cmd[0] == 'l' && cmd[1] == ' ') {
		handle_motor_cmd(cmd + 2, &motor_l, &duty_l, "L");
	} else if (cmd[0] == 'r' && cmd[1] == ' ') {
		handle_motor_cmd(cmd + 2, &motor_r, &duty_r, "R");
	} else if (strncmp(cmd, "on", 2) == 0) {
		uint8_t pct = 100;
		const char *rest = cmd + 2;

		if (*rest == ' ' || *rest == '\t') {
			pct = parse_duty(rest);
		}
		motor_set(&motor_l, &duty_l, pct, "L");
		motor_set(&motor_r, &duty_r, pct, "R");
	} else if (strcmp(cmd, "off") == 0) {
		motor_set(&motor_l, &duty_l, 0, "L");
		motor_set(&motor_r, &duty_r, 0, "R");
	} else if (strcmp(cmd, "status") == 0) {
		printk(">>> STATUS L=%u%% R=%u%% CAL=%s LED=%s\n",
		       duty_l, duty_r,
		       cal_state == CAL_ACTIVE ? "ON" :
		       cal_state == CAL_COLLECTING ? "..." : "OFF",
		       led_indicate ? "ON" : "OFF");
	} else if (strcmp(cmd, "help") == 0) {
		printk(">>> Commands:\n");
		printk(">>>   L on [0-100]  - left motor (D0)\n");
		printk(">>>   L off\n");
		printk(">>>   R on [0-100]  - right motor (D1)\n");
		printk(">>>   R off\n");
		printk(">>>   on [0-100]    - both motors\n");
		printk(">>>   off           - both off\n");
		printk(">>>   status        - full status\n");
		printk(">>>   cal           - calibrate IMU (10s)\n");
		printk(">>>   cal off       - disable calibration\n");
		printk(">>>   led on/off    - motor indicator LEDs\n");
	} else {
		printk(">>> ERR unknown '%s'\n", cmd);
	}
}

/* ---- IMU ---- */
static void fetch_and_print_imu(void)
{
#if DT_NODE_EXISTS(IMU_NODE)
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int ret;

	if (!imu_ready) {
		return;
	}

	ret = sensor_sample_fetch(imu_dev);
	if (ret < 0) {
		printk("IMU fetch err=%d\n", ret);
		return;
	}

	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret < 0) {
		return;
	}

	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret < 0) {
		return;
	}

	float ax = sv_to_f(&accel[0]);
	float ay = sv_to_f(&accel[1]);
	float az = sv_to_f(&accel[2]);
	float gx = sv_to_f(&gyro[0]);
	float gy = sv_to_f(&gyro[1]);
	float gz = sv_to_f(&gyro[2]);

	if (cal_state == CAL_COLLECTING) {
		cal_acc_sum[0] += ax;
		cal_acc_sum[1] += ay;
		cal_acc_sum[2] += az;
		cal_gyr_sum[0] += gx;
		cal_gyr_sum[1] += gy;
		cal_gyr_sum[2] += gz;
		cal_n++;

		if ((k_uptime_get() - cal_start_time) >= CAL_DURATION_MS) {
			cal_finalize();
		}
	}

	if (cal_state == CAL_ACTIVE) {
		ax *= acc_scale;
		ay *= acc_scale;
		az *= acc_scale;
		gx -= gyr_bias[0];
		gy -= gyr_bias[1];
		gz -= gyr_bias[2];
	}

	if (cal_state == CAL_COLLECTING) {
		int sec_left = (int)((CAL_DURATION_MS -
				      (k_uptime_get() - cal_start_time)) / 1000);
		if (sec_left < 0) {
			sec_left = 0;
		}
		printk("ACC %f %f %f | GYR %f %f %f [CAL %ds]\n",
		       (double)ax, (double)ay, (double)az,
		       (double)gx, (double)gy, (double)gz, sec_left);
	} else {
		const char *tag = (cal_state == CAL_ACTIVE) ? " [CAL]" : "";

		printk("ACC %f %f %f | GYR %f %f %f%s\n",
		       (double)ax, (double)ay, (double)az,
		       (double)gx, (double)gy, (double)gz, tag);
	}
#endif
}

/* ---- main ---- */
int main(void)
{
	int ret;
	int64_t last_red_toggle = k_uptime_get();
	bool red_state = false;
	int64_t last_imu_print = k_uptime_get();
	char cmd_buf[CMD_BUF_SIZE];
	size_t cmd_len = 0U;

	if (!gpio_is_ready_dt(&led_r) ||
	    !gpio_is_ready_dt(&led_g) ||
	    !gpio_is_ready_dt(&led_b)) {
		printk("ERR: LED GPIO not ready\n");
		return 0;
	}
	if (!pwm_is_ready_dt(&motor_l) || !pwm_is_ready_dt(&motor_r)) {
		printk("ERR: Motor PWM not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);

	pwm_set_pulse_dt(&motor_l, 0);
	pwm_set_pulse_dt(&motor_r, 0);

#if DT_NODE_EXISTS(IMU_NODE)
	if (!device_is_ready(imu_dev)) {
		printk("WARN: IMU not ready\n");
		imu_ready = false;
	} else {
		imu_ready = true;
		printk("IMU: ready\n");
		ret = sensor_sample_fetch(imu_dev);
		printk("IMU: first fetch %s\n", ret < 0 ? "FAIL" : "OK");
	}
#else
	printk("WARN: IMU node not in devicetree\n");
#endif

	if (!device_is_ready(console_dev)) {
		printk("ERR: UART not ready\n");
		return 0;
	}
	uart_irq_callback_set(console_dev, uart_isr);
	uart_irq_rx_enable(console_dev);

	printk("=== READY (IMU %d Hz) ===\n", 1000 / IMU_PRINT_MS);
	printk("L=D0 R=D1 | L on 50 / R off / cal / led on / status / help\n");

	while (1) {
		uint8_t c;

		/* Red LED heartbeat: 600ms on / 400ms off */
		int64_t red_elapsed = k_uptime_get() - last_red_toggle;

		if ((red_state && red_elapsed >= RED_ON_MS) ||
		    (!red_state && red_elapsed >= RED_OFF_MS)) {
			red_state = !red_state;
			gpio_pin_set_dt(&led_r, red_state ? 1 : 0);
			last_red_toggle = k_uptime_get();
		}

		/* IMU read + print at 100 Hz */
		if ((k_uptime_get() - last_imu_print) >= IMU_PRINT_MS) {
			fetch_and_print_imu();
			last_imu_print = k_uptime_get();
		}

		/* Drain RX ring buffer */
		while (ring_buf_get(&rx_ring, &c, 1) == 1) {
			if (c == '\r' || c == '\n') {
				if (cmd_len > 0U) {
					cmd_buf[cmd_len] = '\0';
					process_cmd(cmd_buf);
					cmd_len = 0U;
				}
				continue;
			}
			if (cmd_len < (CMD_BUF_SIZE - 1U)) {
				cmd_buf[cmd_len++] = (char)c;
			} else {
				cmd_len = 0U;
				printk("ERR: cmd too long\n");
			}
		}

		k_msleep(1);
	}

	return 0;
}

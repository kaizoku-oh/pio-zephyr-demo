#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pio_zephyr_demo, LOG_LEVEL_DBG);

/* Blink delay in milliseconds */
#define SLEEP_TIME_MS (1000)

/* The devicetree node identifiers for the "led0" "led1" and "led2" aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

/* LED GPIO information specified in devicetree */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

void main(void)
{
  k_msleep(1000);
  LOG_INF("============================================");
  LOG_INF("========== PlatformIO Zephyr demo ==========");
  LOG_INF("============================================");

  gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);

  gpio_pin_toggle_dt(&led1);
  while(1)
  {
    gpio_pin_toggle_dt(&led0);
    gpio_pin_toggle_dt(&led1);
    gpio_pin_toggle_dt(&led2);

    k_msleep(SLEEP_TIME_MS);
  }
}

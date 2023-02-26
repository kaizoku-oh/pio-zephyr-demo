#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pio_zephyr_demo, LOG_LEVEL_DBG);

/* Blink delay in milliseconds */
#define BLINK_DELAY_MS (100)

/* The devicetree node identifiers for the "led0" "led1" and "led2" aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

/* LED GPIO information specified in devicetree */
static const struct gpio_dt_spec greenLED = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec blueLED = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec redLED = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

void main(void)
{
  k_msleep(1000);
  LOG_INF("============================================");
  LOG_INF("========== PlatformIO Zephyr demo ==========");
  LOG_INF("============================================");

  gpio_pin_configure_dt(&greenLED, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure_dt(&blueLED, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure_dt(&redLED, GPIO_OUTPUT_ACTIVE);

  while(1)
  {
    gpio_pin_toggle_dt(&greenLED);
    k_msleep(BLINK_DELAY_MS);
    gpio_pin_toggle_dt(&blueLED);
    k_msleep(BLINK_DELAY_MS);
    gpio_pin_toggle_dt(&redLED);
    k_msleep(BLINK_DELAY_MS);
  }
}

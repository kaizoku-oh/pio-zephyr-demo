/*-----------------------------------------------------------------------------------------------*/
/* Includes                                                                                      */
/*-----------------------------------------------------------------------------------------------*/
#include <zephyr.h>
#include <drivers/gpio.h>
#include <net/net_if.h>
#include <net/net_mgmt.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pio_zephyr_demo, LOG_LEVEL_DBG);

/*-----------------------------------------------------------------------------------------------*/
/* Defines                                                                                       */
/*-----------------------------------------------------------------------------------------------*/
/* Blink delay in milliseconds */
#define BLINK_DELAY_MS (100)
/* The devicetree node identifiers for the "led0" "led1" and "led2" aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
/* The devicetree node identifier for the "sw0" alias */
#define SW0_NODE DT_ALIAS(sw0)

/*-----------------------------------------------------------------------------------------------*/
/* Private function prototypes                                                                   */
/*-----------------------------------------------------------------------------------------------*/
/* Button callback function */
static void _button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
/* DHCP client callback function */
static void _dhcpv4_handler(struct net_mgmt_event_callback *, uint32_t, struct net_if *);

/*-----------------------------------------------------------------------------------------------*/
/* Private variables                                                                             */
/*-----------------------------------------------------------------------------------------------*/
/* LEDs GPIO information specified in devicetree */
static const struct gpio_dt_spec greenLED = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec blueLED = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec redLED = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
/* Button GPIO information specified in devicetree */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
/* GPIO callback structure */
static struct gpio_callback buttonCbData;
/* Network Management event callback structure */
static struct net_mgmt_event_callback netMgmtEvtCb;

/*-----------------------------------------------------------------------------------------------*/
/* Exported functions                                                                            */
/*-----------------------------------------------------------------------------------------------*/
void main(void)
{
  struct net_if *netIf;

  k_msleep(1000);
  LOG_INF("%s", "============================================");
  LOG_INF("%s", "========== PlatformIO Zephyr demo ==========");
  LOG_INF("%s", "============================================");

  /* Configure LEDs */
  gpio_pin_configure_dt(&greenLED, GPIO_OUTPUT_ACTIVE);
  LOG_INF("Setting up green LED at %s pin %d", greenLED.port->name, greenLED.pin);
  gpio_pin_configure_dt(&blueLED, GPIO_OUTPUT_ACTIVE);
  LOG_INF("Setting up blue LED at %s pin %d", blueLED.port->name, blueLED.pin);
  gpio_pin_configure_dt(&redLED, GPIO_OUTPUT_ACTIVE);
  LOG_INF("Setting up red LED at %s pin %d", redLED.port->name, redLED.pin);

  /* Configure button */
  gpio_pin_configure_dt(&button, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  gpio_init_callback(&buttonCbData, _button_pressed_cb, BIT(button.pin));
  gpio_add_callback(button.port, &buttonCbData);
  LOG_INF("Setting up button at %s pin %d", button.port->name, button.pin);

  /* Start DHCP client */
  net_mgmt_init_event_callback(&netMgmtEvtCb, _dhcpv4_handler, NET_EVENT_IPV4_ADDR_ADD);
  net_mgmt_add_event_callback(&netMgmtEvtCb);
  netIf = net_if_get_default();
  net_dhcpv4_start(netIf);

  /* Simulate LEDs racing effect */
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

/*-----------------------------------------------------------------------------------------------*/
/* Private functions                                                                             */
/*-----------------------------------------------------------------------------------------------*/
static void _button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  LOG_INF("Button pressed at %" PRIu32, k_cycle_get_32());
}

static void _dhcpv4_handler(struct net_mgmt_event_callback *pstNetMgmtEvtCb, uint32_t mgmtEvent, struct net_if *netIf)
{
  char buf[NET_IPV4_ADDR_LEN];

  switch (mgmtEvent)
  {
  case NET_EVENT_IPV4_DHCP_START:
    break;
  case NET_EVENT_IPV4_DHCP_STOP:
    break;
  case NET_EVENT_IPV4_DHCP_BOUND:
    break;
  case NET_EVENT_IPV4_ADDR_ADD:
    if(NET_ADDR_DHCP == netIf->config.ip.ipv4->unicast[0].addr_type)
    {
      LOG_INF("Your address: %s", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->unicast[0].address.in_addr, buf, sizeof(buf)));
      LOG_INF("Lease time: %u seconds", netIf->config.dhcpv4.lease_time);
      LOG_INF("Subnet: %s", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->netmask, buf, sizeof(buf)));
      LOG_INF("Router: %s", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->gw, buf, sizeof(buf)));
    }
    break;
  default:
    break;
  }
}
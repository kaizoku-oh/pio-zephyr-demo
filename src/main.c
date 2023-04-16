/*-----------------------------------------------------------------------------------------------*/
/* Includes                                                                                      */
/*-----------------------------------------------------------------------------------------------*/
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <net/net_if.h>
#include <net/net_mgmt.h>
#include <net/socket.h>
#include <net/mqtt.h>
#include <random/rand32.h>

/*-----------------------------------------------------------------------------------------------*/
/* Defines                                                                                       */
/*-----------------------------------------------------------------------------------------------*/
/* Blink delay in milliseconds */
#define BLINK_DELAY_MS (100)
/* Publish delay in milliseconds */
#define PUBLISH_DELAY_MS (3000)
/* The devicetree node identifiers for the "led0" "led1" and "led2" aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
/* The devicetree node identifier for the "sw0" alias */
#define SW0_NODE DT_ALIAS(sw0)
/* IP address for "test.mosquitto.org" */
#define BROKER_IP_ADDRESS "91.121.93.94"

/*-----------------------------------------------------------------------------------------------*/
/* Private function prototypes                                                                   */
/*-----------------------------------------------------------------------------------------------*/
/* Button callback function */
static void _button_pressed_cb(const struct device *, struct gpio_callback *, uint32_t);
/* DHCP client callback function */
static void _dhcpv4_handler(struct net_mgmt_event_callback *, uint32_t, struct net_if *);
/* MQTT client callback function */
static void _mqtt_evt_handler(struct mqtt_client *, const struct mqtt_evt *);

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
static struct gpio_callback buttonCbData = {0};
/* Network Management event callback structure */
static struct net_mgmt_event_callback netMgmtEvtCb = {0};
/* Got IP address flag */
static bool gotIP = false;
/* Buffers for MQTT client */
static uint8_t rxBuffer[256] = {0};
static uint8_t txBuffer[256] = {0};
/* MQTT client context */
static struct mqtt_client clientCtx = {0};
/* MQTT Broker address information */
static struct sockaddr_storage broker = {0};
/* MQTT connection flag */
static bool mqttConnected = false;

/*-----------------------------------------------------------------------------------------------*/
/* Exported functions                                                                            */
/*-----------------------------------------------------------------------------------------------*/
void main(void)
{
  struct net_if *netIf = NULL;
  struct sockaddr_in *broker4 = NULL;
  struct pollfd fds[1] = {0};
  struct mqtt_publish_param publishParams;

  printk("============================================\r\n");
  printk("========== PlatformIO Zephyr demo ==========\r\n");
  printk("Board:     %s\r\n", CONFIG_BOARD);
  printk("MCU:       %s\r\n", CONFIG_SOC);
  printk("Frequency: %d MHz\r\n", CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000);
  printk("============================================\r\n");

  /* Configure LEDs */
  gpio_pin_configure_dt(&greenLED, GPIO_OUTPUT_ACTIVE);
  printk("Setting up green LED at %s pin %d\r\n", greenLED.port->name, greenLED.pin);
  gpio_pin_configure_dt(&blueLED, GPIO_OUTPUT_ACTIVE);
  printk("Setting up blue LED at %s pin %d\r\n", blueLED.port->name, blueLED.pin);
  gpio_pin_configure_dt(&redLED, GPIO_OUTPUT_ACTIVE);
  printk("Setting up red LED at %s pin %d\r\n", redLED.port->name, redLED.pin);

  /* Configure button */
  gpio_pin_configure_dt(&button, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  gpio_init_callback(&buttonCbData, _button_pressed_cb, BIT(button.pin));
  gpio_add_callback(button.port, &buttonCbData);
  printk("Setting up button at %s pin %d\r\n", button.port->name, button.pin);

  /* Start DHCP client */
  net_mgmt_init_event_callback(&netMgmtEvtCb, _dhcpv4_handler, NET_EVENT_IPV4_ADDR_ADD);
  net_mgmt_add_event_callback(&netMgmtEvtCb);
  netIf = net_if_get_default();
  net_dhcpv4_start(netIf);

  /* Wait for an IP address from the DHCP server */
  printk("Waiting for IP address");
  while(!gotIP)
  {
    printk(".");
    k_msleep(500);
  }

  /* Connect to MQTT broker */
  mqtt_client_init(&clientCtx);

  /* Broker configuration */
  broker4 = (struct sockaddr_in *)&broker;
  broker4->sin_family = AF_INET;
  broker4->sin_port = htons(1883);
  zsock_inet_pton(AF_INET, BROKER_IP_ADDRESS, &broker4->sin_addr);

  /* MQTT client configuration */
  clientCtx.broker = &broker;
  clientCtx.evt_cb = _mqtt_evt_handler;
  clientCtx.client_id.utf8 = (uint8_t *)"zephyr_mqtt_client";
  clientCtx.client_id.size = sizeof("zephyr_mqtt_client") - 1;
  clientCtx.password = NULL;
  clientCtx.user_name = NULL;
  clientCtx.protocol_version = MQTT_VERSION_3_1_1;
  clientCtx.transport.type = MQTT_TRANSPORT_NON_SECURE;

  /* MQTT buffers configuration */
  clientCtx.rx_buf = rxBuffer;
  clientCtx.rx_buf_size = sizeof(rxBuffer);
  clientCtx.tx_buf = txBuffer;
  clientCtx.tx_buf_size = sizeof(txBuffer);

  /* Start connecting to the broker */
  mqtt_connect(&clientCtx);

  /* Wait for MQTT client to connect to the broker */
  printk("Connecting to %s (test.mosquitto.org)", BROKER_IP_ADDRESS);
  while(!mqttConnected)
  {
    printk(".");
    k_msleep(500);
    fds[0].fd = clientCtx.transport.tcp.sock;
    fds[0].events = ZSOCK_POLLIN;
    zsock_poll(fds, 1, 500);
    mqtt_input(&clientCtx);
  }
  printk("\r\n");

  /* Simulate LEDs racing effect */
  while(1)
  {
    gpio_pin_toggle_dt(&greenLED);
    k_msleep(BLINK_DELAY_MS);
    gpio_pin_toggle_dt(&blueLED);
    k_msleep(BLINK_DELAY_MS);
    gpio_pin_toggle_dt(&redLED);
    k_msleep(BLINK_DELAY_MS);

    /* Publish MQTT message */
    publishParams.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    publishParams.message.topic.topic.utf8 = (uint8_t *)"zephyr/test";
    publishParams.message.topic.topic.size = strlen(publishParams.message.topic.topic.utf8);
    publishParams.message.payload.data = (uint8_t *)"Hello world from zephyr!";
    publishParams.message.payload.len = strlen(publishParams.message.payload.data);
    publishParams.message_id = sys_rand32_get();
    publishParams.dup_flag = 0U;
    publishParams.retain_flag = 0U;
    mqtt_publish(&clientCtx, &publishParams);
    k_msleep(PUBLISH_DELAY_MS);
  }
}

/*-----------------------------------------------------------------------------------------------*/
/* Private functions                                                                             */
/*-----------------------------------------------------------------------------------------------*/
static void _button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  printk("Button pressed at %"PRIu32"\r\n", k_cycle_get_32());
}

static void _dhcpv4_handler(struct net_mgmt_event_callback *netMgmtEvtCb, uint32_t mgmtEvent, struct net_if *netIf)
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
      printk("\r\nYour address: %s\r\n", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->unicast[0].address.in_addr, buf, sizeof(buf)));
      printk("Lease time: %u seconds\r\n", netIf->config.dhcpv4.lease_time);
      printk("Subnet: %s\r\n", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->netmask, buf, sizeof(buf)));
      printk("Router: %s\r\n", net_addr_ntop(AF_INET, &netIf->config.ip.ipv4->gw, buf, sizeof(buf)));
      gotIP = true;
    }
    break;
  default:
    break;
  }
}

static void _mqtt_evt_handler(struct mqtt_client *client, const struct mqtt_evt *evt)
{
  switch(evt->type)
  {
  case MQTT_EVT_CONNACK:
    mqttConnected = true;
    break;
  case MQTT_EVT_DISCONNECT:
    mqttConnected = false;
    break;
  default:
    break;
  }
}
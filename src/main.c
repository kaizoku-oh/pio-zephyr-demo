#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pio_zephyr_demo, LOG_LEVEL_DBG);
#include <kernel.h>

void main(void)
{
  k_msleep(1000);
  LOG_INF("============================================");
  LOG_INF("========== PlatformIO Zephyr demo ==========");
  LOG_INF("============================================");
  while(1)
  {
    k_msleep(1000);
  }
}

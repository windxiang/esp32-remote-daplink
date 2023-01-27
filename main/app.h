#ifndef _APP_H_
#define _APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_task.h>
#include <esp_idf_version.h>

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#if CONFIG_IDF_TARGET_ESP32S2
#include <esp32s2/rom/gpio.h>
#include <esp32s2/rom/usb/chip_usb_dw_wrapper.h>
#include <esp32s2/rom/usb/usb_persist.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/rom/gpio.h>
#include <esp32s3/rom/usb/chip_usb_dw_wrapper.h>
#include <esp32s3/rom/usb/usb_persist.h>
#else
#error Unsupported architecture.
#endif

#include <soc/gpio_periph.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/usb_periph.h>

#include <hal/usb_hal.h>
#include <soc/gpio_periph.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/usb_periph.h>
#include <soc/usb_wrap_struct.h>

// DAPLink

#ifndef delay
#define delay(time) vTaskDelay(time / portTICK_PERIOD_MS);
#endif // delay

#ifdef __cplusplus
}
#endif

#endif /* _APP_H_ */

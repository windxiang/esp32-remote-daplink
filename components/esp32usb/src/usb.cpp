#include "usb.h"
#include <esp_log.h>

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

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>

#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_task.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/usb_hal.h>
#include <soc/gpio_periph.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/usb_periph.h>
#include <soc/usb_wrap_struct.h>
#include <string>
#include "class/hid/hid_device.h"

static constexpr const char* const TAG = "USB";

#if CFG_TUD_CDC
#include "class/cdc/cdc_device.h"
static void cdc_task(void)
{
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++) {
        // connected() check for DTR bit
        // Most but not all terminal client set this when making connection
        // if ( tud_cdc_n_connected(itf) )
        {
            if (tud_cdc_n_available(itf)) {
                uint8_t buf[128 + 2] = { 0 };

                uint16_t size = tud_cdc_n_read(itf, buf, sizeof(buf) - 2);
                buf[size] = 0;
                printf("%s\r\n", (char*)buf);

                tud_cdc_n_write_str(itf, (const char*)buf);
                tud_cdc_n_write_flush(itf);
            }
        }
    }
}
#endif

/**
 * @brief 初始化ESP32 USB系统
 *
 * @param external_phy
 */
static void init_usb_subsystem(bool external_phy = false)
{
    ESP_LOGI(TAG, "Initializing USB peripheral");
    if ((chip_usb_get_persist_flags() & USBDC_PERSIST_ENA) == USBDC_PERSIST_ENA) {
        // Enable USB/IO_MUX peripheral reset on next reboot.
        REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_IO_MUX_RESET_DISABLE);
        REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
    } else {
        // Normal startup flow, reinitailize the USB peripheral.
        periph_module_reset(PERIPH_USB_MODULE);
        periph_module_enable(PERIPH_USB_MODULE);
    }

    ESP_LOGD(TAG, "Initializing USB HAL");
    usb_hal_context_t hal;
    hal.use_external_phy = external_phy;
    usb_hal_init(&hal);

    if (external_phy) {
        // gpio_output_set_high(0x10, 0, 0x1E, 0xE);
    } else {
        ESP_LOGV(TAG, "Setting GPIO %d drive to %d", USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
        gpio_set_drive_capability((gpio_num_t)USBPHY_DM_NUM, (gpio_drive_cap_t)GPIO_DRIVE_CAP_3);
        ESP_LOGV(TAG, "Setting GPIO %d drive to %d", USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);
        gpio_set_drive_capability((gpio_num_t)USBPHY_DP_NUM, (gpio_drive_cap_t)GPIO_DRIVE_CAP_3);
    }

    for (const usb_iopin_dsc_t* iopin = usb_periph_iopins; iopin->pin != -1;
         ++iopin) {
        if (external_phy || (iopin->ext_phy_only == 0)) {
            gpio_pad_select_gpio(iopin->pin);
            if (iopin->is_output) {
                ESP_LOGV(TAG, "Configuring USB GPIO %d as OUTPUT", iopin->pin);
                gpio_matrix_out(iopin->pin, iopin->func, false, false);
            } else {
                ESP_LOGV(TAG, "Configuring USB GPIO %d as INPUT", iopin->pin);
                gpio_matrix_in(iopin->pin, iopin->func, false);
                gpio_pad_input_enable(iopin->pin);
            }
            gpio_pad_unhold(iopin->pin);
        }
    }

    ESP_LOGI(TAG, "USB system initialized");
}

/**
 * @brief USB任务
 *
 * @param param
 */
static void usb_device_task(void* param)
{
    // 初试和TinyUSB
    ESP_LOGV(TAG, "Initializing TinyUSB");
    if (!tusb_init()) {
        ESP_LOGE(TAG, "Failed to initialize TinyUSB stack!");
        abort();
    }

    ESP_LOGV(TAG, "EspUSB Task (%s) starting execution", CONFIG_ESPUSB_TASK_NAME);
    while (1) {
        tud_task();
#if CFG_TUD_CDC
        cdc_task();
#endif
    }
}

/**
 * @brief 启动USB线程
 *
 */
void start_usb_task()
{
    BaseType_t res = xTaskCreatePinnedToCore(usb_device_task, CONFIG_ESPUSB_TASK_NAME, CONFIG_ESPUSB_TASK_STACK_SIZE, nullptr, CONFIG_ESPUSB_TASK_PRIORITY + 100, nullptr, 1);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task for USB.");
        abort();
    }
    ESP_LOGI(TAG, "Created EspUSB task: %s", CONFIG_ESPUSB_TASK_NAME);
}

/**
 * @brief 初始化USB系统
 *
 */
void init_esp32_usb(void)
{
    static const char* const readme_txt = "This is esp32usb's MassStorage Class demo.\r\n\r\n"
                                          "If you find any bugs or get any questions, feel free to file an\r\n"
                                          "issue at github.com/atanisoft/esp32usb";

    init_usb_subsystem();

#if CONFIG_ESPUSB_CDC
    init_usb_cdc();
#endif
    configure_usb_descriptor_str(USB_DESC_MANUFACTURER, "ARM");
    configure_usb_descriptor_str(USB_DESC_PRODUCT, "DAPLink CMSIS-DAP");
    configure_usb_descriptor_str(USB_DESC_SERIAL_NUMBER, "070000010669ff505553726687095118a5a5a5a597969908");
    configure_usb_descriptor_str(USB_DESC_HID, "CMSIS-DAP v1");
    // configure_virtual_disk("esp32usb", 0x0100);
    // add_readonly_file_to_virtual_disk("readme.txt", readme_txt, strlen(readme_txt));
    // add_partition_to_virtual_disk("spiffs", "spiffs.bin");
    // add_firmware_to_virtual_disk();
    start_usb_task();
}

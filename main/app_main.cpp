#include "app.h"
#include "DAP_config.h"

// 移植:
//     https://github.com/atanisoft/esp32usb

extern void init_esp32_usb(void);

extern void usbd_hid_init(void);

#ifdef __cplusplus
extern "C" {
#endif

extern void DAP_Setup(void);
extern uint8_t SWD_Transfer(uint32_t request, uint32_t* data);
void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    fflush(stdout);

    DAP_Setup();
    usbd_hid_init();
    init_esp32_usb();

    // while (1) {
        // DAP_SETUP();
        // SWD_Transfer(2, NULL);
        // vTaskDelay(1000 * 60 * 60 / portTICK_PERIOD_MS);
    // }
}

} // extern "C" {
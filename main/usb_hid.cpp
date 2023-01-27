#include "sdkconfig.h"

#if CONFIG_ESPUSB_HID

#include "class/hid/hid.h"
#include "class/hid/hid_device.h"
#include "DAP_queue.h"
#include "DAP.h"
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

static volatile uint8_t USB_ResponseIdle = 1;
static DAP_queue DAP_Cmd_queue;

// USB HID Callback: when system initializes
void usbd_hid_init(void)
{
    USB_ResponseIdle = 1;
    DAP_queue_init(&DAP_Cmd_queue);
}

void hid_send_packet(void)
{
    uint8_t* sbuf;
    int slen;

    if (DAP_queue_get_send_buf(&DAP_Cmd_queue, &sbuf, &slen)) {
        if (slen > CONFIG_ESPUSB_HID_BUFSIZE) {
            USB_ResponseIdle = 1;
        } else {
            USB_ResponseIdle = 0;
            tud_hid_report(0, sbuf, CONFIG_ESPUSB_HID_BUFSIZE);
            // vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    } else {
        USB_ResponseIdle = 1;
    }
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, /*uint16_t*/ uint8_t len)
{
    hid_send_packet();
}

/**
 * @brief 收到SET_REPORT控制请求 或 收到OUT端点的数据时调用
 *
 * @param itf
 * @param report_id
 * @param report_type
 * @param buffer
 * @param bufsize
 * @return void
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    if (bufsize == 0 || buffer == NULL) {
        return;
    }

    if (buffer[0] == ID_DAP_TransferAbort) {
        return;
    }

    uint8_t* rbuf;

    if (DAP_queue_execute_buf(&DAP_Cmd_queue, buffer, bufsize, &rbuf)) {
        if (1 == USB_ResponseIdle) {
            USB_ResponseIdle = 0;
            hid_send_packet();
        }
    }
}

#endif // CONFIG_ESPUSB_HID

#include "sdkconfig.h"

#if CONFIG_ESPUSB_HID

#include "class/hid/hid.h"
#include "class/hid/hid_device.h"

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
TU_ATTR_WEAK uint16_t tud_hid_get_report_cb(
    uint8_t itf, uint8_t report_id, hid_report_type_t report_type,
    uint8_t* buffer, uint16_t reqlen)
{
    (void)itf;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
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
TU_ATTR_WEAK void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

#endif // CONFIG_ESPUSB_HID

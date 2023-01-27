#include "sdkconfig.h"

#if CONFIG_ESPUSB_HID
#include <class/hid/hid.h>
#endif // CONFIG_ESPUSB_HID

#include <esp_log.h>
#include <string>
#include "usb.h"

static constexpr const char* const TAG = "USB";

// When CDC is enabled set the default descriptor to use ACM mode.
#if CONFIG_ESPUSB_CDC
#define USB_DEVICE_CLASS TUSB_CLASS_MISC
#define USB_DEVICE_SUBCLASS MISC_SUBCLASS_COMMON
#define USB_DEVICE_PROTOCOL MISC_PROTOCOL_IAD
#else
// #define USB_DEVICE_CLASS 0x00
// #define USB_DEVICE_SUBCLASS 0x00
// #define USB_DEVICE_PROTOCOL 0x00
#define USB_DEVICE_CLASS 0xEF
#define USB_DEVICE_SUBCLASS 0x02
#define USB_DEVICE_PROTOCOL 0x01
#endif

// Used to generate the USB PID based on enabled interfaces.
#define _PID_MAP(itf, n) ((CFG_TUD_##itf) << (n))

/// USB 设备描述符。
static tusb_desc_device_t s_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_DEVICE_CLASS,
    .bDeviceSubClass = USB_DEVICE_SUBCLASS,
    .bDeviceProtocol = USB_DEVICE_PROTOCOL,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = CONFIG_ESPUSB_USB_VENDOR_ID,
    .idProduct = 0x0204, //(0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) | _PID_MAP(DFU_RT, 5)),
    .bcdDevice = CONFIG_ESPUSB_DESC_BCDDEVICE,
    .iManufacturer = USB_DESC_MANUFACTURER,
    .iProduct = USB_DESC_PRODUCT,
    .iSerialNumber = USB_DESC_SERIAL_NUMBER,
    .bNumConfigurations = 0x01
};

/// USB Device Endpoint assignments.
///
/// NOTE: The ESP32-S2 has four input FIFOs available, unfortunately this will
/// result in some overlap between features. The notification endpoint is not
/// connected to the FIFOs.
///
/// @todo switch to dynamic endpoint assignment except for CDC and NOTIF which
/// require static definitions.
typedef enum {
    /// Vendor endpoint.
    ENDPOINT_VENDOR_OUT = 0x01,

    /// Mass Storage endpoint.
    ENDPOINT_MSC_OUT = 0x02,

    /// CDC endpoint.
    ///
    /// NOTE: This matches the ESP32-S2 ROM code mapping.
    ENDPOINT_CDC_OUT = 0x03,

    /// MIDI endpoint.
    ENDPOINT_MIDI_OUT = 0x04,

    /// HID endpoint.
    ENDPOINT_HID_OUT = 0x05,

    /// HID endpoint.
    ENDPOINT_HID_IN = 0x85,

    /// Mass Storage endpoint.
    ENDPOINT_MSC_IN = 0x82,

    /// Vendor endpoint.
    ENDPOINT_VENDOR_IN = 0x83,

    /// MIDI endpoint.
    ENDPOINT_MIDI_IN = 0x83,

    /// CDC endpoint.
    ///
    /// NOTE: This matches the ESP32-S2 ROM code mapping.
    ENDPOINT_CDC_IN = 0x84,

    /// Notification endpoint.
    ///
    /// NOTE: This matches the ESP32-S2 ROM code mapping.
    ENDPOINT_NOTIF = 0x85,
} esp_usb_endpoint_t;

/// USB Interface indexes.
typedef enum {
#if CONFIG_ESPUSB_CDC
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
#endif
#if CONFIG_ESPUSB_MSC
    ITF_NUM_MSC,
#endif
#if CONFIG_ESPUSB_HID
    ITF_NUM_HID,
#endif
#if CONFIG_ESPUSB_MIDI
    ITF_NUM_MIDI,
    ITF_NUM_MIDI_STREAMING,
#endif
#if CONFIG_ESPUSB_VENDOR
    ITF_NUM_VENDOR,
#endif
#if CONFIG_ESPUSB_DFU
    ITF_NUM_DFU_RT,
#endif
    ITF_NUM_TOTAL // 配置描述符的总配置数量
} esp_usb_interface_t;

/// Total size of the USB device descriptor configuration data.
static constexpr uint16_t USB_DESCRIPTORS_CONFIG_TOTAL_LEN = TUD_CONFIG_DESC_LEN + (CONFIG_ESPUSB_CDC * TUD_CDC_DESC_LEN) + (CONFIG_ESPUSB_MSC * TUD_MSC_DESC_LEN) + (CONFIG_ESPUSB_HID * TUD_HID_INOUT_DESC_LEN) + (CONFIG_ESPUSB_VENDOR * TUD_VENDOR_DESC_LEN) + (CONFIG_ESPUSB_MIDI * TUD_MIDI_DESC_LEN) + (CONFIG_ESPUSB_DFU * TUD_DFU_RT_DESC_LEN);

#if CONFIG_ESPUSB_HID
// HID Report Descriptor
static uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE)),
    // TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL)),
    // TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(REPORT_ID_GAMEPAD))
};

static const uint8_t daplink_report[] = {
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26,
    0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02,
    0x95, 0x40, 0x09, 0x01, 0x91, 0x02, 0x95, 0x01, 0x09, 0x01,
    0xB1, 0x02, 0xC0
};

// Invoked when received GET HID REPORT DESCRIPTOR request
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance)
{
    return daplink_report;
    // return desc_hid_report;
}
#endif // CONFIG_ESPUSB_HID

/// USB 配置描述符
uint8_t const desc_configuration[USB_DESCRIPTORS_CONFIG_TOTAL_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, USB_DESCRIPTORS_CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, CONFIG_ESPUSB_MAX_POWER_USAGE),

#if CONFIG_ESPUSB_CDC
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, USB_DESC_CDC, ENDPOINT_NOTIF, 8, ENDPOINT_CDC_OUT, ENDPOINT_CDC_IN, CONFIG_ESPUSB_CDC_FIFO_SIZE),
#endif
#if CONFIG_ESPUSB_MSC
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, USB_DESC_MSC, ENDPOINT_MSC_OUT, ENDPOINT_MSC_IN, CONFIG_ESPUSB_MSC_FIFO_SIZE),
#endif
#if CONFIG_ESPUSB_HID
    // TUD_HID_DESCRIPTOR(ITF_NUM_HID, USB_DESC_HID, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), ENDPOINT_HID_IN, CONFIG_ESPUSB_HID_BUFSIZE, 10),
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, USB_DESC_HID, HID_ITF_PROTOCOL_NONE, sizeof(daplink_report), ENDPOINT_HID_OUT, ENDPOINT_HID_IN, CONFIG_ESPUSB_HID_BUFSIZE, 1),
#endif
#if CONFIG_ESPUSB_VENDOR
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, USB_DESC_VENDOR, ENDPOINT_VENDOR_OUT, ENDPOINT_VENDOR_IN, CONFIG_ESPUSB_VENDOR_FIFO_SIZE),
#endif
#if CONFIG_ESPUSB_MIDI
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, USB_DESC_MIDI, ENDPOINT_MIDI_OUT, ENDPOINT_MIDI_IN, CONFIG_ESPUSB_MIDI_FIFO_SIZE),
#endif
#if CONFIG_ESPUSB_DFU
    TUD_DFU_RT_DESCRIPTOR(ITF_NUM_DFU_RT, USB_DESC_DFU, 0x0d, CONFIG_ESPUSB_DFU_DISCONNECT_DELAY, CONFIG_ESPUSB_DFU_BUFSIZE),
#endif
};

/// USB device descriptor strings.
///
/// NOTE: Only ASCII characters are supported at this time.
static std::string s_str_descriptor[USB_DESC_MAX_COUNT] = {
    "", // LANGUAGE (unused in tud_descriptor_string_cb)
    "", // USB_DESC_MANUFACTURER
    "", // USB_DESC_PRODUCT
    "", // USB_DESC_SERIAL_NUMBER
    "", // USB_DESC_CDC
    "", // USB_DESC_MSC
    "", // USB_DESC_HID
    "", // USB_DESC_VENDOR
    "", // USB_DESC_MIDI
    "", // USB_DESC_DFU
};

/// Maximum length of the USB device descriptor strings.
static constexpr size_t MAX_DESCRIPTOR_LEN = 126;

/// Temporary holding buffer for USB device descriptor string data in UTF-16
/// format.
///
/// NOTE: Only ASCII characters are supported at this time.
static uint16_t _desc_str[MAX_DESCRIPTOR_LEN + 1];

/**
 * @brief 初始化默认的USB描述符
 *
 * @param desc
 * @param version
 */
void configure_usb_descriptor(tusb_desc_device_t* desc, uint16_t version)
{
    if (desc) {
        memcpy(&s_descriptor, desc, sizeof(tusb_desc_device_t));
    } else if (version) {
        s_descriptor.bcdDevice = version;
    }
}

/**
 * @brief 设置USB字符串描述符
 *
 * @param index
 * @param value
 */
void configure_usb_descriptor_str(esp_usb_descriptor_index_t index, const char* value)
{
    // truncate the descriptor string (if needed).
    size_t str_len = strlen(value);
    if (str_len > MAX_DESCRIPTOR_LEN) {
        ESP_LOGE(TAG, "USB descriptor(%d) text will be truncated (%zu > %zu)", index, str_len, MAX_DESCRIPTOR_LEN);
        str_len = MAX_DESCRIPTOR_LEN;
    }

    s_str_descriptor[index].assign(value, str_len);
    ESP_LOGI(TAG, "USB descriptor(%d) text:%s", index, s_str_descriptor[index].c_str());
}

// =============================================================================
// TinyUSB CALLBACKS
// =============================================================================

extern "C" {

/**
 * @brief 收到 GET DEVICE DESCRIPTOR 时调用
 * TinyUSB中来调用
 *
 * @return uint8_t const*
 */
uint8_t const* tud_descriptor_device_cb(void)
{
    return (uint8_t const*)&s_descriptor;
}

/**
 * @brief 收到 GET CONFIGURATION DESCRIPTOR 时调用
 * TinyUSB中来调用
 * @param index
 * @return uint8_t const*
 */
uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations
    return desc_configuration;
}

/**
 * @brief 收到 GET STRING DESCRIPTOR 请求时调用
 * TinyUSB中来调用
 * @param index
 * @param langid
 * @return uint16_t const*
 */
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    uint8_t chr_count;
    // clear the last descriptor
    bzero(_desc_str, TU_ARRAY_SIZE(_desc_str));

    if (index == 0) {
        _desc_str[1] = tu_htole16(0x0409);
        chr_count = 1;
    } else if (index >= USB_DESC_MAX_COUNT) {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors
        return NULL;
    } else {
        // copy the string into the temporary array starting at offset 1
        size_t idx = 1;
        for (char ch : s_str_descriptor[index]) {
            _desc_str[idx++] = tu_htole16(ch);
        }
        chr_count = s_str_descriptor[index].length();
    }

    // length and type
    _desc_str[0] = tu_htole16((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

#if CONFIG_ESPUSB_DFU
// Invoked when the DFU Runtime mode is requested
void tud_dfu_rt_reboot_to_dfu(void)
{
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_PROCPU_RST);
}
#endif // CONFIG_ESPUSB_DFU

} // extern "C"
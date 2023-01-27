        #ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

#define SWCLK_TCK_PIN_Bit GPIO_NUM_10
#define SWDIO_OUT_PIN_Bit GPIO_NUM_11
#define SWDIO_IN_PIN_Bit GPIO_NUM_12
#define nRESET_PIN_Bit GPIO_NUM_13
#define CONNECTED_LED_PIN_Bit GPIO_NUM_14

#define DAP_PACKET_SIZE 64
#define DAP_PACKET_COUNT 8 // DAP队列大小
#define DAP_SWD 1
#define DAP_JTAG 0
#define CPU_CLOCK (240 * 1000 * 1000)
#define DAP_DEFAULT_SWJ_CLOCK (5 * 1000 * 1000)
#define IO_PORT_WRITE_CYCLES 2
#define SWO_UART 0
#define DAP_DEFAULT_PORT 1 ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.
#define SWO_MANCHESTER 0 ///< SWO Manchester:  1 = available, 0 = not available

/**
 * @brief 设置IO为输出模式
 *
 * @param GPIOx
 * @param pin_bit
 * @return __inline
 */
static __inline void pin_out_init(gpio_num_t pin_bit)
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT64(pin_bit);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

static __inline void pin_out_od_init(gpio_num_t pin_bit)
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(pin_bit);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

static __inline void pin_in_init(gpio_num_t pin_bit, uint8_t mode)
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT64(pin_bit);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    if (mode == 1) {
        // 上拉
        gpio_set_pull_mode(pin_bit, GPIO_PULLUP_ONLY);
    } else if (mode == 2) {
        // 下拉
        gpio_set_pull_mode(pin_bit, GPIO_PULLDOWN_ONLY);
    } else {
        // 浮空
        gpio_set_pull_mode(pin_bit, GPIO_FLOATING);
    }
}

//**************************************************************************************************
/**
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor


DAP Hardware I/O Pin Access Functions
-------------------------------------
The various I/O Pins are accessed by functions that implement the Read, Write, Set, or Clear to
these I/O Pins.

For the SWDIO I/O Pin there are additional functions that are called in SWD I/O mode only.
This functions are provided to achieve faster I/O that is possible with some advanced GPIO
peripherals that can independently write/read a single I/O pin without affecting any other pins
of the same I/O port. The following SWDIO I/O Pin functions are provided:
 - \ref PIN_SWDIO_OUT_ENABLE to enable the output mode from the DAP hardware.
 - \ref PIN_SWDIO_OUT_DISABLE to enable the input mode to the DAP hardware.
 - \ref PIN_SWDIO_IN to read from the SWDIO I/O pin with utmost possible speed.
 - \ref PIN_SWDIO_OUT to write to the SWDIO I/O pin with utmost possible speed.
*/

// Configure DAP I/O pins ------------------------------

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
Configures the DAP Hardware I/O pins for JTAG mode:
 - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
 - TDO to input mode.
*/
static __inline void PORT_JTAG_SETUP(void)
{
#if (DAP_JTAG != 0)

#endif
}

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
 - SWCLK, SWDIO, nRESET to output mode and set to default high level.
 - TDI, TMS, nTRST to HighZ mode (pins are unused in SWD mode).
*/
static __inline void PORT_SWD_SETUP(void)
{
    // Set SWCLK HIGH
    pin_out_init(SWCLK_TCK_PIN_Bit);
    gpio_set_level(SWCLK_TCK_PIN_Bit, 1);

    // Set SWDIO HIGH
    pin_out_init(SWDIO_OUT_PIN_Bit);
    gpio_set_level(SWDIO_OUT_PIN_Bit, 1);

    // Set RESET HIGH
    pin_out_od_init(nRESET_PIN_Bit);
    gpio_set_level(nRESET_PIN_Bit, 1);
}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
static __inline void PORT_OFF(void)
{
    pin_in_init(SWCLK_TCK_PIN_Bit, 0);
    pin_in_init(SWDIO_OUT_PIN_Bit, 0);
    pin_in_init(SWDIO_IN_PIN_Bit, 0);
}

// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
static inline uint32_t PIN_SWCLK_TCK_IN(void)
{
    return (gpio_get_level(SWCLK_TCK_PIN_Bit) & 1);
}

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
static inline void PIN_SWCLK_TCK_SET(void)
{
    gpio_set_level(SWCLK_TCK_PIN_Bit, 1);
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
static inline void PIN_SWCLK_TCK_CLR(void)
{
    gpio_set_level(SWCLK_TCK_PIN_Bit, 0);
}

// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Get Input.
\return Current status of the SWDIO/TMS DAP hardware I/O pin.
*/
static inline uint32_t PIN_SWDIO_TMS_IN(void)
{
    return (gpio_get_level(SWDIO_IN_PIN_Bit) & 1);
}

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
static inline void PIN_SWDIO_TMS_SET(void)
{
    gpio_set_level(SWDIO_OUT_PIN_Bit, 1);
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
static inline void PIN_SWDIO_TMS_CLR(void)
{
    gpio_set_level(SWDIO_OUT_PIN_Bit, 0);
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
static inline uint32_t PIN_SWDIO_IN(void)
{
    return (gpio_get_level(SWDIO_IN_PIN_Bit) & 1);
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
static inline void PIN_SWDIO_OUT(uint32_t bit)
{
    gpio_set_level(SWDIO_OUT_PIN_Bit, (bit & 1));
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
static inline void PIN_SWDIO_OUT_ENABLE(void)
{
    pin_out_init(SWDIO_OUT_PIN_Bit);
    gpio_set_level(SWDIO_OUT_PIN_Bit, 0);
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
static inline void PIN_SWDIO_OUT_DISABLE(void)
{
    pin_in_init(SWDIO_OUT_PIN_Bit, 0);
}

// TDI Pin I/O ---------------------------------------------

/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
static inline uint32_t PIN_TDI_IN(void)
{
    return (0); // Not available
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
static inline void PIN_TDI_OUT(uint32_t bit)
{
    ; // Not available
}

// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
static inline uint32_t PIN_TDO_IN(void)
{
    return (0); // Not available
}

// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
static inline uint32_t PIN_nTRST_IN(void)
{
    return (0); // Not available
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
static inline void PIN_nTRST_OUT(uint32_t bit)
{
    ; // Not available
}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
static inline uint32_t PIN_nRESET_IN(void)
{
    return (gpio_get_level(nRESET_PIN_Bit) & 1);
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
// TODO - sw specific implementation should be created

static inline void PIN_nRESET_OUT(uint32_t bit)
{
    gpio_set_level(nRESET_PIN_Bit, (bit & 1));
}

//**************************************************************************************************
/**
\defgroup DAP_Config_LEDs_gr CMSIS-DAP Hardware Status LEDs
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware may provide LEDs that indicate the status of the CMSIS-DAP Debug Unit.

It is recommended to provide the following LEDs for status indication:
 - Connect LED: is active when the DAP hardware is connected to a debugger.
 - Running LED: is active when the debugger has put the target device into running state.
*/

/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
static __inline void LED_CONNECTED_OUT(uint32_t bit)
{
    gpio_set_level(CONNECTED_LED_PIN_Bit, (bit & 1));
}

/** Debug Unit: Set status Target Running LED.
\param bit status of the Target Running LED.
           - 1: Target Running LED ON: program execution in target started.
           - 0: Target Running LED OFF: program execution in target stopped.
*/
static __inline void LED_RUNNING_OUT(uint32_t bit)
{
    ; // Not available
}

///@}

//**************************************************************************************************
/**
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
static __inline void DAP_SETUP(void)
{
    /* Configure I/O pin SWCLK */
    pin_out_init(SWCLK_TCK_PIN_Bit);
    gpio_set_level(SWCLK_TCK_PIN_Bit, 1);

    pin_out_init(SWDIO_OUT_PIN_Bit);
    gpio_set_level(SWDIO_OUT_PIN_Bit, 1);

    pin_in_init(SWDIO_IN_PIN_Bit, 1);

    pin_out_od_init(nRESET_PIN_Bit);
    gpio_set_level(nRESET_PIN_Bit, 1);

    pin_out_init(CONNECTED_LED_PIN_Bit);
    gpio_set_level(CONNECTED_LED_PIN_Bit, 1);
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
static __inline uint32_t RESET_TARGET(void)
{
    return (0); // change to '1' when a device reset sequence is implemented
}

///@}

#endif /* __DAP_CONFIG_H__ */

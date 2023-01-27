
set(srcs_cmsisdap
    "cmsis-dap/DAP.c"
    "cmsis-dap/DAP_queue.c"
    "cmsis-dap/DAP_vendor.c"
    "cmsis-dap/JTAG_DP.c"
    "cmsis-dap/SW_DP.c"
    "cmsis-dap/SWO.c"
)

set(srcs_usb
    "usb_hid.cpp"
)

idf_component_register(SRCS "app_main.cpp"  "${srcs_cmsisdap}" "${srcs_usb}"
                    INCLUDE_DIRS "." "./cmsis-dap" 
                    REQUIRES spi_flash esp32usb)

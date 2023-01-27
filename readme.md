# 基于ESP32S3 的远程 DAPLink 调试模块
* 使用场景：无人机远程调试、平衡车等不方便连接电脑的场景下进行调试、下载
* 参考:[esp32usb](https://github.com/atanisoft/esp32usb)
* 参考:[tinyusb](https://github.com/hathach/tinyusb)
* 参考:[daplink](https://github.com/ARMmbed/DAPLink)
* 已完成:USB HID DAPLink的协议栈移植，已经可以正常识别出STM32，但是在擦除下载过程中，还有问题还没找到原因
* 后续还要使用ESP32建立Server，通过WIFI来连接，进行远程调试

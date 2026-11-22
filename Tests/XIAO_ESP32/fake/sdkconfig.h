// sdkconfig.h - stands for ESP-IDF's header of that name: which chip the program is built for (XIAO test harness).
// run.sh builds the program twice: plain for the XIAO ESP32C3, with -DXIAO_KIT_S3 for the XIAO ESP32S3.
#pragma once
#include "fake_core.h"
#ifdef XIAO_KIT_S3
#define CONFIG_IDF_TARGET_ESP32S3 1
#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x0009
#else
#define CONFIG_IDF_TARGET_ESP32C3 1
#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x0005
#endif

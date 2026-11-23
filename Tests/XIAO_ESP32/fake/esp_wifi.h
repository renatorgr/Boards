// esp_wifi.h - stands for the ESP-IDF header of that name: the access point's list of devices (XIAO test harness).
#pragma once
#include "fake_core.h"
typedef int esp_err_t;
#define ESP_WIFI_MAX_CONN_NUM 15
typedef struct { uint8_t mac[6]; int8_t rssi; } wifi_sta_info_t;
typedef struct wifi_sta_list_t { wifi_sta_info_t sta[ESP_WIFI_MAX_CONN_NUM]; int num; } wifi_sta_list_t;
esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t* sta);

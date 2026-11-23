// esp_netif.h - stands for the ESP-IDF header of that name: which address the access point's DHCP gave each device (XIAO test harness).
#pragma once
#include "fake_core.h"
typedef int esp_err_t;
typedef struct esp_netif_obj esp_netif_t;
typedef struct { uint32_t addr; } esp_ip4_addr_t;
typedef struct { uint8_t mac[6]; esp_ip4_addr_t ip; } esp_netif_pair_mac_ip_t;
esp_netif_t* esp_netif_get_handle_from_ifkey(const char* if_key);
esp_err_t esp_netif_dhcps_get_clients_by_mac(esp_netif_t* esp_netif, int num, esp_netif_pair_mac_ip_t* mac_ip_pair);

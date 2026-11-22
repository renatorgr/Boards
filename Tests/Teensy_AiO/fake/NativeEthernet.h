// NativeEthernet.h - the Teensy 4.1's Ethernet (NativeEthernet + its FNET stack) on fake::Device: UDP per port,
// the web page's TCP connections, the link, mDNS (Teensy test harness, Tests/Teensy_AiO).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fake_core.h"

// ---- FNET, as far as zWeb.ino and zEthernet.ino use it
typedef void* fnet_socket_t;
typedef uint32_t fnet_size_t;
typedef uint32_t fnet_uint32_t;
typedef int32_t fnet_int32_t;
typedef uint16_t fnet_uint16_t;
typedef uint8_t fnet_uint8_t;
typedef int fnet_return_t;
typedef int fnet_bool_t;
#define FNET_NULL nullptr
#define FNET_ERR (-1)
#define FNET_OK 0
#define FNET_TRUE 1
#define FNET_FALSE 0
#define AF_INET 2
#define SOCK_STREAM 1
#define SOCK_DGRAM 2
#define SOL_SOCKET 0xFFFF
#define SO_RCVBUF 0x1002
#define SO_SNDBUF 0x1001
#define SO_LINGER 0x0080
#define IPPROTO_TCP 6
#define TCP_NODELAY 1
#define INADDR_ANY 0u
#define FNET_HTONS(n) ((uint16_t)((((n) & 0xFF) << 8) | (((n) >> 8) & 0xFF)))
struct fnet_in_addr { uint32_t s_addr; };
struct fnet_sockaddr { uint16_t sa_family; uint16_t sa_port; uint32_t sa_scope_id; uint8_t sa_data[16]; };
struct fnet_sockaddr_in { uint16_t sin_family; uint16_t sin_port; uint32_t sin_scope_id; fnet_in_addr sin_addr; uint8_t pad[12]; };
struct fnet_linger { fnet_bool_t l_onoff; fnet_uint16_t l_linger; };
typedef struct { char* key_name; char* key_value; } fnet_mdns_txt_key_t;
fnet_socket_t fnet_socket(int family, int type, int protocol);
int  fnet_socket_bind(fnet_socket_t s, const fnet_sockaddr* a, fnet_size_t len);
int  fnet_socket_listen(fnet_socket_t s, int backlog);
fnet_socket_t fnet_socket_accept(fnet_socket_t s, fnet_sockaddr* a, fnet_size_t* len);
int  fnet_socket_close(fnet_socket_t s);
int  fnet_socket_setopt(fnet_socket_t s, int level, int name, const void* v, fnet_size_t len);
void fnet_poll();
void* fnet_netif_get_default();

// ---- NativeEthernet
#define MAX_SOCK_NUM 8
#define UDP_TX_PACKET_MAX_SIZE 24
enum EthernetLinkStatus { Unknown, LinkON, LinkOFF };
enum EthernetHardwareStatus { EthernetNoHardware, EthernetW5100, EthernetW5200, EthernetW5500, EthernetTeensy41 };

class EthernetClass
{
public:
    int  begin(uint8_t* mac, unsigned long timeout = 60000, unsigned long responseTimeout = 4000);
    void begin(uint8_t* mac, IPAddress ip);
    EthernetLinkStatus linkStatus();
    EthernetHardwareStatus hardwareStatus();
    IPAddress localIP();
    void setLocalIP(const IPAddress ip);
    void setSocketSize(size_t) {}
    void setSocketNum(uint8_t) {}
    void setStackHeap(size_t) {}
    int  maintain() { return 0; }
    fnet_socket_t socket_ptr[MAX_SOCK_NUM] = {};
    static IntervalTimer _fnet_poll;
};
extern EthernetClass Ethernet;

class EthernetClient
{
public:
    EthernetClient() : sock_(MAX_SOCK_NUM) {}
    explicit EthernetClient(uint8_t sock) : sock_(sock) {}
    operator bool();
    uint8_t connected();
    int     available();
    int     read();
    int     read(uint8_t* buf, size_t n);
    size_t  write(uint8_t b) { return write(&b, 1); }
    size_t  write(const uint8_t* buf, size_t n);
    int     availableForWrite();
    void    flush() {}
    void    stop();
    IPAddress remoteIP() { return IPAddress(192, 168, 5, 10); }
private:
    fake::TcpConn* conn();
    uint8_t sock_;
};

class MDNSClass
{
public:
    int  begin(const char* host, int services = 1);
    void setServiceName(const char* name);
    int  addService(const char* type, uint16_t port, const fnet_mdns_txt_key_t* (*txt)() = nullptr);
};
extern MDNSClass MDNS;

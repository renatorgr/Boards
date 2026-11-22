// lwip/sockets.h - stands for lwIP's header of that name: the UDP sockets and send() the XIAO program uses
// (XIAO test harness). The bodies are in fake_core.cpp. Not the PC's own sockets: no system socket header is
// included anywhere in the program or the pretend core, so these names are free.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cerrno>

#define AF_INET       2
#define SOCK_DGRAM    2
#define IPPROTO_UDP   17
#define SOL_SOCKET    0xfff
#define SO_REUSEADDR  0x0004
#define SO_BROADCAST  0x0020
#define MSG_DONTWAIT  0x08
#define INADDR_ANY    ((uint32_t)0)

#ifndef __socklen_t_defined
typedef uint32_t socklen_t;
#define __socklen_t_defined
#endif
struct in_addr { uint32_t s_addr; };
struct sockaddr { uint8_t sa_len; uint8_t sa_family; char sa_data[14]; };
struct sockaddr_in { uint8_t sin_len; uint8_t sin_family; uint16_t sin_port; struct in_addr sin_addr; char sin_zero[8]; };

static inline uint16_t htons(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }
static inline uint32_t htonl(uint32_t v) { return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) | ((v >> 8) & 0xFF00) | (v >> 24); }

int lwip_socket(int domain, int type, int protocol);
int lwip_setsockopt(int s, int level, int optname, const void* optval, socklen_t optlen);
int lwip_bind(int s, const struct sockaddr* name, socklen_t namelen);
int lwip_recvfrom(int s, void* mem, size_t len, int flags, struct sockaddr* from, socklen_t* fromlen);
int lwip_sendto(int s, const void* data, size_t size, int flags, const struct sockaddr* to, socklen_t tolen);
int lwip_close(int s);
int send(int s, const void* data, size_t size, int flags);      // on a browser's connection: what it takes now

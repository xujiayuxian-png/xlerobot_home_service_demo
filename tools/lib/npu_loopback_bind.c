/* Restrict the vendor experiment server's IP listeners to loopback. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>

int bind(int fd, const struct sockaddr *address, socklen_t length) {
  int (*original)(int, const struct sockaddr *, socklen_t) = dlsym(RTLD_NEXT, "bind");
  if (!original) { errno = ENOSYS; return -1; }
  if (address && address->sa_family == AF_INET && length >= sizeof(struct sockaddr_in)) {
    struct sockaddr_in local;
    memcpy(&local, address, sizeof(local));
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return original(fd, (const struct sockaddr *)&local, sizeof(local));
  }
  if (address && address->sa_family == AF_INET6 && length >= sizeof(struct sockaddr_in6)) {
    struct sockaddr_in6 local;
    memcpy(&local, address, sizeof(local));
    local.sin6_addr = in6addr_loopback;
    local.sin6_scope_id = 0;
    return original(fd, (const struct sockaddr *)&local, sizeof(local));
  }
  return original(fd, address, length);
}

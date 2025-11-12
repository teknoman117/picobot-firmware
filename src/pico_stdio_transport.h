#ifndef PICO_STDIO_TRANSPORT_H
#define PICO_STDIO_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct uxrCustomTransport;

bool pico_stdio_transport_open(struct uxrCustomTransport * transport);
bool pico_stdio_transport_close(struct uxrCustomTransport * transport);
size_t pico_stdio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf,
        size_t len, uint8_t * err);
size_t pico_stdio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len,
        int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // PICO_STDIO_TRANSPORT_H

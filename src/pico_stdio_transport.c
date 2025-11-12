#include <stdio.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "tusb.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

#define RX_BYTES_AVAILABLE 0x01

void pico_stdio_transport_bytes_available_callback(void *handle) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t task = handle;

    // when bytes are available, wake up executor
    if (handle && tud_cdc_available()) {
        xTaskNotifyFromISR(handle, RX_BYTES_AVAILABLE, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void usleep(uint64_t us) {
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_stdio_transport_open(struct uxrCustomTransport * transport) {
    stdio_set_chars_available_callback(pico_stdio_transport_bytes_available_callback,
            xTaskGetCurrentTaskHandle());
    return true;
}

bool pico_stdio_transport_close(struct uxrCustomTransport * transport) {
    stdio_set_chars_available_callback(NULL, NULL);
    return true;
}

size_t pico_stdio_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len,
        uint8_t *errcode) {
    int ret = stdio_put_string(buf, len, false, false);
    if (ret != len) {
        *errcode = 1;
    }

    stdio_flush();
    return ret;
}

size_t pico_stdio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len,
        int timeout, uint8_t *errcode) {
    absolute_time_t when = make_timeout_time_ms(timeout);

    // drain existing chars
    size_t received = 0;
    int rc = stdio_get_until(buf, len, make_timeout_time_us(0));
    if (rc != PICO_ERROR_TIMEOUT) {
        received = rc;
    }

    // block transport/executor thread until either timeout or we receive some data
    if (timeout) {
        BaseType_t xResult = pdTRUE;
        while (received != len && xResult) {
            uint32_t ulNotifiedValue = 0;
            int64_t remaining_time = absolute_time_diff_us(get_absolute_time(), when);
            xResult = xTaskNotifyWait(pdFALSE, UINT32_MAX, &ulNotifiedValue,
                    pdMS_TO_TICKS(us_to_ms(remaining_time)));

            rc = stdio_get_until(buf + received, len - received, make_timeout_time_us(0));
            if (rc != PICO_ERROR_TIMEOUT) {
                received += rc;
            }
        }
    }

    // signal timeout error if we've not managed to fetch designed byte count
    if (received != len) {
        *errcode = 1;
    }
    return received;
}

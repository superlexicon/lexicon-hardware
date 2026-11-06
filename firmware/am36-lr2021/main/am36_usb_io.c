/**
 * @file am36_usb_io.c
 * @brief High-speed USB-Serial-JTAG I/O implementation for Lierda AM36-Pico (ESP32-S3)
 */
#include "am36_usb_io.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_log.h"

#define AM36_USB_RX_BUF_SIZE  (16 * 1024)
#define AM36_USB_TX_BUF_SIZE  (16 * 1024)

static SemaphoreHandle_t s_usb_tx_mutex = NULL;
static bool s_usb_initialized = false;

esp_err_t am36_usb_io_init(void)
{
    if (s_usb_initialized) {
        return ESP_OK;
    }

    s_usb_tx_mutex = xSemaphoreCreateMutex();
    if (s_usb_tx_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = AM36_USB_RX_BUF_SIZE,
        .tx_buffer_size = AM36_USB_TX_BUF_SIZE,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK) {
        s_usb_initialized = true;
    }
    return err;
}

int am36_usb_io_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    if (!s_usb_initialized || buf == NULL || max_len == 0) {
        return 0;
    }
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    int n = usb_serial_jtag_read_bytes(buf, max_len, ticks);
    return (n > 0) ? n : 0;
}

void am36_usb_io_write_bytes(const uint8_t *data, size_t len)
{
    if (!s_usb_initialized || data == NULL || len == 0) {
        return;
    }

    xSemaphoreTake(s_usb_tx_mutex, portMAX_DELAY);

    size_t offset = 0;
    uint32_t stall_iters = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > 512) {
            chunk = 512;
        }
        int written = usb_serial_jtag_write_bytes(data + offset, chunk, pdMS_TO_TICKS(20));
        if (written > 0) {
            offset += (size_t)written;
            stall_iters = 0;
        } else {
            stall_iters++;
            if (stall_iters > 25) {
                /* Host CDC port not open or stalled > 500ms; drop remainder so radio never blocks */
                break;
            }
        }
    }
    usb_serial_jtag_ll_txfifo_flush();

    xSemaphoreGive(s_usb_tx_mutex);
}

void am36_usb_io_write_str(const char *str)
{
    if (str == NULL) {
        return;
    }
    am36_usb_io_write_bytes((const uint8_t *)str, strlen(str));
}

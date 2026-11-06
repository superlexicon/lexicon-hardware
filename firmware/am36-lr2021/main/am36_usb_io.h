/**
 * @file am36_usb_io.h
 * @brief High-speed non-blocking USB-Serial-JTAG I/O driver for Lierda AM36-Pico (ESP32-S3)
 *
 * Uses the native ESP32-S3 USB-Serial-JTAG controller (CON2 / GPIO19 D-, GPIO20 D+)
 * to achieve ~5-8 Mbps host framing throughput without the 1 Mbps UART bottleneck
 * of the older XIAO nRF54L15 board.
 */
#ifndef AM36_USB_IO_H
#define AM36_USB_IO_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ESP32-S3 USB-Serial-JTAG driver with deep RX/TX ringbuffers.
 */
esp_err_t am36_usb_io_init(void);

/**
 * @brief Read up to max_len bytes from USB-Serial-JTAG with timeout in milliseconds.
 * @return Number of bytes read (>= 0).
 */
int am36_usb_io_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Write exact len bytes to USB-Serial-JTAG, flushing to host.
 *        Thread-safe (protected by internal mutex).
 */
void am36_usb_io_write_bytes(const uint8_t *data, size_t len);

/**
 * @brief Write null-terminated string to USB-Serial-JTAG.
 *        Thread-safe (protected by internal mutex).
 */
void am36_usb_io_write_str(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* AM36_USB_IO_H */

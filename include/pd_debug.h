//
// USB Power Delivery Sink Using FUSB302B
// Copyright (c) 2020 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// Debugging support (sending debugging messages to output)
//

#ifndef _pd_debug_h_
#define _pd_debug_h_

#include <stdint.h>

#if defined(PD_DEBUG)

#define DEBUG_LOG(MSG, VAL) ::usb_pd::debug_log(MSG, VAL)

#define DEBUG_INIT() ::usb_pd::debug_init()

#define DEBUG_UART_LOOP() ::usb_pd::debug_uart_loop()

namespace usb_pd {

void debug_log(const char* msg, uint32_t val);
void debug_init();
void debug_uart_loop(void);

} // namespace usb_pd

#else

#define DEBUG_LOG(MSG, VAL)                                                                                            \
    do {                                                                                                               \
    } while (false)

#define DEBUG_INIT()                                                                                                   \
    do {                                                                                                               \
    } while (false)

#define DEBUG_UART_LOOP()                                                                                              \
    do {                                                                                                               \
    } while (false)

#endif

#endif

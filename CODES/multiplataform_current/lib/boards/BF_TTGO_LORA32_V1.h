#if !defined(_RADIOBOARDS_MAINTAINED_TTGO_LORA32_V1_H)
#define _RADIOBOARDS_MAINTAINED_TTGO_LORA32_V1_H

// sources:

#if USE_SERIAL
    auto& serial = Serial;
    #if USE_SERIAL == 1
        #define INIT_USB_SERIAL    \
        serial.begin(MONITOR_SPEED);
    #else
        #define UART_TX_PIN TX
        #define UART_RX_PIN RX
        #define INIT_EXTERNAL_SERIAL \
        serial.setPins(UART_RX_PIN, UART_TX_PIN); \
        serial.begin(MONITOR_SPEED);
    #endif
#endif

#if USE_SERIAL == 1 ||  USE_SERIAL == 2
#define RADIOLIB_DEBUG_PORT serial
#endif

#define RADIO_BOARDS_NAME "TTGO LoRa32 v1"

#define RADIO_NSS     (18)
#define RADIO_IRQ     (26)
#define RADIO_RST     (14)
#define RADIO_GPIO    (33)

#define Radio       SX1276



#endif
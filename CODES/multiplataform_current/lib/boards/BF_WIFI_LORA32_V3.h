#if !defined(_RADIOBOARDS_MAINTAINED_HELTEC_WIFI_LORA32_V3_H)
#define _RADIOBOARDS_MAINTAINED_HELTEC_WIFI_LORA32_V3_H

// sources:
// https://github.com/espressif/arduino-esp32/blob/master/variants/heltec_*/pins_arduino.h
#if USE_SERIAL
    auto& serial = Serial;
    #if USE_SERIAL == 1
        #define INIT_USB_SERIAL    \
        serial.begin(MONITOR_SPEED);
    #else
        #define UART_TX_PIN 43
        #define UART_RX_PIN 44
        #define INIT_EXTERNAL_SERIAL \
        serial.setPins(UART_RX_PIN, UART_TX_PIN); \
        serial.begin(MONITOR_SPEED);
    #endif
#endif

#if USE_SERIAL == 1 ||  USE_SERIAL == 2
#define RADIOLIB_DEBUG_PORT serial
#endif

#define RADIO_BOARDS_NAME "Heltec WiFi LoRa32 v3"

#define RADIO_NSS     (8)
#define RADIO_IRQ     (14)
#define RADIO_RST     (12)
#define RADIO_GPIO    (13)

#define Radio       SX1262



#endif
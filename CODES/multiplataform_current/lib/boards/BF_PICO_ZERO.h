#if !defined(_BOARD_RASPBERRYPI_PICO_ZERO)
#define _BOARD_RASPBERRYPI_PICO_ZERO

#define RADIO_BOARDS_NAME "Waveshare RP2040 Zero"

#if USE_SERIAL == 1
    auto& serial = Serial;
    #define INIT_USB_SERIAL    \
    serial.begin(MONITOR_SPEED);
#elif USE_SERIAL == 2
    #define UART_TX_PIN 0
    #define UART_RX_PIN 1
    auto& serial = Serial1;
    #define INIT_EXTERNAL_SERIAL    \
    serial.setRX(UART_RX_PIN);      \
    serial.setTX(UART_TX_PIN);      \
    serial.begin(MONITOR_SPEED);
#else
    auto& serial = Serial;
#endif

#if USE_SERIAL == 1 ||  USE_SERIAL == 2
#define RADIOLIB_DEBUG_PORT serial
#endif

#define Radio       SX1276

#define RADIO_NSS     (5)   // i.e., SX1276 radio = new Module(5, 14, 8, 15, SPI, RADIOLIB_DEFAULT_SPI_SETTINGS);
#define RADIO_IRQ     (14)
#define RADIO_RST     (8)
#define RADIO_GPIO    (15)

// this board uses custom SPI to interface with the module
#define RADIO_SPI     SPI
#define RADIO_MISO    (4)
#define RADIO_MOSI    (3)
#define RADIO_SCK     (2)

#define RADIO_SPI_INIT          \
RADIO_SPI.setSCK(RADIO_SCK);    \
RADIO_SPI.setTX(RADIO_MOSI);    \
RADIO_SPI.setRX(RADIO_MISO);    \
RADIO_SPI.begin(false);

#endif

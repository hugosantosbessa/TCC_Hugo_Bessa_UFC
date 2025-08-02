#if !defined(_BOARDS_H)
#define _BOARDS_H

// select the appropriate header
// maintained boards
#if defined(BF_PICO_ZERO)
  #include "BF_PICO_ZERO.h"
#elif defined(BF_WIFI_LORA32_V3)
  #include "BF_WIFI_LORA32_V3.h"
#elif  defined(BF_TTGO_LORA32_V1)
  #include "BF_TTGO_LORA32_V1.h"
#else
  #error "Unsupported or unknown radio board!"
#endif

#if RADIO_BOARDS_DEBUG
  #pragma message "RadioBoards using board: " RADIO_BOARDS_NAME
#endif


#if USE_SERIAL == 1
  #define SerialInit()  INIT_USB_SERIAL
#elif USE_SERIAL == 2
  #define SerialInit() INIT_EXTERNAL_SERIAL
#else
  #define SerialInit() {}
#endif


// setup SPI
#if defined(RADIO_SPI)
  #define RadioBeginSPI()  RADIO_SPI_INIT
#else
  #define RadioBeginSPI()  {}
#endif

// Module constructor
#if defined(RADIO_SPI)
  #define RadioModule() Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO, RADIO_SPI)
  #define RadioModuleHal(HAL) Module(HAL, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO, RADIO_SPI)
#else
    #define RadioModule() Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO)
    #define RadioModuleHal(HAL) Module(HAL, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO)
#endif

// RF switching
#if defined(RADIO_RF_SWITCH)
    RADIO_RF_SWITCH_PINS;
    RADIO_RF_SWITCH_TABLE;
    #define RadioEnableRfSwitch(radio)    radio.setRfSwitchTable(RadioBoards_rfswitch_pins, RadioBoards_rfswitch_table)
#else
    #define RadioEnableRfSwitch(radio)    {}
#endif


#if MODE_DEEP_SLEEP_PICO == 1 && !defined(COUNTER_DEEP_SLEEP_PWM)
  #define SetupBoard()  setup_sleep();
#elif MODE_DEEP_SLEEP_PICO == 1 && defined(COUNTER_DEEP_SLEEP_PWM)
  #define SetupBoard()  \
    setup_sleep();      \
    setupPwmCount(COUNT_GPIO); 
#elif MODE_DEEP_SLEEP_PICO == 2 && defined(COUNTER_DEEP_SLEEP_PWM)
  #define SetupBoard()                \
    setup_sleep();                    \
    setupPwmCount(COUNT_GPIO);        \
    gpio_init(WAKE_GPIO);             \
    gpio_set_dir(WAKE_GPIO, GPIO_IN);
#elif MODE_DEEP_SLEEP_PICO == 2 && !defined(COUNTER_DEEP_SLEEP_PWM)
#define SetupBoard()                \
  setup_sleep();                    \
  gpio_init(WAKE_GPIO);             \
  gpio_set_dir(WAKE_GPIO, GPIO_IN);
#elif !MODE_DEEP_SLEEP_PICO && defined(COUNTER_DEEP_SLEEP_PWM)
  #define SetupBoard() setupPwmCount(COUNT_GPIO);
#else
  #define SetupBoard() {}
#endif




#endif
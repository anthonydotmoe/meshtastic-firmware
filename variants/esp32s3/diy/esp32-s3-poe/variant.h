#define USE_SX1262

#define HAS_ETHERNET (1)
#define USE_WS5500

// WS5500 Ethernet
#define ETH_CS_PIN 14
#define ETH_INT_PIN 10
#define ETH_RST_PIN 9
#define ETH_SCLK_PIN 13
#define ETH_MISO_PIN 12
#define ETH_MOSI_PIN 11

#define LED_PIN 18

#define I2C_SCL 47
#define I2C_SDA 48

#define UART_TX 43
#define UART_RX 44

#define LORA_DIO0 -1 // a No connect on the SX1262 module
#define LORA_RESET 39
#define LORA_DIO1 15 // SX1262 IRQ
#define LORA_DIO2 38 // SX1262 BUSY
#define LORA_DIO3    // Module TCXO enable

#define LORA_SCK 42
#define LORA_MISO 40
#define LORA_MOSI 41
#define LORA_CS 45

#define SX126X_RXEN 2
#define SX126X_TXEN 1

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_DIO2
#define SX126X_RESET LORA_RESET

//#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

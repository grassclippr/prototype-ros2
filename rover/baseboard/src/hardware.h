#ifndef HARDWARE_H
#define HARDWARE_H

#define BOOT_BUTTON_PIN GPIO_NUM_0

// BUSSES
#define TWAI_TX_PIN GPIO_NUM_1
#define TWAI_RX_PIN GPIO_NUM_2
#define UART_RX_PIN GPIO_NUM_3
#define UART_TX_PIN GPIO_NUM_4
#define UART_FAULT_PIN GPIO_NUM_5
#define I2C_SDA_PIN GPIO_NUM_6
#define I2C_SCL_PIN GPIO_NUM_7

// Status LEDS
//#define UART_LED GPIO_NUM_39    // LEDC_CHANNEL_7
#define STATUS_LED GPIO_NUM_40  // LEDC_CHANNEL_0
#define ERROR_LED GPIO_NUM_41   // LEDC_CHANNEL_1
#define TWAI_LED GPIO_NUM_42    // LEDC_CHANNEL_2
#define LYNX_A_LED GPIO_NUM_45  // LEDC_CHANNEL_3
#define LYNX_B_LED GPIO_NUM_46  // LEDC_CHANNEL_4
#define LYNX_C_LED GPIO_NUM_47  // LEDC_CHANNEL_5
#define LYNX_D_LED GPIO_NUM_48  // LEDC_CHANNEL_6

#endif  // HARDWARE_H

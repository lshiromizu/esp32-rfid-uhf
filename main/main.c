#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rfid_module.h"


#define UART_TX_PIN         4
#define UART_RX_PIN         5
#define UART_BAUD_RATE      115200
#define UART_BUFFER_SIZE    1024*2

#define BUTTON_GPIO_PIN     GPIO_NUM_0
#define ENABLE_GPIO_PIN     GPIO_NUM_1


volatile bool buttonPressed = false;


void IRAM_ATTR button_ISR() {
    buttonPressed = true;
    gpio_set_level(ENABLE_GPIO_PIN, 1);
}

void init()
{
    const uart_port_t uart_num = UART_NUM_1;
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_queue, 0));

    gpio_config_t button_conf;
    // configure button pin as input
    button_conf.intr_type = GPIO_INTR_POSEDGE; // interrupt on rising edge
    button_conf.pin_bit_mask = (1ULL << BUTTON_GPIO_PIN); // Select pin
    button_conf.mode = GPIO_MODE_INPUT; // Set as input mode
    button_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up
    button_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down
    gpio_config(&button_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // install ISR
    gpio_isr_handler_add(BUTTON_GPIO_PIN, button_ISR, (void*) BUTTON_GPIO_PIN);   // hook ISR handler for specific gpio pin

    // Configure the enable pin
    gpio_config_t enable_conf;
    enable_conf.intr_type = GPIO_INTR_DISABLE; // No interrupt
    enable_conf.pin_bit_mask = (1ULL << ENABLE_GPIO_PIN); // Select pin
    enable_conf.mode = GPIO_MODE_OUTPUT; // Set as output mode
    gpio_config(&enable_conf);
};

void app_main(void)
{
    init();
    printf("code running\r\n");
    while (1)
    {
        if (buttonPressed == true)
        {
            printf("Button pressed\r\n");
            set_power(6.0, false);
            float current_power = get_power();
            printf("Current power: %.1f dBm\r\n", current_power);

            set_RF_mode(3, false);
            int current_mode = get_RF_mode();
            printf("Current mode: %d \r\n", current_mode);
            buttonPressed = false;
            gpio_set_level(ENABLE_GPIO_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}

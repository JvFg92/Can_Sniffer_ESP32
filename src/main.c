#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "string.h"
#include "stdbool.h"
#include "driver/twai.h" 

#define BUF_SIZE (1024)
#define BAUD_RATE (115200)
#define UART0 UART_NUM_0
#define PC_TXD_PIN (GPIO_NUM_1)
#define PC_RXD_PIN (GPIO_NUM_3)

static const char *TAG = "TWAI_EXAMPLE_PIO";

static QueueHandle_t uart_queue_0; 

void uart_init_config(void) {

    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,             // Taxa de baud
        .data_bits = UART_DATA_8_BITS,      // 8 bits de dados
        .parity = UART_PARITY_DISABLE,      // Sem paridade
        .stop_bits = UART_STOP_BITS_1,      // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Sem controle de fluxo de hardware
        .source_clk = UART_SCLK_APB,        // Usa o clock APB para a UART
    };

    ESP_ERROR_CHECK(uart_param_config(UART0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0, PC_TXD_PIN, PC_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART0, BUF_SIZE * 2, 0, 0, NULL, 0)); 
    uart_queue_0 = xQueueCreate(1, sizeof(char) * BUF_SIZE);

    printf("UART%d inicializada nos pinos TXD (GPIO%d), RXD (GPIO%d) a %d baud.\n",
           UART0, PC_TXD_PIN, PC_RXD_PIN, BAUD_RATE);
}

void uart_send_task(void *pvParameters) {
    char data[BUF_SIZE]; 
    while (1) {
        if (xQueueReceive(uart_queue_0, &data, portMAX_DELAY) == pdPASS) {
            uart_write_bytes(UART0, data, strlen(data));
        }
    }
}

void uart_receive_task(void *pvParameters) {
    static const char *PC_RX_TASK_TAG = "PC_RX_TASK";
    esp_log_level_set(PC_RX_TASK_TAG, ESP_LOG_INFO); 

    char* data = (char*) malloc(BUF_SIZE + 1); 
    if (data == NULL) {
        ESP_LOGE(PC_RX_TASK_TAG, "Falha ao alocar memória para o buffer RX!");
        vTaskDelete(NULL); 
    }

    while (1) {
        memset(data, 0, BUF_SIZE + 1);
        int rxBytes = uart_read_bytes(UART0, data, BUF_SIZE, 5 / portTICK_PERIOD_MS);

        if (rxBytes > 0) {
           data[rxBytes] = '\0';
            //ESP_LOGI(PC_RX_TASK_TAG, "Recebido: '%s'", data); // Loga a mensagem recebida

            if (xQueueSend(uart_queue_0, data, portMAX_DELAY) != pdPASS) {
                    ESP_LOGE(PC_RX_TASK_TAG, "Falha ao enviar dados para a fila");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void) {

    uart_init_config();

    xTaskCreate(uart_send_task, "uart_send_task", BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 1, NULL);


    ESP_LOGI(TAG, "Iniciando o exemplo TWAI com PlatformIO");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "Driver TWAI instalado.");
    } else {
        ESP_LOGE(TAG, "Falha ao instalar o driver TWAI.");
        return;
    }

    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Driver TWAI iniciado.");
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar o driver TWAI.");
        twai_driver_uninstall();
        return;
    }

    twai_message_t message;
    message.identifier = 0x123;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = 8;
    for (int i = 0; i < 8; i++) {
        message.data[i] = i;
    }

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        ESP_LOGI(TAG, "Mensagem TWAI enviada com sucesso.");
    } else {
        ESP_LOGE(TAG, "Falha ao enviar mensagem TWAI.");
    }
}

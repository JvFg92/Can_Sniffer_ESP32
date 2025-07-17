#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "string.h"
#include "stdbool.h"

// Define as configurações para as duas UARTs
#define BUF_SIZE (1024)
#define BAUD_RATE (115200)

// Configuração UART0 (geralmente usada para comunicação com o PC via USB)
#define UART0_PORT UART_NUM_0
#define UART0_TXD_PIN (GPIO_NUM_1)
#define UART0_RXD_PIN (GPIO_NUM_3)

// Configuração UART2 (conectada a um módulo externo, por exemplo)
#define UART2_PORT UART_NUM_2
#define UART2_TXD_PIN (GPIO_NUM_17) // PINO TX DO ESP32 PARA A UART2
#define UART2_RXD_PIN (GPIO_NUM_16) // PINO RX DO ESP32 PARA A UART2

static const char *TAG = "UART_BRIDGE";

// Filas para enviar dados de uma UART para a outra
static QueueHandle_t uart0_tx_queue;
static QueueHandle_t uart2_tx_queue;

// Estrutura para passar parâmetros para as tarefas UART
typedef struct {
    uart_port_t rx_port;
    QueueHandle_t tx_queue;
} uart_task_params_t;

// Função de inicialização genérica para qualquer UART
void uart_init_config(uart_port_t uart_num, int txd_pin, int rxd_pin) {
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART%d inicializada nos pinos TXD (GPIO%d), RXD (GPIO%d) a %d baud.",
             uart_num, txd_pin, rxd_pin, BAUD_RATE);
}

// Tarefa genérica para RECEBER dados de uma UART e ENVIAR para a outra
void uart_receive_and_forward_task(void *pvParameters) {
    uart_task_params_t *params = (uart_task_params_t *)pvParameters;
    uart_port_t rx_port = params->rx_port;
    QueueHandle_t tx_queue = params->tx_queue;

    char* data = (char*) malloc(BUF_SIZE + 1);
    if (data == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar memória para o buffer RX da UART%d!", rx_port);
        vTaskDelete(NULL);
    }

    while (1) {
        memset(data, 0, BUF_SIZE + 1);
        int rxBytes = uart_read_bytes(rx_port, data, BUF_SIZE, 50 / portTICK_PERIOD_MS);
        
        if (rxBytes > 0) {
           data[rxBytes] = '\0'; // Adiciona terminador de string
           ESP_LOGI(TAG, "UART%d -> Recebido: '%s'", rx_port, data);
           
           // Envia os dados recebidos para a fila de transmissão da OUTRA UART
           if (xQueueSend(tx_queue, data, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Falha ao enviar dados da UART%d para a fila de transmissão.", rx_port);
           }
        }
    }
}

// Tarefa genérica para ENVIAR dados de uma fila para uma UART
void uart_send_task(void *pvParameters) {
    uart_task_params_t *params = (uart_task_params_t *)pvParameters;
    uart_port_t tx_port = params->rx_port;
    QueueHandle_t tx_queue = params->tx_queue;
    
    char data[BUF_SIZE];
    while (1) {
        if (xQueueReceive(tx_queue, &data, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "UART%d -> Enviando: '%s'", tx_port, data);
            uart_write_bytes(tx_port, data, strlen(data));
        }
    }
}

void app_main(void) {
    // Inicializa as duas UARTs
    uart_init_config(UART0_PORT, UART0_TXD_PIN, UART0_RXD_PIN);
    uart_init_config(UART2_PORT, UART2_TXD_PIN, UART2_RXD_PIN);

    // Cria as filas de comunicação entre as tarefas
    uart0_tx_queue = xQueueCreate(10, sizeof(char) * BUF_SIZE);
    uart2_tx_queue = xQueueCreate(10, sizeof(char) * BUF_SIZE);

    // Declaração das variáveis
    uart_task_params_t uart0_rx_params;
    uart_task_params_t uart0_tx_params;
    uart_task_params_t uart2_rx_params;
    uart_task_params_t uart2_tx_params;

    // Inicialização das variáveis em tempo de execução
    uart0_rx_params.rx_port = UART0_PORT;
    uart0_rx_params.tx_queue = uart2_tx_queue; // Recebe de UART0 e envia para a fila de transmissão de UART2

    uart0_tx_params.rx_port = UART0_PORT;
    uart0_tx_params.tx_queue = uart0_tx_queue; // Envia de sua própria fila de transmissão para UART0

    uart2_rx_params.rx_port = UART2_PORT;
    uart2_rx_params.tx_queue = uart0_tx_queue; // Recebe de UART2 e envia para a fila de transmissão de UART0

    uart2_tx_params.rx_port = UART2_PORT;
    uart2_tx_params.tx_queue = uart2_tx_queue; // Envia de sua própria fila de transmissão para UART2

    // Cria as tarefas
    xTaskCreate(uart_receive_and_forward_task, "uart0_rx_task", 4096, (void*)&uart0_rx_params, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(uart_send_task, "uart0_tx_task", 4096, (void*)&uart0_tx_params, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(uart_receive_and_forward_task, "uart2_rx_task", 4096, (void*)&uart2_rx_params, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(uart_send_task, "uart2_tx_task", 4096, (void*)&uart2_tx_params, configMAX_PRIORITIES - 3, NULL);
}
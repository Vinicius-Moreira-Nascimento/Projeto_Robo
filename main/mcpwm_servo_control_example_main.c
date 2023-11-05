#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// includes PWM
#include "driver/ledc.h"
#include "esp_err.h"

// Includes BLUETOOTH

#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define DESLIGAR 0        // COMANDO DESLIGAR  / DESATIVAR
#define LIGAR 1           // COMANDO LIGAR / ACIONAR
#define TOGGLE 2          // COMANDO PARA ALTERAR O ESTADO ATUAL
#define BUF_SIZE 500      // Alocacao do buffer do uart na RAM
#define Size_Max_Data 500 // Alocacao do buffer do uart na RAM

#define LED_ESP32 (GPIO_NUM_2) // PINO LATCH (RCLK) DO ESPANSOR DE OUTPUTS
#define TXD_UART1 (GPIO_NUM_1) // PINO TX UART1 = GPIO1
#define RXD_UART1 (GPIO_NUM_3) // PINO RX UART1 = GPIO3
#define RTS_UART (UART_PIN_NO_CHANGE)
#define CTS_UART (UART_PIN_NO_CHANGE)

// defines PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (13) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095 // vai de zero a 8190
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// defines PWM 2
#define LEDC_TIMER_2 LEDC_TIMER_0
#define LEDC_MODE_2 LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_2 (5) // Define the output GPIO
#define LEDC_CHANNEL_2 LEDC_CHANNEL_0
#define LEDC_DUTY_RES_2 LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_2 (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095 // vai de zero a 8190
#define LEDC_FREQUENCY_2 (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// Define Bluetooth
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

char received_data[128];
char contagem[100];

uint8_t Controle_ROBO = DESLIGAR;
uint16_t Velocidade_Motor_1 = DESLIGAR;
uint16_t Velocidade_Motor_2 = DESLIGAR;

/// @brief Fução que Serve para a registrar os eventos base do bluetooth
/// @param event Variavel de evento que marcar a entrada da recepção ou transmissão
/// @param param Variavel que marca os parametros de entrada oude sainda da função
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS)
        {
            printf("\n\r ESP_SPP_INIT_EVT");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        }
        else
        {
            printf("\n\r ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        printf("\n\r ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        printf("\n\r ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        printf("\n\r ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status,
               param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS)
        {
            printf("\n\r ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                   param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        else
        {
            printf("\n\r ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        printf("\n\r ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        printf("\n\r ESP_SPP_DATA_IND_EVT len:%d handle:%ld",
               param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 128)
        {
            strncpy(received_data, (char *)param->data_ind.data, param->data_ind.len);
            received_data[param->data_ind.len] = '\0';
        }
        esp_spp_write(param->write.handle, strlen(contagem), (uint8_t *)contagem);

        break;
    case ESP_SPP_CONG_EVT:
        printf("\n\r ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        printf("\n\r ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        printf("\n\r ESP_SPP_SRV_OPEN_EVT");
        break;
    case ESP_SPP_SRV_STOP_EVT:
        printf("\n\r ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        printf("\n\r ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

/// @brief Task que Gerencia o PWM do GPIO 5
/// @param arg Argumento para inicialização da task
void xPWM(void *arg)
{
    while (1)
    {
        switch (Controle_ROBO)
        {

        case 0: // se receber desliga
            Velocidade_Motor_1 = DESLIGAR;
            Velocidade_Motor_2 = DESLIGAR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 1: // se receber Righ
            Velocidade_Motor_1 = DESLIGAR;
            Velocidade_Motor_2 = 3000;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 2: // se receber Fright
            Velocidade_Motor_1 = DESLIGAR;
            Velocidade_Motor_2 = 3000;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 3: // se receber Bright
            Velocidade_Motor_1 = DESLIGAR;
            Velocidade_Motor_2 = 3000;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 4: // se receber Left
            Velocidade_Motor_1 = 3000;
            Velocidade_Motor_2 = DESLIGAR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 5: // se receber Fleft
            Velocidade_Motor_1 = 3000;
            Velocidade_Motor_2 = DESLIGAR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 6: // se receber Bleft
            Velocidade_Motor_1 = 3000;
            Velocidade_Motor_2 = DESLIGAR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 7: // Se receber  Forward
            Velocidade_Motor_1 = 8000;
            Velocidade_Motor_2 = 8000;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;
        case 8: // Se receber Back
            Velocidade_Motor_1 = DESLIGAR;
            Velocidade_Motor_2 = DESLIGAR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_Motor_1);
            ledc_set_duty(LEDC_MODE_2, LEDC_CHANNEL_2, Velocidade_Motor_2);
            break;

        default:
            break;
        }
        // aguarda novo valor
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        ledc_update_duty(LEDC_MODE_2, LEDC_CHANNEL_2);
    }
}

void xBluetooth(void *arg)
{
    char conversao[300];
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));

        if ((received_data[0] == 'R') && (received_data[1] == 'i') && (received_data[2] == 'g')) // se receber Righ
        {
            Controle_ROBO = 1;
            sprintf(conversao, "Virando para Direita: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'F') && (received_data[1] == 'r') && (received_data[2] == 'i')) // se receber Fright
        {
            Controle_ROBO = 2;
            sprintf(conversao, "Virando para Direita: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'B') && (received_data[1] == 'r') && (received_data[2] == 'i')) // se receber Bright
        {
            Controle_ROBO = 3;
            sprintf(conversao, "Virando para Direita: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'L') && (received_data[1] == 'e') && (received_data[2] == 'f')) // se receber Left
        {
            Controle_ROBO = 4;
            sprintf(conversao, "Virando para Esquerda: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'F') && (received_data[1] == 'l') && (received_data[2] == 'e')) // se receber Fleft
        {
            Controle_ROBO = 5;
            sprintf(conversao, "Virando para Esquerda: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'B') && (received_data[1] == 'l') && (received_data[2] == 'e')) // se receber Bleft
        {
            Controle_ROBO = 6;
            sprintf(conversao, "Virando para Esquerda: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'F') && (received_data[1] == 'o') && (received_data[2] == 'r')) // Se receber  Forward
        {
            Controle_ROBO = 7;
            sprintf(conversao, "Andando em frente: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else if ((received_data[0] == 'B') && (received_data[1] == 'a') && (received_data[2] == 'c')) // Se receber Back
        {
            Controle_ROBO = 8;
            sprintf(conversao, "Andando para trás: dados recebidas: %s", received_data);
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
            uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
            uart_write_bytes(UART_NUM_1, "\n\r", 2);
        }
        else{
            Controle_ROBO = DESLIGAR;
        }
    }
}

void xLED(void *arg)
{
    static bool led = 0;
    while (1)
    {
        led = !led;
        gpio_set_level(LED_ESP32, led);
        vTaskDelay(pdMS_TO_TICKS(350));
    }
}

void app_main()
{

    //======================== INICIALIZAÇÃO DA NVS ===========================//
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //============================== BLUETOOTH ===============================//
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_spp_register_callback(esp_spp_cb);

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    esp_spp_enhanced_init(&bt_spp_cfg);
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    //========================== Configuração do PWM =========================//

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    //========================== Prepara o canal 1 do PWM =======================//
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);

    //========================== Prepara o canal 2 do PWM =======================//

    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode = LEDC_MODE_2,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_2,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_2,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config(&ledc_channel_2);

    //================================GPIO LED=================================//

    gpio_config_t io_conf;                     // Ponteiro de configuração dos pinos
    io_conf.intr_type = GPIO_MODE_DEF_DISABLE; // Desabilita Interrupçoes
    io_conf.mode = GPIO_MODE_OUTPUT;           // set as output mode
    io_conf.pin_bit_mask = (1ULL << LED_ESP32);
    io_conf.pull_down_en = 0; // Desabilita modo PULL-DOWN
    io_conf.pull_up_en = 0;   // Desabilita modo PULL-UP
    gpio_config(&io_conf);    // Configura GPIO com as configurações para OUTPUT

    //====================== Configuração da UART ==============================//

    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    // UART0
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, TXD_UART1, RXD_UART1, RTS_UART, CTS_UART);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreatePinnedToCore(xPWM, "xPWM", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(xLED, "xLED", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(xBluetooth, "xBluetooth", 2048, NULL, 1, NULL, 0);
}

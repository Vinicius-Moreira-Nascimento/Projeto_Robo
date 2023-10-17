
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

#define DESLIGAR 0   // COMANDO DESLIGAR  / DESATIVAR
#define LIGAR 1      // COMANDO LIGAR / ACIONAR
#define TOGGLE 2     // COMANDO PARA ALTERAR O ESTADO ATUAL
#define BUF_SIZE 500 // Alocacao do buffer do uart na RAM
#define Size_Max_Data 500 // Alocacao do buffer do uart na RAM

#define LED_ESP32 (GPIO_NUM_2) // PINO LATCH (RCLK) DO ESPANSOR DE OUTPUTS
#define TXD_UART1 (GPIO_NUM_1) // PINO TX UART1 = GPIO1
#define RXD_UART1 (GPIO_NUM_3) // PINO RX UART1 = GPIO3
#define RTS_UART (UART_PIN_NO_CHANGE)
#define CTS_UART (UART_PIN_NO_CHANGE)

// defines PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (5) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095 // vai de zero a 8190
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// Define Bluetooth
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

// Define UART
#define Max_Sequence_Length 10
#define Sync_Byte 0x55

char received_data[128];
char contagem[100];
uint16_t Velocidade_PWM = DESLIGAR;


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
            // esp_spp_write(param->write.handle, param->data_ind.len, param->data_ind.data);
            strncpy(received_data, (char *)param->data_ind.data, param->data_ind.len);
            // Use printf to print the received data as characters
            printf("\n\r Received data: %.*s \n\r", param->data_ind.len, (char *)param->data_ind.data);
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
    char Velocidade[10];
    uint16_t tamanho = DESLIGAR;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        sprintf(Velocidade, "%d", Velocidade_PWM);
        tamanho = strlen(Velocidade);

        //
        uart_write_bytes(UART_NUM_1, "\n\r Velocidade do PWM: ", 22);
        uart_write_bytes(UART_NUM_1, Velocidade, tamanho);
        uart_write_bytes(UART_NUM_1, "\n\r", 2);
        //

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, Velocidade_PWM);
        // aguarda novo valor
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}

/// @brief Task que Gerencia os dados a telemetria do bluetooth
/// @param arg
void xBluetooth(void *arg)
{
    static bool led = 0;
    static int pisca_led = 0;
    // const char *data = "recebi";

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        // printf("\n\r Dados recebidos na task: %s \n\r", received_data);
        uart_write_bytes(UART_NUM_1, received_data, strlen(received_data));

        led = !led;
        // gpio_set_level(LED_ESP32, led); // pega o nivel logico do GPIO

        if (received_data[0] == '1')
        {
            // LIDAR COM LIGAR O LED
            received_data[0] = 5;
            pisca_led = 0;

            sprintf(contagem, "\n\r ligando o led \n\r");
            gpio_set_level(LED_ESP32, 1); // Turn on the LED
        }
        else if (received_data[0] == '0')
        {
            received_data[0] = 5;
            pisca_led = 0;
            // lidar com Desligar o LED
            sprintf(contagem, "\n\r Desligando o led \n\r");
            gpio_set_level(LED_ESP32, 0);
        }
        else if (received_data[0] == '2')
        {
            received_data[0] = 5;
            pisca_led = 1;
            // lidar com piscar o led
            sprintf(contagem, "\n\r Piscando o led \n\r");
        }
        else if (received_data[0] == '3')
        {
            received_data[0] = 5;

            // lidar com o aumento do pwm

            // Velocidade_PWM
            //  VAI DE 3000 A 8190
            if (Velocidade_PWM >= 8000)
            {
                Velocidade_PWM = 8000;
            }
            else
            {
                Velocidade_PWM = Velocidade_PWM + 1000;
            }
            sprintf(contagem, "\n\r Aumentando Velocidade \n\r");
        }
        else if (received_data[0] == '4')
        {
            received_data[0] = 5;

            // lidar com a diminuição do PWM

            if (Velocidade_PWM == 3000)
            {
                Velocidade_PWM = 3000;
            }
            else
            {
                Velocidade_PWM = Velocidade_PWM - 1000;
            }
            sprintf(contagem, "\n\r Diminuindo Velocidade \n\r");
        }
        else if (received_data[0] == '6')
        {
            received_data[0] = 5;

            // lidar com a desligar pwm
            Velocidade_PWM = DESLIGAR;

            sprintf(contagem, "\n\r DESLIGANDO PWM \n\r");
        }
        else if (received_data[0] == '5')
        {
            uart_write_bytes(UART_NUM_1, "\n\r Aguardando Dado..", 20);
        }

        if (pisca_led == 1)
        {
            gpio_set_level(LED_ESP32, led);
        }
    }
}

/// @brief Task que Gerencia os dados a via Comunicação UART
/// @param arg
// void xUART(void *arg)
// {
//     static uint16_t receivedByte = 0;
//     static bool Is_Sequence_Started = false;
//     static uint8_t byte_Counter = 0;
//     static uint8_t buffer[10] = 0;
//     // Is it a sequence to start
//     char Data_receive_Serial_Bus[Size_Max_Data];
//     while (1)
//     {
//         int RxUART2 = uart_read_bytes(UART_NUM_2, Data_receive_Serial_Bus, Size_Max_Data, pdMS_TO_TICKS(200));
//         while (RxUART2)
//         {
//             uint8_t receivedByte = uart_read_bytes(UART_NUM_2, Data_receive_Serial_Bus, Size_Max_Data, pdMS_TO_TICKS(200));
//             // Serial.println(receivedByte, HEX);

//             if (receivedByte == 0x55)
//             {

//                 Is_Sequence_Started = true;
//                 byte_Counter = 0;
//             }

//             if (Is_Sequence_Started)
//             {
//                 buffer[byte_Counter] = receivedByte;
//                 byte_Counter++;

//                 if (byte_Counter >= 3)
//                 {
//                     Is_Sequence_Started = false;

//                     for (int i = 0; i < 3; i++)
//                         buffer[i];
//                 }
//             }
//         }
//     }
// }

/// @brief Task que Gerencia os dados a via Comunicação UART
/// @param arg
void xUART(void *arg)
{
    char Data_receive_Serial_Bus[Max_Sequence_Length];
    uint8_t buffer[Max_Sequence_Length];
    uint8_t sequenceLength = 0;
    bool isSequenceStarted = false;

    while (1)
    {
        // Aguarda interrupção ou timeout para a leitura da UART
        int RxUART2 = uart_read_bytes(UART_NUM_2, Data_receive_Serial_Bus, Max_Sequence_Length, pdMS_TO_TICKS(200));

        if (RxUART2 > 0)
        {
            for (int i = 0; i < RxUART2; i++)
            {
                // Verifica se é o byte de sincronização
                if (Data_receive_Serial_Bus[i] == Sync_Byte)
                {
                    // Reinicia o buffer e a contagem da sequência
                    sequenceLength = 0;
                    isSequenceStarted = true;
                }

                // Armazena o byte no buffer se a sequência está em andamento
                if (isSequenceStarted && sequenceLength < Max_Sequence_Length)
                {
                    buffer[sequenceLength] = Data_receive_Serial_Bus[i];
                    sequenceLength++;

                    // Verifica se a sequência foi completada
                    if (sequenceLength >= 3)
                    {
                        // Faça algo com a sequência completa aqui
                        // ...

                        // Reinicia a contagem da sequência
                        sequenceLength = 0;
                        isSequenceStarted = false;
                    }
                }
            }
        }
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

    //========================== Prepara o canal do PWM =======================//
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);

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
    xTaskCreatePinnedToCore(xUART, "xUART", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(xBluetooth, "xBluetooth", 2048, NULL, 1, NULL, 0);

}

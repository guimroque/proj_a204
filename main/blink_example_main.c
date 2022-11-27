/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = " leitura -> ";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
// -> definição dos pinos
#define LED_VM 15
#define LED_VD 4
#define LED_AM 2

// -> valor de ref do pino analogico [sugestão: 0.3v]
const float VALUE_REF_IN_MIN = 0.3;
const float VALUE_REF_IN_MAX = 4.3;

// -> valores de referencia para temperatura
const int VALUE_REF_100_ = 0;
const int VALUE_REF_0 = 128;
const int VALUE_REF_400 = 694;
const int VALUE_REF_500 = 820;

static uint8_t s_led_state = 0;
int value_analog = 0;

// -> função para conversão de faixas de valores
float resize(
    float value,
    float in_min,
    float in_max,
    float out_min,
    float out_max)
{
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

static void compare_analog_read()
{
    // -> valor mínimo de leitura do pino analogico [deslocamento de zero para o valor de referencia]
    const int min_input = resize(VALUE_REF_IN_MIN, 0, 5, 0, 1023);
    const int max_input = resize(VALUE_REF_IN_MAX, 0, 5, 0, 1023);

    // -> valores de referencia convertido em 8bits
    // -> corrente variando de 1 a 5 volts
    const int min_out_current = resize(min_input, min_input, max_input, 51, 255);
    const int max_out_current = resize(max_input, min_input, max_input, 51, 255);

    const int min_out_voltage = resize(min_input, min_input, max_input, 0, 204);
    const int max_out_voltage = resize(max_input, min_input, max_input, 0, 204);

    // -> vetor que contem as referencias de temperatura
    const int adcRef[4] = {VALUE_REF_100_ + min_input, VALUE_REF_0 + min_input, VALUE_REF_400 + min_input, VALUE_REF_500 + min_input};

    // -> decisão de qual led acender
    bool state_vm = value_analog <= adcRef[0] || value_analog >= adcRef[3];
    bool state_am = (value_analog > adcRef[0] && value_analog < adcRef[1]) || (value_analog < adcRef[3] && value_analog > adcRef[2]);
    bool state_vd = value_analog > adcRef[1] && value_analog <= adcRef[2];

    int out_current = value_analog < min_input ? min_out_current : value_analog > max_input ? max_out_current
                                                                                            : resize(value_analog, min_input, max_input, min_out_current, max_out_current);
    float out_current_v = resize(out_current, 0, 255, 0, 5);

    int out_voltage = value_analog < min_input ? min_out_voltage : value_analog > max_input ? max_out_voltage
                                                                                            : resize(value_analog, min_input, max_input, min_out_voltage, max_out_voltage);
    float out_voltage_v = resize(out_voltage, 0, 255, 0, 5);

    // -> logs dos estados

    ESP_LOGI("============================================ [GPIOS] ============================================");
    ESP_LOGI("[Input Ref. Min.] %d!", min_input);
    ESP_LOGI("[Input Ref. Max.] %d!", max_input);
    ESP_LOGI("[Out Ref. Min. Voltage] %d!", min_out_voltage);
    ESP_LOGI("[Out Ref. Max. Voltage] %d!", max_out_voltage);
    ESP_LOGI("[Out Ref. Min. Current] %d!", min_out_current);
    ESP_LOGI("[Out Ref. Max. Current] %d!", max_out_current);
    ESP_LOGI("[Out Current] %d!", out_current);
    ESP_LOGI("[Out Voltage] %d!", out_voltage);
    ESP_LOGI("[Out Current[V]] %f!", out_current_v);
    ESP_LOGI("[Out Voltage[V]] %f!", out_voltage_v);
    ESP_LOGI("[Analog Read] %d!", value_analog);
    ESP_LOGI("[State AM] %d!", state_am);
    ESP_LOGI("[State VM] %d!", state_vm);
    ESP_LOGI("[State VD] %d!", state_vd);
    ESP_LOGI("============================================ [GPIOS] ============================================");

    // -> acender leds
    gpio_set_level(LED_AM, state_am);
    gpio_set_level(LED_VD, state_vd);
    gpio_set_level(LED_VM, state_vm);
    // -> Saída de tensão deve ser convertida na faixa de 0 a 4v
    // -> converter de 10bits (1024) para 8bits (0 a 255)
    // -> converter de 0 a 255 para 0 a 4v
    // -> resize(value_analog, min_input, );
    // -> Saída de corrente deve ser convertida na faixa de 1 a 4v
    // -> converter de 10bits (1024) para 8bits (0 a 255)
    // -> converter de 0 a 255 para 1 a 4v
    return;
}

// -> [ CONFIGURE_LED ] -> Configura os dispositivos de entrada e saída
static void configure_gpios(void)
{
    gpio_reset_pin(LED_AM);
    gpio_reset_pin(LED_VD);
    gpio_reset_pin(LED_VM);

    gpio_set_direction(LED_AM, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_VD, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_VM, GPIO_MODE_OUTPUT);
}

void app_main(void)
{

    // -> chamando a função de configuração para os pinos usados
    configure_gpios();

    while (1)
    {
        // -> leitura do pino analogico e aletação dos estados de saída
        compare_analog_read();

        if (value_analog >= 1024)
            value_analog = 0;
        else
            value_analog += 10;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

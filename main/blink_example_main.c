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
const float VALUE_REF_IN = 0.3;

// -> valores de referencia para temperatura
const int VALUE_REF_100_ = 0;
const int VALUE_REF_0 = 128;
const int VALUE_REF_400 = 694;
const int VALUE_REF_500 = 820;

static uint8_t s_led_state = 0;
int value_analog = 0;
// -> valor da entrada de referencia convertido em 10bits
const int min_input = VALUE_REF_IN * 204.8;
const int adcRef[4] = {VALUE_REF_100_ + min_input, VALUE_REF_0 + min_input, VALUE_REF_400 + min_input, VALUE_REF_500 + min_input};

static void compare_analog_read()
{
    // -> decisão de qual led acender
    bool state_vm = value_analog <= adcRef[0] || value_analog >= adcRef[3];

    bool state_am = (value_analog > adcRef[0] && value_analog < adcRef[1]) || (value_analog < adcRef[3] && value_analog > adcRef[2]);

    bool state_vd = value_analog > adcRef[1] && value_analog <= adcRef[2];
    // -> logs dos estados
    ESP_LOGI(TAG, "[Analog Read] %d!", value_analog);
    ESP_LOGI(TAG, "[State AM] %d!", state_am);
    ESP_LOGI(TAG, "[State VM] %d!", state_vm);
    ESP_LOGI(TAG, "[State VD] %d!", state_vd);
    // -> acender leds
    gpio_set_level(LED_AM, state_am);
    gpio_set_level(LED_VD, state_vd);
    gpio_set_level(LED_VM, state_vm);

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
        // -> apenas um LOG
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

        compare_analog_read();

        if (value_analog >= 1024)
            value_analog = 0;
        else
            value_analog += 10;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

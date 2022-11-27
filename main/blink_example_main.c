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
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = " leitura -> ";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
// -> definição dos pinos
#define LED_VM 15
#define LED_VD 4
#define LED_AM 2
#define LEDC_OUTPUT_VOLTAGE 12
#define LEDC_OUTPUT_CURRENT 13

// -> definição constantes PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_2
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// -> definição constantes leitura analogica
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_11

static int adc_raw[2][10];

// -> valores de referencia para leitura ADC
const float VALUE_REF_IN_MIN = 0.35;
const float VALUE_REF_IN_MAX = 2.2;

const float VALUE_ADC_IN_MIN = 0.15;
const float VALUE_ADC_IN_MAX = 2.45;
const float VALUE_ADC_SCALE = 4095;

// -> valores de referencia para temperatura
const int VALUE_REF_100_ = 454;
const int VALUE_REF_0 = 1047;
const int VALUE_REF_400 = 3243;
const int VALUE_REF_500 = 3749;

int value_analog = 0;
int min_input = 0;
int max_input = 0;

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};
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

static void write_analogic_out(int value, int channel)
{

    // -> valor de referencia para duty cycle 4095*2 = tempo máximo de ciclo
    int duty = resize(value, 0, 255, 0, 4095 * 2);

    // -> atualiza valor do duty cycle
    ledc_set_duty(LEDC_MODE, channel, duty);
    // -> aplica novos valores de duty cycle
    ledc_update_duty(LEDC_MODE, channel);

    return;
}

static void compare_analog_read()
{
    // -> valor mínimo de leitura do pino analogico [deslocamento de zero para o valor de referencia]
    min_input = resize(VALUE_REF_IN_MIN, VALUE_ADC_IN_MIN, VALUE_ADC_IN_MAX, 0, VALUE_ADC_SCALE);
    max_input = resize(VALUE_REF_IN_MAX, VALUE_ADC_IN_MIN, VALUE_ADC_IN_MAX, 0, VALUE_ADC_SCALE);

    // -> valores de referencia convertido em 8bits
    // -> corrente variando de 1 a 5 volts
    const int min_out_current = resize(min_input, min_input, max_input, 51, 255);
    const int max_out_current = resize(max_input, min_input, max_input, 51, 255);

    const int min_out_voltage = resize(min_input, min_input, max_input, 0, 255);
    const int max_out_voltage = resize(max_input, min_input, max_input, 0, 255);

    // -> vetor que contem as referencias de temperatura
    const int adcRef[4] = {VALUE_REF_100_ + min_input, VALUE_REF_0 + min_input, VALUE_REF_400 + min_input, VALUE_REF_500 + min_input};

    // -> decisão de qual led acender
    bool state_vm = value_analog <= adcRef[0] || value_analog >= adcRef[3];
    bool state_am = (value_analog > adcRef[0] && value_analog < adcRef[1]) || (value_analog < adcRef[3] && value_analog > adcRef[2]);
    bool state_vd = value_analog > adcRef[1] && value_analog <= adcRef[2];

    int out_current = value_analog < min_input ? min_out_current : value_analog > max_input ? max_out_current
                                                                                            : resize(value_analog, min_input, max_input, min_out_current, max_out_current);
    float out_current_v = resize(out_current, 0, 255, 0, 3.3); // -> convert to out 0 <-> 5volts
    // -> alterar para 0 <-> 3.3volts

    int out_voltage = value_analog < min_input ? min_out_voltage : value_analog > max_input ? max_out_voltage
                                                                                            : resize(value_analog, min_input, max_input, min_out_voltage, max_out_voltage);
    float out_voltage_v = resize(out_voltage, 0, 255, 0, 3.3); // -> convert to out 0 <-> 5volts
    // -> alterar para 0 <-> 3.3volts

    // -> logs dos estados

    ESP_LOGI(TAG, "============================================ [GPIOS] ============================================");
    ESP_LOGI(TAG, "[Input Ref. Min.] %d", min_input);
    ESP_LOGI(TAG, "[Input Ref. Max.] %d", max_input);
    ESP_LOGI(TAG, "[Out Ref. Min. Voltage] %d", min_out_voltage);
    ESP_LOGI(TAG, "[Out Ref. Max. Voltage] %d", max_out_voltage);
    ESP_LOGI(TAG, "[Out Ref. Min. Current] %d", min_out_current);
    ESP_LOGI(TAG, "[Out Ref. Max. Current] %d", max_out_current);
    ESP_LOGI(TAG, "[Out Current] %d", out_current);
    ESP_LOGI(TAG, "[Out Voltage] %d", out_voltage);
    ESP_LOGI(TAG, "[Out Current[V]] %f", out_current_v);
    ESP_LOGI(TAG, "[Out Voltage[V]] %f", out_voltage_v);
    ESP_LOGI(TAG, "[Analog Read] %d", value_analog);
    ESP_LOGI(TAG, "[State AM] %d", state_am);
    ESP_LOGI(TAG, "[State VM] %d", state_vm);
    ESP_LOGI(TAG, "[State VD] %d", state_vd);
    for (int i = 0; i < 4; i++)
    {
        ESP_LOGI(TAG, "[Ref. %d] %d", i, adcRef[i]);
    }
    ESP_LOGI(TAG, "============================================ [GPIOS] ============================================");

    // -> acender leds
    gpio_set_level(LED_AM, state_am);
    gpio_set_level(LED_VD, state_vd);
    gpio_set_level(LED_VM, state_vm);

    // -> settar saidas analogicas
    write_analogic_out(out_current, LEDC_CHANNEL);
    write_analogic_out(out_current, LEDC_CHANNEL);

    // -> Saída de tensão deve ser convertida na faixa de 0 a 4v
    // -> converter de 10bits (1024) para 8bits (0 a 255)
    // -> converter de 0 a 255 para 0 a 4v
    // -> resize(value_analog, min_input, );
    // -> Saída de corrente deve ser convertida na faixa de 1 a 4v
    // -> converter de 10bits (1024) para 8bits (0 a 255)
    // -> converter de 0 a 255 para 1 a 4v
    return;
}

static void configure_analogic_out(void)
{
    // Set timmer do PWM
    ledc_timer_config_t analogic_out_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&analogic_out_timer);

    // Set PWM
    ledc_channel_config_t out_current_channel = {
        .speed_mode = LEDC_MODE,         // -> velocidade do led
        .channel = LEDC_CHANNEL,         // -> cannal de comutação dutty
        .timer_sel = LEDC_TIMER,         // -> timmer usado para gerar o PWM
        .intr_type = LEDC_INTR_DISABLE,  // -> interrupção[desabilitada]
        .gpio_num = LEDC_OUTPUT_CURRENT, // -> pino onde o led está conectado
        .duty = 4095,                    // Set duty to 50%
        .hpoint = 0};
    ledc_channel_config(&out_current_channel);

    ledc_channel_config_t out_voltage_channel = {
        .speed_mode = LEDC_MODE,         // -> velocidade do led
        .channel = LEDC_CHANNEL,         // -> cannal de comutação dutty
        .timer_sel = LEDC_TIMER,         // -> timmer usado para gerar o PWM
        .intr_type = LEDC_INTR_DISABLE,  // -> interrupção[desabilitada]
        .gpio_num = LEDC_OUTPUT_VOLTAGE, // -> pino onde o led está conectado
        .duty = 4095,                    // Set duty to 50%
        .hpoint = 0};
    ledc_channel_config(&out_voltage_channel);

    return;
}

static void configure_analogic_in(void)
{
    //-------------ADC1 Init---------------//

    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    //-------------ADC1 Config---------------//
    adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config);
}

static void read_analogic_in()
{
    adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]);
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0] + min_input);
    value_analog = adc_raw[0][0] + min_input;
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
    configure_analogic_out();
    configure_analogic_in();

    while (1)
    {
        // -> leitura do pino analogico e aletação dos estados de saída
        compare_analog_read();
        read_analogic_in();
        // if (value_analog >= VALUE_ADC_SCALE)
        //     value_analog = 0;
        // else
        //     value_analog += 50;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

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

// -> definição constantes para configuração do pino PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_2
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// -> definição constantes para configuração do pino ADC
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_11

// -> vetor para armazenar valor lido no pino ADC
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

// -> inicialização de variaveis globais para referencia de mínimo e máximo nas leituas ADC
int min_input = 0;
int max_input = 0;

// -> Inicialização do ADC
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};

// -> Função para conversão de faixas de valores
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
    // -> converte o valor de duty de 8bits para 12bits
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
    // -> Reajuste do offset [deslocamento de zero para o valor de referencia]
    min_input = resize(VALUE_REF_IN_MIN, VALUE_ADC_IN_MIN, VALUE_ADC_IN_MAX, 0, VALUE_ADC_SCALE);
    max_input = resize(VALUE_REF_IN_MAX, VALUE_ADC_IN_MIN, VALUE_ADC_IN_MAX, 0, VALUE_ADC_SCALE);

    // -> calcula os valores de referencia para corrente em 8bits
    const int min_out_current = resize(min_input, min_input, max_input, 38, 193);
    const int max_out_current = resize(max_input, min_input, max_input, 38, 193);

    // -> calcula os valores de referencia para tensao em 8bits
    const int min_out_voltage = resize(min_input, min_input, max_input, 0, 255);
    const int max_out_voltage = resize(max_input, min_input, max_input, 0, 255);

    // -> Vetor com as referencias de temperatura usadas na sinalização, e um adicional do offset, para reajustar a comparação
    const int adcRef[4] = {VALUE_REF_100_ + min_input, VALUE_REF_0 + min_input, VALUE_REF_400 + min_input, VALUE_REF_500 + min_input};

    // -> Lógica booleana que decide qual led deve ser acesso
    bool state_vm = value_analog <= adcRef[0] || value_analog >= adcRef[3];
    bool state_am = (value_analog > adcRef[0] && value_analog < adcRef[1]) || (value_analog < adcRef[3] && value_analog > adcRef[2]);
    bool state_vd = value_analog > adcRef[1] && value_analog <= adcRef[2];

    // -> atualiza valor do duty cycle para a saída de tensão, com travamento de limites maximos e mínimos [0.5 a 2.5v]
    int out_current = value_analog < min_input ? min_out_current : value_analog > max_input ? max_out_current
                                                                                            : resize(value_analog, min_input, max_input, min_out_current, max_out_current);
    // -> faz a conversão do valor calculado em 8bits para 0 a 3.3v, apenas para log
    float out_current_v = resize(out_current, 0, 255, 0.5, 2.5);

    // -> atualiza valor do duty cycle para a saída de tensão, com travamento de limites maximos e mínimos [0 a 3.3v]
    int out_voltage = value_analog < min_input ? min_out_voltage : value_analog > max_input ? max_out_voltage
                                                                                            : resize(value_analog, min_input, max_input, min_out_voltage, max_out_voltage);
    // -> faz a conversão do valor calculado em 8bits para 0 a 3.3v, apenas para log
    float out_voltage_v = resize(out_voltage, 0, 255, 0, 3.3);

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

    return;
}

// -> Faz a configuração e inicia os pinos de pwm [Saídas de tensão e corrente]
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
        .duty = 0,                       // Set duty to 50%
        .hpoint = 0};
    ledc_channel_config(&out_current_channel);

    ledc_channel_config_t out_voltage_channel = {
        .speed_mode = LEDC_MODE,         // -> velocidade do led
        .channel = LEDC_CHANNEL,         // -> cannal de comutação dutty
        .timer_sel = LEDC_TIMER,         // -> timmer usado para gerar o PWM
        .intr_type = LEDC_INTR_DISABLE,  // -> interrupção[desabilitada]
        .gpio_num = LEDC_OUTPUT_VOLTAGE, // -> pino onde o led está conectado
        .duty = 0,                       // Set duty to 50%
        .hpoint = 0};
    ledc_channel_config(&out_voltage_channel);

    return;
}

// -> Faz configuração do pino ADC [Entrada Analógica]
static void configure_analogic_in(void)
{
    // -> inicia o adc
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    // -> configura o adc
    adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config);
}

// -> Faz a leitura do pino analogico
static void read_analogic_in()
{
    adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]);                                              // -> leitura
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0] + min_input); // -> log
    value_analog = adc_raw[0][0] + min_input;                                                                       // -> seta a constante de monitoramento do pino analogico com o valor lido
    return;
}

// -> Configura os leds como dispositos digitais de saída
static void configure_gpios(void)
{
    // -> reseta os pinos
    gpio_reset_pin(LED_AM);
    gpio_reset_pin(LED_VD);
    gpio_reset_pin(LED_VM);

    // -> configura os pinos como saída
    gpio_set_direction(LED_AM, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_VD, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_VM, GPIO_MODE_OUTPUT);
}

void app_main(void)
{

    // -> funções de configuração
    configure_gpios();
    configure_analogic_out();
    configure_analogic_in();

    while (1)
    {
        // -> funções de execução
        read_analogic_in();
        compare_analog_read();

        // -> frequencia de execução do loop
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

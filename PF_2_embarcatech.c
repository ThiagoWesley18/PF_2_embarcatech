#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <math.h>
#include <ctype.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "display_oled.h"
#include "inc/ssd1306.h"
#include "neopixel.c"

#define SCAN_INTERVAL_MS 2000

// Pino e número de LEDs da matriz de LEDs.
#define LED_PIN 7
#define LED_COUNT 25

// Pino e canal do microfone no ADC.
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)

// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV 96.f
#define SAMPLES 200 // Número de amostras que serão feitas do ADC.
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) // Ajuste do valor do ADC para Volts.
#define ADC_MAX 3.3f
#define ADC_STEP (3.3f/5.f) // Intervalos de volume do microfone.
#define abs(x) ((x < 0) ? (-x) : (x))

// Configure o SSID e senha da sua rede
const char ssid[] = SSID_HOST;
const char password[] = PASSWORD_HOST;
const char target_ssid[] = "A54 de Thiago";

// Pinos do I2C para o display OLED
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

volatile bool found_target =  false;
volatile bool scan_active = false;
int scan_result = 0;

// Canal e configurações do DMA
uint dma_channel;
dma_channel_config dma_cfg;

// Buffer de amostras do ADC.
uint16_t adc_buffer[SAMPLES];

void sample_mic();
float mic_power();
uint8_t get_intensity(float v);

void wifi_setup();
void mic_setup();
void oled_setup();
bool repeating_timer_callback(struct repeating_timer *t);
static int scan_callback(void *env, const cyw43_ev_scan_result_t *result);


int main() {
    stdio_init_all();
    sleep_ms(2000); // Aguarda UART

    mic_setup();
    oled_setup();
    wifi_setup();

    connecte_oled(0);

    struct repeating_timer timer;
    add_repeating_timer_ms(SCAN_INTERVAL_MS, repeating_timer_callback, NULL, &timer);

    printf("\n----\nIniciando loop...\n----\n");
    while (true) {
        // Realiza uma amostragem do microfone.
        sample_mic();

        // Pega a potência média da amostragem do microfone.
        float avg = mic_power();
        avg = 2.f * abs(ADC_ADJUST(avg)); // Ajusta para intervalo de 0 a 3.3V. (apenas magnitude, sem sinal)

        uint intensity = get_intensity(avg); // Calcula intensidade do volume.

        start_Matrix(&intensity);

        if(intensity >= 6){
            scan_result++;
            printf("scan_result: %d\n", scan_result);
        }
         // Flag para ativar ou desativar o scan por voz.
        if(scan_result >= 10){
            scan_active = true;
            scan_result = 0;
        }
         
    }
    cyw43_arch_deinit();
    return 0;
}



// Callback chamado ao final do scan Wi-Fi
static int scan_callback(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        if (strcmp(result->ssid, target_ssid) == 0) {
            found_target = true;  // Rede encontrada
        }
    }
    return 0;     
}

// Callback do timer que inicia a varredura Wi-Fi
bool repeating_timer_callback(struct repeating_timer *t) {
    if (!cyw43_wifi_scan_active(&cyw43_state)) { 
        if(scan_active){
            // O scan terminou, podemos processar os resultados
            if (found_target) {
                //printf("Rede '%s' encontrada!\n", target_ssid);
                connecte_oled(1);
            } else {
                connecte_oled(2);
                //printf("Rede '%s' não encontrada!\n", target_ssid);
            }

            found_target = false; // Reseta para a próxima varredura

            // Inicia um novo scan se nenhum scan estiver ativo
            cyw43_wifi_scan_options_t scan_options = {0};
            int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_callback);
            if (err != 0) {
                printf("Erro ao iniciar scan: %d\n", err);
            }
        }else{
            connecte_oled(0);
        }
    }

    return true; 
}


void mic_setup(){
    // Preparação do ADC.
    printf("Preparando ADC...\n");

    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);
    adc_set_clkdiv(ADC_CLOCK_DIV);
    adc_fifo_setup(
        true, // Habilitar FIFO
        true, // Habilitar request de dados do DMA
        1, // Threshold para ativar request DMA é 1 leitura do ADC
        false, // Não usar bit de erro
        false // Não fazer downscale das amostras para 8-bits, manter 12-bits.
    );
    printf("ADC Configurado!\n\n");

    printf("Preparando DMA...");

    // Configurações do DMA.
    dma_channel = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // Tamanho da transferência é 16-bits (usamos uint16_t para armazenar valores do ADC)
    channel_config_set_read_increment(&dma_cfg, false); // Desabilita incremento do ponteiro de leitura (lemos de um único registrador)
    channel_config_set_write_increment(&dma_cfg, true); // Habilita incremento do ponteiro de escrita (escrevemos em um array/buffer)
    channel_config_set_dreq(&dma_cfg, DREQ_ADC); // Usamos a requisição de dados do ADC

    printf("Configuracoes completas!\n");
}

void oled_setup(){
    // Preparação da matriz de LEDs.
    printf("Preparando NeoPixel...");
    
    npInit(LED_PIN, LED_COUNT);

    // Inicialização do display OLED
    printf("Iniciando display OLED...\n");

    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

}
void wifi_setup(){
    printf("Iniciando Wifi...\n");

    // Configuração Wifi
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_BRAZIL)) {
        printf("Falha ao inicializar CYW43\n");
    }
    printf("CYW43 inicializado\n");

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar no Wi-Fi\n");
        cyw43_arch_deinit();
    }
    printf("Conectado ao Wi-Fi: %s\n", ssid);
}

/**
 * Realiza as leituras do ADC e armazena os valores no buffprintf("itensidade: %2d\n", intensity);er.
 */
void sample_mic() {
    adc_fifo_drain(); // Limpa o FIFO do ADC.
    adc_run(false); // Desliga o ADC (se estiver ligado) para configurar o DMA.
  
    dma_channel_configure(dma_channel, &dma_cfg,
      adc_buffer, // Escreve no buffer.
      &(adc_hw->fifo), // Lê do ADC.
      SAMPLES, // Faz SAMPLES amostras.
      true // Liga o DMA.
    );
  
    // Liga o ADC e espera acabar a leitura.
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    
    // Acabou a leitura, desliga o ADC de novo.
    adc_run(false);
  }
  
  /**
   * Calcula a potência média das leituras do ADC. (Valor RMS)
   */
  float mic_power() {
    float avg = 0.f;
  
    for (uint i = 0; i < SAMPLES; ++i)
      avg += adc_buffer[i] * adc_buffer[i];
    
    avg /= SAMPLES;
    return sqrt(avg);
  }
  
  /**
   * Calcula a intensidade do volume registrado no microfone, de 0 a 4, usando a tensão.
   */
  uint8_t get_intensity(float v) {
    uint count = 0;
  
    while ((v -= ADC_STEP/20) > 0.f)
      ++count;
    
    return count;
  }

#include <stdio.h>                       // Biblioteca para entrada/saida padrao: printf, sprintf, etc.
#include "pico/stdlib.h"                 // SDK do Pico
#include "hardware/i2c.h"                // I2C do RP2040
#include "hardware/pio.h"                // PIO do RP2040
#include "hardware/pwm.h"                // PWM do RP2040
#include "blink.pio.h"                   // Programa PIO para controlar a matriz de LEDs WS2812
#include "lib/bh1750_light_sensor.h"     // Biblioteca do sensor de luminosidade BH1750
#include "lib/ssd1306.h"                 // Biblioteca do display OLED SSD1306
#include "lib/font.h"                    // Fontes para o SSD1306
#include "FreeRTOS.h"                    // Kernel FreeRTOS
#include "FreeRTOSConfig.h"              // Configuracoes do FreeRTOS
#include "task.h"                        // Tarefas do FreeRTOS
#include "semphr.h"                      // Semaforos do FreeRTOS
#include "queue.h"                       // Filas do FreeRTOS
#include <math.h>                        // Biblioteca de funcoes matematicas

#define WS2812_PIN 7
#define BUZZER_PIN 21

// I2C para os sensores 
#define I2C_PORT i2c0                    // Porta I2C para os sensores
#define I2C_SDA 0                        // SDA para I2C_PORT
#define I2C_SCL 1                        // SCL para I2C_PORT

// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

#define GY33_I2C_ADDR 0x29

// Definicoes do sensor GY302
#define GY302_S0_PIN 8
#define GY302_S1_PIN 9
#define GY302_S2_PIN 10
#define GY302_S3_PIN 11
#define GY302_OUT_PIN 12

// Escalas de frequencia do GY302
#define GY302_POWER_DOWN 0x00
#define GY302_2_PERCENT   0x01
#define GY302_20_PERCENT  0x02
#define GY302_100_PERCENT 0x03

// Filtros de cor do GY302
#define GY302_RED_FILTER    0x00
#define GY302_GREEN_FILTER  0x01
#define GY302_BLUE_FILTER   0x02
#define GY302_CLEAR_FILTER  0x03

// Registros do sensor GY33
#define ENABLE_REG 0x80
#define ATIME_REG 0x81
#define CONTROL_REG 0x8F
#define ID_REG 0x92
#define STATUS_REG 0x93
#define CDATA_REG 0x94 //  "Clear"
#define RDATA_REG 0x96 //  "Red"
#define GDATA_REG 0x98 //  "Green"
#define BDATA_REG 0x9A //  "Blue"

ssd1306_t ssd;
PIO pio;
uint offset;
uint sm;

// Função para escrever um valor em um registro do GY-33
void gy33_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, GY33_I2C_ADDR, buffer, 2, false);
}

// Função para ler um valor de um registro do GY-33
uint16_t gy33_read_register(uint8_t reg)
{
    uint8_t buffer[2];
    i2c_write_blocking(I2C_PORT, GY33_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, GY33_I2C_ADDR, buffer, 2, false);
    return (buffer[1] << 8) | buffer[0]; // Combina os bytes em um valor de 16 bits
}

// Inicializa o GY-33
void gy33_init()
{
    gy33_write_register(ENABLE_REG, 0x03);  // Ativa o sensor (Power ON e Ativação do ADC)
    gy33_write_register(ATIME_REG, 0xF5);   // Tempo de integração (ajusta a sensibilidade) D5 => 103ms
    gy33_write_register(CONTROL_REG, 0x00); // Configuração de ganho padrão (1x) (pode ir até 60x)
}

// Lê os valores RGB e Clear do GY-33
void gy33_read_color(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    *c = gy33_read_register(CDATA_REG);
    *r = gy33_read_register(RDATA_REG);
    *g = gy33_read_register(GDATA_REG);
    *b = gy33_read_register(BDATA_REG);
}

// Seleciona a escala de frequencia do sensor GY302 (S0 e S1)
void gy302_set_frequency_scaling(uint8_t scaling)
{
    switch(scaling) {
        case GY302_POWER_DOWN:
            gpio_put(GY302_S0_PIN, 0);
            gpio_put(GY302_S1_PIN, 0);
            break;
        case GY302_2_PERCENT:
            gpio_put(GY302_S0_PIN, 0);
            gpio_put(GY302_S1_PIN, 1);
            break;
        case GY302_20_PERCENT:
            gpio_put(GY302_S0_PIN, 1);
            gpio_put(GY302_S1_PIN, 0);
            break;
        case GY302_100_PERCENT:
            gpio_put(GY302_S0_PIN, 1);
            gpio_put(GY302_S1_PIN, 1);
            break;
    }
}

// Seleciona o filtro de cor do sensor GY302 (S2 e S3)
void gy302_set_color_filter(uint8_t filter)
{
    switch(filter) {
        case GY302_RED_FILTER:
            gpio_put(GY302_S2_PIN, 0);
            gpio_put(GY302_S3_PIN, 0);
            break;
        case GY302_GREEN_FILTER:
            gpio_put(GY302_S2_PIN, 1);
            gpio_put(GY302_S3_PIN, 1);
            break;
        case GY302_BLUE_FILTER:
            gpio_put(GY302_S2_PIN, 0);
            gpio_put(GY302_S3_PIN, 1);
            break;
        case GY302_CLEAR_FILTER:
            gpio_put(GY302_S2_PIN, 1);
            gpio_put(GY302_S3_PIN, 0);
            break;
    }
}

// Conta pulsos no pino OUT do sensor GY302
uint32_t gy302_read_frequency(void)
{
    uint32_t pulse_count = 0;
    uint32_t start_time = time_us_32();
    uint32_t timeout = 100000; // 100ms timeout
    
    while ((time_us_32() - start_time) < timeout) {
        if (gpio_get(GY302_OUT_PIN) == 1) {
            pulse_count++;
            while (gpio_get(GY302_OUT_PIN) == 1) {
                if ((time_us_32() - start_time) >= timeout) break;
            }
        }
    }
    
    return pulse_count;
}

// Seleciona o filtro e executa leitura de frequencia
uint32_t gy302_read_color(uint8_t filter)
{
    gy302_set_color_filter(filter);
    sleep_ms(10); // Allow sensor to stabilize
    return gy302_read_frequency();
}

// Realiza leituras para RED, GREEN, BLUE e CLEAR e as grava nos ponteiros fornecidos
void gy302_read_all_colors(uint32_t *red, uint32_t *green, uint32_t *blue, uint32_t *clear)
{
    *red = gy302_read_color(GY302_RED_FILTER);
    *green = gy302_read_color(GY302_GREEN_FILTER);
    *blue = gy302_read_color(GY302_BLUE_FILTER);
    *clear = gy302_read_color(GY302_CLEAR_FILTER);
}

// Inicializa o GY302
void gy302_init(void)
{
    gpio_init(GY302_S0_PIN);
    gpio_init(GY302_S1_PIN);
    gpio_init(GY302_S2_PIN);
    gpio_init(GY302_S3_PIN);
    gpio_init(GY302_OUT_PIN);
    
    gpio_set_dir(GY302_S0_PIN, GPIO_OUT);
    gpio_set_dir(GY302_S1_PIN, GPIO_OUT);
    gpio_set_dir(GY302_S2_PIN, GPIO_OUT);
    gpio_set_dir(GY302_S3_PIN, GPIO_OUT);
    gpio_set_dir(GY302_OUT_PIN, GPIO_IN);
    
    gpio_pull_up(GY302_OUT_PIN);
    
    gy302_set_frequency_scaling(GY302_20_PERCENT);
    gy302_set_color_filter(GY302_CLEAR_FILTER);
}

// Inicializacoes de perifericos e módulos
void setup()
{
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    
    // I2C do Display pode ser diferente dos sensores. Funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa o I2C0
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o sensor de luz BH1750
    bh1750_power_on(I2C_PORT);

    pio = pio0;
    offset = pio_add_program(pio, &blink_program);
    sm = pio_claim_unused_sm(pio, true);
    blink_program_init(pio, sm, offset, WS2812_PIN);
}

int main()
{
    stdio_init_all();
    setup();
    
    char str_lux[10]; // Buffer para armazenar a string

    printf("Iniciando GY-33...\n");
    gy33_init();

    // Inicializa o sensor GY302
    printf("Iniciando GY302...\n");
    gy302_init();

    // Le todas as cores
    uint32_t red, green, blue, clear;
    gy302_read_all_colors(&red, &green, &blue, &clear);

    // Le cor especifica
    uint32_t light_level = gy302_read_color(GY302_CLEAR_FILTER);

    // Muda a escala de frequencia do GY302
    gy302_set_frequency_scaling(GY302_100_PERCENT);
    char str_red[5]; // Buffer para armazenar a string
    char str_green[5];
    char str_blue[5];
    char str_clear[5];

    bool cor = true;
    while (true)
    {
        uint16_t r, g, b, c;
        gy33_read_color(&r, &g, &b, &c);
        printf("Cor detectada - R: %d, G: %d, B: %d, Clear: %d\n", r, g, b, c);

        sprintf(str_red, "%d R", r); // Converte o inteiro em string
        sprintf(str_green, "%d G", g);
        sprintf(str_blue, "%d B", b);
        sprintf(str_clear, "%d C", c);

        // Leitura do sensor de Luz BH1750
        uint16_t lux = bh1750_read_measurement(I2C_PORT);
        printf("Lux = %d\n", lux);

        sprintf(str_lux, "%d Lux", lux); // Converte o inteiro em string

        //cor = !cor;
        //  Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);                               // Limpa o display

        // Cabeçalho
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);      
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 14);       

        // Linhas separadoras
        ssd1306_line(&ssd, 3, 24, 123, 24, cor);               
        ssd1306_line(&ssd, 3, 40, 123, 40, cor);                

        // Sensor BH1750 
        ssd1306_draw_string(&ssd, "BH1750:", 8, 28);           
        ssd1306_draw_string(&ssd, str_lux, 70, 28);            

        // Sensor GY33 
        ssd1306_draw_string(&ssd, "GY33:", 8, 46);          
        //ssd1306_draw_string(&ssd, str_gy33, 70, 46);         

        // Atualiza o display
        ssd1306_send_data(&ssd);

        sleep_ms(500);
    }

    vTaskStartScheduler();
    panic_unsupported();
}

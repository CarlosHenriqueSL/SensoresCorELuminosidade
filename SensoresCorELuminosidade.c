#include <stdio.h>                       // Biblioteca para entrada/saida padrao: printf, sprintf, etc.
#include "pico/stdlib.h"                 // SDK do Pico
#include "hardware/i2c.h"                // I2C do RP2040
#include "hardware/pio.h"                // PIO do RP2040
#include "hardware/clocks.h"             // Clocks do RP2040
#include "matriz_LED.pio.h"              // Programa PIO para controlar a matriz de LEDs WS2812
#include "lib/bh1750_light_sensor.h"     // Biblioteca do sensor de luminosidade BH1750
#include "lib/ssd1306.h"                 // Biblioteca do display OLED SSD1306
#include "lib/font.h"                    // Fontes para o SSD1306

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
float red, green, blue;

double apagar_leds[25][3] = // Apagar LEDs da matriz
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

double desenho_cores[25][3]; // Desenho de acordo a leitura do gy33

// Função para atualizar a matriz de cores com base nas leituras do GY33
void atualizar_matriz_cores(double matriz[25][3], uint16_t r, uint16_t g, uint16_t b) {
    // Normaliza os valores RGB (0-65535) para o range 0.0-1.0
    double red_norm = (double)r / 65535.0;
    double green_norm = (double)g / 65535.0;
    double blue_norm = (double)b / 65535.0;
    
    printf("Valores normalizados - R: %.2f, G: %.2f, B: %.2f\n", red_norm, green_norm, blue_norm);
    // Preenche toda a matriz com a cor detectada
    for (int i = 0; i < 25; i++) {
        matriz[i][0] = red_norm;   // Red
        matriz[i][1] = green_norm; // Green
        matriz[i][2] = blue_norm;  // Blue
    }
}

// Função para controlar o buzzer baseado no nível de Lux
void buzzer_lux(uint16_t lux) {
    static uint32_t ultimo_beep = 0;
    uint32_t tempo_atual = time_us_32() / 1000; // Converte para milissegundos
    uint32_t intervalo_beep;
    
    // Define o intervalo do beep baseado no nível de Lux
    if (lux > 1000) {
        intervalo_beep = 100;  // Muito brilhante (100ms)
    } else if (lux > 500) {
        intervalo_beep = 300;  // Brilhante (300ms)
    } else if (lux > 200) {
        intervalo_beep = 600;  // Moderado (600ms)
    } else if (lux > 50) {
        intervalo_beep = 1000; // Baixo (1s)
    } else {
        intervalo_beep = 2000; // Muito baixo (2s)
    }
    
    // Verifica se é hora de fazer o beep
    if (tempo_atual - ultimo_beep >= intervalo_beep) {
        gpio_put(BUZZER_PIN, 1);  // Liga o buzzer
        sleep_ms(50);             // Duração do beep (50ms)
        gpio_put(BUZZER_PIN, 0);  // Desliga o buzzer
        ultimo_beep = tempo_atual;
    }
}

// Função para converter RGB em um valor de 32 bits
uint matrix_rgb(float r, float g, float b)
{
    unsigned char R, G, B;
    R = r * 255;
    G = g * 255;
    B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Função para desenhar na matriz com verificações de segurança
void desenho_pio(double desenho[25][3], uint32_t valor_led, PIO pio, uint sm)
{   
    // Envia dados para a matriz de LEDs com timeout
    for (int16_t i = 0; i < 25; i++)
    {
        valor_led = matrix_rgb(desenho[i][0], desenho[i][1], desenho[i][2]);
        
        // Timeout simples para evitar travamentos
        uint32_t timeout = 1000; // 1000 tentativas
        while (pio_sm_is_tx_fifo_full(pio, sm) && timeout > 0) {
            sleep_us(1);
            timeout--;
        }
        
        if (timeout > 0) {
            pio_sm_put(pio, sm, valor_led);
        } else {
            printf("Timeout no LED %d\n", i);
            break; // Sai do loop se houver timeout
        }
    }
}

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

    // Inicializa o PIO para controle dos LEDs WS2812
    gpio_init(WS2812_PIN);          
    gpio_set_dir(WS2812_PIN, GPIO_OUT);
    bool frequenciaClock;                                    // Variável para a frequência de clock
    frequenciaClock = set_sys_clock_khz(128000, false);      // frequência de clock de 128MHz
    pio = pio0;
    offset = pio_add_program(pio, &matriz_LED_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_sm_set_enabled(pio, sm, true);                       // Habilita a máquina de estado
    matriz_LED_program_init(pio, sm, offset, WS2812_PIN);
}

int main()
{
    stdio_init_all();
    printf("Iniciando sistema...\n");

    // Aguarda 2 segundos para inicialização
    sleep_ms(2000);

    setup();
    printf("Setup concluído!\n");

    // Apaga os LEDs da matriz inicialmente
    desenho_pio(apagar_leds, 0, pio, sm);
    printf("LEDs apagados\n");

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
    uint32_t loop_counter = 0;
    
    while (true)
    {        
        uint16_t r, g, b, c;
        gy33_read_color(&r, &g, &b, &c);

        // Atualiza a matriz de cores com base na leitura do GY33
        atualizar_matriz_cores(desenho_cores, r, g, b);

        sprintf(str_red, "R:%d", r); // Converte o inteiro em string
        sprintf(str_green, "G:%d", g);
        sprintf(str_blue, "B:%d", b);
        sprintf(str_clear, "C:%d", c);

        // Leitura do sensor de Luz BH1750
        uint16_t lux = bh1750_read_measurement(I2C_PORT);

        // Controla o buzzer baseado no nível de Lux
        buzzer_lux(lux);

        sprintf(str_lux, "%d Lux", lux); // Converte o inteiro em string

        //cor = !cor;
        //  Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);                               // Limpa o display

        // Cabeçalho
        ssd1306_draw_string(&ssd, "SENSORES", 30, 6);        

        // Linhas separadoras
        ssd1306_line(&ssd, 3, 18, 123, 18, cor);               
        ssd1306_line(&ssd, 3, 33, 123, 33, cor);                

        // Sensor BH1750 
        ssd1306_draw_string(&ssd, "BH1750:", 5, 22);           
        ssd1306_draw_string(&ssd, str_lux, 65, 22);            

        // Sensor GY33 
        ssd1306_draw_string(&ssd, "GY33:", 5, 40);
        ssd1306_draw_string(&ssd, str_red, 8, 50);
        ssd1306_draw_string(&ssd, str_green, 43, 50);
        ssd1306_draw_string(&ssd, str_blue, 78, 50);
        ssd1306_draw_string(&ssd, str_clear, 78, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        // Desenha na matriz de LEDs com a cor detectada pelo GY33
        desenho_pio(desenho_cores, 0, pio, sm);

        sleep_ms(500);
    }
}

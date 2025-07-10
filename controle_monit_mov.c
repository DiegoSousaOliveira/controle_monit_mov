#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"

// --- I2C para o OLED (SSD1306)
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

// Área do display
struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};
uint8_t ssd[ssd1306_buffer_length];

// --- Configuração do MPU6050 (I2C0)
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define MPU6050_ADDR 0x68

// --- Configuração do Servo no GPIO 8 (PWM)
#define SERVO_PIN 8
#define SERVO_MIN 500    // 0º
#define SERVO_MAX 2500   // 180º

// --- Funções Display
void clean_display(uint8_t ssd[ssd1306_buffer_length]) {
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
}

void write_display(char *text[], size_t num_lines, uint8_t ssd[ssd1306_buffer_length]) {
    int y = 0;
    for (uint i = 0; i < num_lines; i++) {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}

// --- Inicialização do MPU6050
void mpu6050_init() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

int16_t read_word(uint8_t reg) {
    uint8_t data[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
    return (data[0] << 8) | data[1];
}

// --- Controle do servo motor
void init_servo_pwm() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_wrap(slice, 20000); // 50Hz
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_enabled(slice, true);
}

void set_servo_angle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint pulse_width = SERVO_MIN + (angle / 180.0f) * (SERVO_MAX - SERVO_MIN);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_gpio_level(SERVO_PIN, pulse_width);
}

int main() {
    stdio_init_all();

    // Inicializa I2C1 para display OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa display OLED
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    clean_display(ssd);

    // Inicializa I2C0 para MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Inicializa periféricos
    sleep_ms(2000);
    mpu6050_init();
    init_servo_pwm();

    printf("Sistema iniciado com sucesso.\n");

    while (true) {
        int16_t acc_x = read_word(0x3B);
        int16_t acc_y = read_word(0x3D);
        int16_t acc_z = read_word(0x3F);

        float ax = acc_x / 16384.0f;
        float ay = acc_y / 16384.0f;
        float az = acc_z / 16384.0f;

        float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
        float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;

        // Atualiza servo com base no pitch
        float servo_angle = pitch + 90;
        set_servo_angle(servo_angle);

        // --- Atualiza display OLED
        char linha1[32], linha2[32], linha3[32];
        char *linhas[3] = {linha1, linha2, linha3};

        snprintf(linha1, sizeof(linha1), "Pitch: %.1f", pitch);
        snprintf(linha2, sizeof(linha2), "Roll : %.1f", roll);

        if (fabs(pitch) > 45.0f) {
            snprintf(linha3, sizeof(linha3), "ALERTA: inclinacao!");
        } else {
            snprintf(linha3, sizeof(linha3), "Servo: %.0f", servo_angle);
        }

        clean_display(ssd);
        write_display(linhas, 3, ssd);

        sleep_ms(300);
    }
}

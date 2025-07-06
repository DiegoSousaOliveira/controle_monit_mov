#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

#define I2C_PORT i2c1
#define SDA_PIN 3
#define SCL_PIN 2
#define MPU6050_ADDR 0x68

void mpu6050_init() {
    uint8_t buf[] = {0x6B, 0x00}; // registrador PWR_MGMT_1 para 0 (acorda o sensor)
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

int16_t read_word(uint8_t reg) {
    uint8_t data[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
    return (data[0] << 8) | data[1];
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000); // 400KHz

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(5000);
    mpu6050_init();
    printf("MPU6050 iniciando\n");

    while (true) {
        int16_t acc_x = read_word(0x3B);
        int16_t acc_y = read_word(0x3D);
        int16_t acc_z = read_word(0x3F);

        printf("\nacc_x: %d, acc_y: %d, acc_z: %d\n", acc_x, acc_y, acc_z);


        // Normalização (valores típicos: ±16384 para ±2g)
        float ax = acc_x / 16384.0f;
        float ay = acc_y / 16384.0f;
        float az = acc_z / 16384.0f;

        // Calculo inclinação (pitch e roll)
        float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        printf("Inclinação: Pitch = %.2f°\n", pitch);
        printf("Inclinação: Roll = %.2f°\n", roll);
        
        sleep_ms(1000);
    }
}

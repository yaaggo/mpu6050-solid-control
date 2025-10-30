#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "../include/mpu6050.h"

#define BOOTSEL_BUTTON_PIN 6

mpu6050_t mpu;

// Quaternion [w, x, y, z]
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float beta = 0.1f; // Ganho do filtro Madgwick

absolute_time_t last_time = 0;

volatile bool button_pressed = false;
volatile absolute_time_t last_button_time = 0;

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == BOOTSEL_BUTTON_PIN) {
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_button_time, now) > 200000) {
            last_button_time = now;
            button_pressed = true;
        }
    }
}

void setup_bootsel_button() {
    gpio_init(BOOTSEL_BUTTON_PIN);
    gpio_set_dir(BOOTSEL_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BOOTSEL_BUTTON_PIN);
    gpio_set_irq_enabled_with_callback(BOOTSEL_BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Filtro Madgwick para fusão de sensores com quaternions
void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Converte graus/s para rad/s
    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // Taxa de mudança do quaternion baseado no giroscópio
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // Normaliza acelerômetro
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Variáveis auxiliares
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    // Gradiente descendente
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
    
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Aplica feedback
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integra
    q[0] += qDot1 * dt;
    q[1] += qDot2 * dt;
    q[2] += qDot3 * dt;
    q[3] += qDot4 * dt;

    // Normaliza quaternion
    recipNorm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

// Converte quaternion para ângulos de Euler
void quaternion_to_euler(float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
    else
        *pitch = asinf(sinp) * 180.0f / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

int main() {
    stdio_init_all();
    sleep_ms(4000);

    setup_bootsel_button();
    printf("Botao BOOTSEL configurado no GPIO 6\n");

    if (!mpu6050_init(&mpu)) {
        printf("erro ao inicializar MPU6050\n");
        return 1;
    }

    printf("Calibrando... mantenha o sensor parado e nivelado!\n");
    mpu6050_calibrate(&mpu, 1000);
    printf("Calibrado!\n");
    
    sleep_ms(500);
    last_time = get_absolute_time();

    while (true) {
        if (button_pressed) {
            printf("\nEntrando em modo BOOTSEL...\n");
            sleep_ms(100);
            reset_usb_boot(0, 0);
        }

        mpu6050_data_t data;
        if (mpu6050_read_data(&mpu, &data)) {
            absolute_time_t now = get_absolute_time();
            float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;
            last_time = now;
            
            if (dt > 0.1f) dt = 0.02f;

            // Atualiza quaternion com filtro Madgwick
            madgwick_update(
                data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps,
                data.accel_x_g, data.accel_y_g, data.accel_z_g,
                dt
            );

            // Converte para ângulos de Euler
            float roll, pitch, yaw;
            quaternion_to_euler(&roll, &pitch, &yaw);

            printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
        }
        sleep_ms(20);
    }
}
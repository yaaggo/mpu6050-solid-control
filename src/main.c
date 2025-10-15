#include <stdio.h>
#include "pico/stdlib.h"
#include "../include/mpu6050.h"

mpu6050_t mpu;

float pitch_f = 0.0f;
float roll_f = 0.0f;
float alpha = 0.98f;

int main() {
    stdio_init_all();
    sleep_ms(4000);

    if (!mpu6050_init(&mpu)) {
        printf("erro ao inicializar MPU6050\n");
        return 1;
    }

    mpu6050_calibrate(&mpu, 500);
    printf("calibrado\n");

    while (true) {
        mpu6050_data_t data;
        if (mpu6050_read_data(&mpu, &data)) {
            float pitch_acc = mpu6050_calculate_pitch(data.accel_x_g, data.accel_y_g, data.accel_z_g);
            float roll_acc  = mpu6050_calculate_roll(data.accel_x_g, data.accel_y_g, data.accel_z_g);

            static absolute_time_t last = 0;
            absolute_time_t now = get_absolute_time();
            float dt = (last == 0) ? 0.0f : absolute_time_diff_us(last, now) / 1e6f;
            last = now;

            pitch_f += data.gyro_y_dps * dt;
            roll_f  += data.gyro_x_dps * dt;

            pitch_f = alpha * pitch_f + (1.0f - alpha) * pitch_acc;
            roll_f  = alpha * roll_f  + (1.0f - alpha) * roll_acc;

            float yaw = mpu6050_calculate_yaw(&mpu);

            printf("%.2f,%.2f,%.2f\n", roll_f, pitch_f, yaw);
        }
        sleep_ms(20);
    }
}
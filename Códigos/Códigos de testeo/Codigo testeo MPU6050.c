/*
 * main.c -- Prueba MPU-6050 en ESP-IDF nativo (XIAO ESP32-C6)
 * Reescritura de test_mpu6050.ino usando drivers nativos ESP-IDF v6.0.2
 * (driver/i2c_master.h -- la API legacy driver/i2c.h esta en EOL desde v6.0)
 *
 * CONEXIONADO (igual que en la version Arduino, mapa de pines v5):
 *   MPU-6050 VCC -> 3V3        MPU-6050 SCL -> GPIO23 (D5)
 *   MPU-6050 GND -> GND        MPU-6050 SDA -> GPIO22 (D4)
 *   MPU-6050 INT -> GPIO2 (D2) MPU-6050 AD0 -> GND (fija addr 0x68)
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MPU6050_TEST";

#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     23
#define MPU_ADDR        0x68
#define MPU_INT_PIN     2

// Registros del MPU-6050 (ver Seccion 5.3 del documento del proyecto)
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75
#define REG_ACCEL_XOUT_H 0x3B
#define REG_CONFIG      0x1A

// Registros de offset de hardware del MPU-6050
#define REG_XA_OFFS_H   0x06
#define REG_YA_OFFS_H   0x08
#define REG_ZA_OFFS_H   0x0A
#define REG_XG_OFFS_H   0x13
#define REG_YG_OFFS_H   0x15
#define REG_ZG_OFFS_H   0x17

// Offsets de calibracion (ESPECIFICOS DE ESTA PLACA Y SU MONTAJE FISICO --
// recalibrar con el sketch de Electronic Cats si cambia el enclosure,
// la orientacion del sensor, o si se reemplaza el chip).
// Calculados por PID tuning en superficie confiable, promedio de 2
// corridas consistentes entre si (diferencia <=1 LSB por eje):
#define OFFSET_ACCEL_X  (-242)
#define OFFSET_ACCEL_Y  (-1493)
#define OFFSET_ACCEL_Z  (1140)
#define OFFSET_GYRO_X   (77)
#define OFFSET_GYRO_Y   (1473)
#define OFFSET_GYRO_Z   (11)

// Se escriben en CADA arranque (no se miden en tiempo de ejecucion) --
// esto asegura que el chip siempre parte de estos valores conocidos,
// sin importar que haya quedado grabado de una sesion de flasheo anterior.
// (Declaradas aca arriba, definidas mas abajo, una vez que mpu_dev existe)
static esp_err_t mpu_write_offset_reg(uint8_t reg_h, int16_t value);
static void mpu_apply_calibration(void);

// Corrección adicional de software para GY: el registro de hardware
// (YG_OFFS) no compensa completamente este eje -- quedó un sesgo
// residual sistemático. Valor medido con diagnose_gy_bias(): promedio
// 22.41 sobre 500 muestras, error estandar 0.314 (sensor plano y
// quieto). Recalcular si cambia el montaje/orientacion del sensor.
#define GY_SOFTWARE_CORRECTION (22)

// Cuantas muestras promediar en el diagnostico puntual (ver mas abajo)
#define GY_DIAGNOSIS_SAMPLES (500)

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_dev;
static QueueHandle_t motion_evt_queue;

// ISR: solo encola el evento, no hace trabajo pesado (buena practica en ISR)
static void IRAM_ATTR mpu_int_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(motion_evt_queue, &gpio_num, NULL);
}

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(mpu_dev, buf, 2, -1);
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(mpu_dev, &reg, 1, data, len, -1);
}

static esp_err_t mpu_write_offset_reg(uint8_t reg_h, int16_t value) {
    uint8_t buf[3] = {reg_h, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_transmit(mpu_dev, buf, 3, -1);
}

static void mpu_apply_calibration(void) {
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_XA_OFFS_H, OFFSET_ACCEL_X));
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_YA_OFFS_H, OFFSET_ACCEL_Y));
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_ZA_OFFS_H, OFFSET_ACCEL_Z));
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_XG_OFFS_H, OFFSET_GYRO_X));
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_YG_OFFS_H, OFFSET_GYRO_Y));
    ESP_ERROR_CHECK(mpu_write_offset_reg(REG_ZG_OFFS_H, OFFSET_GYRO_Z));
    ESP_LOGI(TAG, "Offsets de calibracion aplicados (valores fijos, sin medicion en runtime)");
}

// HERRAMIENTA DE DIAGNOSTICO PUNTUAL -- correr UNA VEZ, anotar el valor
// exacto que imprime, y despues actualizar GY_SOFTWARE_CORRECTION con
// ese numero. No es parte del arranque normal del firmware final.
// Requiere que el sensor este PLANO y QUIETO durante todo el diagnostico
// (~50s a 500 muestras / 100ms por ciclo).
static void diagnose_gy_bias(void) {
    int64_t sum_gy = 0;
    int64_t sum_gy_sq = 0;
    uint8_t raw[14];
    int valid = 0;

    ESP_LOGI(TAG, "=== DIAGNOSTICO GY: dejar el sensor PLANO y QUIETO ===");
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < GY_DIAGNOSIS_SAMPLES; i++) {
        if (mpu_read_regs(REG_ACCEL_XOUT_H, raw, 14) == ESP_OK) {
            int16_t gy_raw = (raw[10] << 8) | raw[11];
            sum_gy += gy_raw;
            sum_gy_sq += (int64_t)gy_raw * gy_raw;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "Diagnostico fallido: 0 muestras validas");
        return;
    }

    double mean = (double)sum_gy / valid;
    double variance = ((double)sum_gy_sq / valid) - (mean * mean);
    double stddev = sqrt(variance);
    double stderr_mean = stddev / sqrt((double)valid);

    ESP_LOGI(TAG, "=== RESULTADO (%d muestras): promedio=%.2f, desv.std=%.2f, error_std=%.3f ===",
              valid, mean, stddev, stderr_mean);
    ESP_LOGI(TAG, "=== Actualizar GY_SOFTWARE_CORRECTION a: %d ===", (int)(mean + 0.5));
}

void app_main(void) {
    // --- Configurar bus I2C (HP I2C, GPIO22/23 segun mapa de pines v5) ---
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu_dev));

    // --- Verificar comunicacion: WHO_AM_I debe devolver 0x68 ---
    uint8_t who_am_i = 0;
    esp_err_t err = mpu_read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU-6050 no detectado (WHO_AM_I=0x%02X, err=%s). "
                       "Verificar SDA/SCL, AD0->GND, alimentacion 3.3V.",
                       who_am_i, esp_err_to_name(err));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MPU-6050 detectado correctamente (WHO_AM_I=0x%02X)", who_am_i);

    // --- Sacar del modo sleep (config activa de prueba, NO es el modo
    //     low-power documentado en Seccion 5.3 -- eso se prueba aparte) ---
    ESP_ERROR_CHECK(mpu_write_reg(REG_PWR_MGMT_1, 0x00));

    // --- Activar DLPF (filtro digital pasabajos) para reducir ruido ---
    // DLPF_CFG=3 -> ancho de banda accel ~44Hz, gyro ~42Hz (vs ~260Hz
    // sin filtrar por defecto). Reduce notablemente el jitter en reposo.
    ESP_ERROR_CHECK(mpu_write_reg(REG_CONFIG, 0x03));

    // --- Aplicar calibracion fija (valores conocidos, sin medir en runtime) ---
    mpu_apply_calibration();

    // --- DIAGNOSTICO PUNTUAL: descomentar solo para recalcular
    //     GY_SOFTWARE_CORRECTION con precision. Sensor plano y quieto.
    //     Una vez que tengas el valor, volver a comentar esta linea.
    //     Ultima corrida: promedio=22.41, error_std=0.314 (500 muestras) ---
    // diagnose_gy_bias();

    // --- Configurar GPIO2 como entrada de interrupcion (INT del MPU) ---
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << MPU_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&int_conf);

    motion_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_INT_PIN, mpu_int_isr_handler, (void *)MPU_INT_PIN);

    ESP_LOGI(TAG, "Listo. Formato: AX,AY,AZ,GX,GY,GZ,TEMP (unidades crudas)");

    uint8_t raw[14];
    uint32_t io_num;

    while (1) {
        if (mpu_read_regs(REG_ACCEL_XOUT_H, raw, 14) == ESP_OK) {
            int16_t ax = (raw[0] << 8) | raw[1];
            int16_t ay = (raw[2] << 8) | raw[3];
            int16_t az = (raw[4] << 8) | raw[5];
            int16_t temp_raw = (raw[6] << 8) | raw[7];
            int16_t gx = (raw[8] << 8) | raw[9];
            int16_t gy = ((raw[10] << 8) | raw[11]) - GY_SOFTWARE_CORRECTION;
            int16_t gz = (raw[12] << 8) | raw[13];

            float temp_c = (temp_raw / 340.0f) + 36.53f;

            printf("%d,%d,%d,%d,%d,%d,%.2f\n", ax, ay, az, gx, gy, gz, temp_c);
        } else {
            ESP_LOGW(TAG, "Error de lectura I2C");
        }

        // Chequear si hubo evento de interrupcion (no bloqueante)
        if (xQueueReceive(motion_evt_queue, &io_num, 0)) {
            ESP_LOGI(TAG, ">> INT de movimiento detectado en GPIO%d <<", (int)io_num);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz, prueba de banco
    }
}

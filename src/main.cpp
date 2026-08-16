// main.cpp - Seguidor de linha ESP32-S3 (ESP-IDF 6.0.2)
// - 12 sensores frontais + 4 laterais QRE1113 via 74HC4067
// - Manta preta, linha/marcações brancas
// - Controle PD (direção) + PI (velocidade média)
// - Motores com VNH5050 (PWM via LEDC) + sucção
// - Encoders via GPIO ISR (agora com sinal conforme sentido de rotação)
// - IMU LSM6DSRTR via I2C
// - Modos de teste de periféricos selecionáveis em CURRENT_TEST
// - Serial BLE simples: "M0" = parar, "M1" = andar

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"

// Parâmetros do robô (flash/NVS) e tabela de encoder (flash/NVS, em chunks BLE)
#include "nvs_flash.h"
#include "ble_params.h"
#include "ble_table.h"

#define TAG "LINE_FOLLOWER"

// ==================== SELEÇÃO DE MODO DE TESTE ====================

typedef enum {
    TEST_MODE_LINE_FOLLOW = 0,   // seguidor completo
    TEST_MODE_QRE,               // testar sensores QRE
    TEST_MODE_IMU,               // testar IMU
    TEST_MODE_MOTORS,            // testar motores
    TEST_MODE_ENCODERS,          // testar encoders
    TEST_MODE_LED                // testar LED simples
} test_mode_t;

// Escolha aqui o que quer rodar:
#define CURRENT_TEST TEST_MODE_LINE_FOLLOW

// ==================== CONFIGURAÇÃO DE SENSORES ====================

#define NUM_FRONT_SENSORS        12      // 12 sensores frontais
#define NUM_SIDE_SENSORS         4       // 2 esquerda, 2 direita

// --- MUX 74HC4067 ---
// OUT_MUX -> IO10 (ESP32-S3: ADC1_CHANNEL_9 ≈ ADC_CHANNEL_9)
#define MUX_ADC_CH         ADC_CHANNEL_9
#define FRONT_MUX_ADC_CH   MUX_ADC_CH
#define SIDE_MUX_ADC_CH    MUX_ADC_CH

// GPIOs de seleção do multiplexador (MUX0..3)
// MUX0  IO39, MUX1 IO40, MUX2 IO41, MUX3 IO42
#define MUX_SEL0_GPIO      GPIO_NUM_39
#define MUX_SEL1_GPIO      GPIO_NUM_40
#define MUX_SEL2_GPIO      GPIO_NUM_41
#define MUX_SEL3_GPIO      GPIO_NUM_42

#define MUX_NUM_BITS       4
#define MUX_NUM_CHANNELS   (1 << MUX_NUM_BITS)

// Mapeamento: índice do sensor -> canal do mux
static const uint8_t front_mux_channel[NUM_FRONT_SENSORS] = {
    13,12,7,6,5,4,0,1,2,3,8,9
};

static const uint8_t side_mux_channel[NUM_SIDE_SENSORS] = {
     15, 14, 10, 11
};

// ==================== THRESHOLDS E JANELAS ====================

// Linha/marcação BRANCA em manta PRETA:
//   - valores ADC BAIXOS = preto (fundo)
//   - valores ADC ALTOS  = branco (linha/marca)
#define SIDE_MARK_THRESHOLD      2500    // TODO: calibrar
#define FRONT_MARK_THRESHOLD     2500    // TODO: calibrar

// Tudo PRETO (sem linha) = leituras abaixo deste limiar em TODOS os sensores
#define BLACK_LEVEL_THRESHOLD    1500    // TODO: calibrar

// Thresholds em tempo de execução (iniciam com os defaults)
static uint16_t g_side_mark_threshold  = SIDE_MARK_THRESHOLD;
static uint16_t g_front_mark_threshold = FRONT_MARK_THRESHOLD;
static uint16_t g_black_level_threshold = BLACK_LEVEL_THRESHOLD;

// Mínimo de sensores frontais “brancos” para considerar cruzamento
#define MIN_FRONT_ON_FOR_CROSS   6       // TODO: ajustar

// Janela de média móvel (em amostras da task de linha)
#define MOVING_AVG_WINDOW        5

// Número de leituras consecutivas com TODOS os sensores em preto para parar
#define ALL_BLACK_REQUIRED       9

// ==================== CONTROLE ====================

#define CONTROL_LOOP_PERIOD_MS   10.0f

#define KP_DIR_DEFAULT           0.3f
#define KD_DIR_DEFAULT           0.6f
#define KP_VEL_DEFAULT           0.5f
#define KI_VEL_DEFAULT           0.1f
#define TARGET_SPEED_DEFAULT     100.0f    // contagens de encoder por período lógico

// Variáveis globais configuráveis (fixas aqui, sem BLE)
volatile float g_kp_dir              = KP_DIR_DEFAULT;
volatile float g_kd_dir              = KD_DIR_DEFAULT;
volatile float g_kp_vel              = KP_VEL_DEFAULT;
volatile float g_ki_vel              = KI_VEL_DEFAULT;
volatile float g_target_speed_counts = TARGET_SPEED_DEFAULT;

// Flag de habilitação do robô via BLE: 0 = parado, 1 = andando
static volatile bool g_run_enabled = false;

// Velocidade normalizada de motor (-1.0 .. +1.0)
#define MOTOR_CMD_MIN            (-1.0f)
#define MOTOR_CMD_MAX            ( 1.0f)

// ==================== PINOUT ====================

// Motores de tração
// PWM_ESQ IO3, PWM_DIR IO38
#define GPIO_MOTOR_LEFT_PWM      GPIO_NUM_3
#define GPIO_MOTOR_RIGHT_PWM     GPIO_NUM_38

// Motor de sucção (MOT_SUCCAO IO11)
#define GPIO_SUCTION_PWM         GPIO_NUM_11

// Driver VNH5050 – motor direito
// CS_MR IO1, DIR_MR2 IO2, DIR_MR1 IO37
#define CS_MR_GPIO               GPIO_NUM_1
#define DIR_MR2_GPIO             GPIO_NUM_2
#define DIR_MR1_GPIO             GPIO_NUM_37

// Driver VNH5050 – motor esquerdo
// CS_ML IO8, DIR_ML2 IO9, DIR_ML1 IO15
#define CS_ML_GPIO               GPIO_NUM_8
#define DIR_ML2_GPIO             GPIO_NUM_9
#define DIR_ML1_GPIO             GPIO_NUM_15

// Encoders (vamos usar L1 e R1)
// ENC_L1 IO6, ENC_L2 IO7, ENC_R1 IO12, ENC_R2 IO13
#define GPIO_ENC_L1              GPIO_NUM_6
#define GPIO_ENC_R1              GPIO_NUM_12

// LED de teste (escolha um pino livre)
#define GPIO_TEST_LED            GPIO_NUM_47

// --- IMU LSM6DSRTR (I2C) ---
// INT1 IO48, SCL_IMU IO36, SDA_IMU IO35
#define IMU_I2C_NUM      I2C_NUM_0
#define IMU_I2C_SDA_GPIO GPIO_NUM_35
#define IMU_I2C_SCL_GPIO GPIO_NUM_36
#define IMU_INT1_GPIO    GPIO_NUM_48
#define IMU_I2C_FREQ_HZ  400000

// Endereço I2C da LSM6DSR (SA0/SDO baixo → 0x6A, alto → 0x6B)
#define LSM6DSR_ADDR     0x6B

// Registradores principais
#define LSM6DSR_REG_WHO_AM_I 0x0F
#define LSM6DSR_REG_CTRL1_XL 0x10
#define LSM6DSR_REG_CTRL2_G  0x11
#define LSM6DSR_REG_CTRL3_C  0x12
#define LSM6DSR_REG_OUTX_L_G 0x22
#define LSM6DSR_REG_OUTX_L_XL 0x28

// ==================== LEDC PARA MOTORES/SUCÇÃO ====================

#define MOTOR_PWM_FREQ_HZ      20000              // 20 kHz
#define MOTOR_PWM_RESOLUTION   LEDC_TIMER_10_BIT  // 0..1023

#define LEDC_MODE_MOTOR        LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_MOTOR       LEDC_TIMER_0
#define LEDC_CHANNEL_LEFT      LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT     LEDC_CHANNEL_1
#define LEDC_CHANNEL_SUCTION   LEDC_CHANNEL_2

// ==================== TIPOS / VARIÁVEIS GLOBAIS ====================

typedef struct {
    bool left_mark;
    bool right_mark;
    bool is_crossing;
} lane_marks_t;

// Handle global do ADC (modo oneshot)
static adc_oneshot_unit_handle_t s_adc_handle;

// Encoders (contagem via GPIO ISR)
static volatile int32_t s_encoder_left_counts  = 0;
static volatile int32_t s_encoder_right_counts = 0;
// NOVO: sinal atual de cada roda (+1 frente, -1 ré, 0 parado)
static volatile int8_t s_encoder_left_sign  = 0;
static volatile int8_t s_encoder_right_sign = 0;

// Calibração global para todos os QRE (frontais + laterais)
static uint16_t g_qre_min_all = UINT16_MAX;
static uint16_t g_qre_max_all = 0;

// ==================== PROTÓTIPOS ====================
extern "C" void nimble_uart_init(void);  // vindo de ble_nimble_uart.cpp

static void mux_init(void);
static void mux_set_channel(uint8_t ch);

static void qre_init(void);
static void qre_read_front(uint16_t *front_values);
static void qre_read_side(uint16_t *side_values);

static lane_marks_t detect_lane_marks(const uint16_t *front_values,
                                      const uint16_t *side_values);

static float compute_line_position(const uint16_t *front_values);

static void motors_init(void);
static void motor_set_speed_norm(float left, float right);
static void suction_init(void);
static void suction_set(float level);

static void encoders_init(void);
static int16_t encoder_get_counts_left(void);
static int16_t encoder_get_counts_right(void);
static void encoder_clear_counts(void);

static esp_err_t imu_i2c_init(void);
static esp_err_t lsm6dsr_write_reg(uint8_t reg, uint8_t val);
static esp_err_t lsm6dsr_read_regs(uint8_t reg, uint8_t *data, size_t len);
static esp_err_t imu_init(void);
static esp_err_t imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                              int16_t *gx, int16_t *gy, int16_t *gz);

static void line_follow_task(void *arg);
static void qre_test_task(void *arg);
static void imu_test_task(void *arg);
static void motors_test_task(void *arg);
static void encoders_test_task(void *arg);
static void led_test_task(void *arg);

// ==================== MUX 74HC4067 ====================

static void mux_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << MUX_SEL0_GPIO) |
                           (1ULL << MUX_SEL1_GPIO) |
                           (1ULL << MUX_SEL2_GPIO) |
                           (1ULL << MUX_SEL3_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    gpio_set_level(MUX_SEL0_GPIO, 0);
    gpio_set_level(MUX_SEL1_GPIO, 0);
    gpio_set_level(MUX_SEL2_GPIO, 0);
    gpio_set_level(MUX_SEL3_GPIO, 0);
}

static void mux_set_channel(uint8_t ch)
{
    gpio_set_level(MUX_SEL0_GPIO, (ch >> 0) & 0x01);
    gpio_set_level(MUX_SEL1_GPIO, (ch >> 1) & 0x01);
    gpio_set_level(MUX_SEL2_GPIO, (ch >> 2) & 0x01);
    gpio_set_level(MUX_SEL3_GPIO, (ch >> 3) & 0x01);
    esp_rom_delay_us(10);
}

// ==================== SENSORES QRE (ADC ONESHOT) ====================
static void qre_calib_global_init(void)
{
    g_qre_min_all = UINT16_MAX;
    g_qre_max_all = 0;
}

static void qre_calib_global_update(const uint16_t *front, const uint16_t *side)
{
    for (int i = 0; i < NUM_FRONT_SENSORS; i++) {
        uint16_t v = front[i];
        if (v < g_qre_min_all) g_qre_min_all = v;
        if (v > g_qre_max_all) g_qre_max_all = v;
    }

    for (int i = 0; i < NUM_SIDE_SENSORS; i++) {
        uint16_t v = side[i];
        if (v < g_qre_min_all) g_qre_min_all = v;
        if (v > g_qre_max_all) g_qre_max_all = v;
    }
}

static void qre_calib_global_finish_and_apply(void)
{
    uint16_t minv = g_qre_min_all;  // ≈ branco
    uint16_t maxv = g_qre_max_all;  // ≈ preto
    if (minv == UINT16_MAX || maxv <= minv) {
        ESP_LOGW(TAG, "Calibracao QRE invalida (min/max nao atualizados), mantendo thresholds default.");
        return;
    }

    // faixa = [minv (branco) ... maxv (preto)]
    uint16_t range = (uint16_t)(maxv - minv);

    // threshold para "linha branca": um pouco acima do branco (baixo)
    uint16_t thr_line = (uint16_t)(minv + range / 4);

    // threshold para "tudo preto": um pouco abaixo do preto (alto)
    uint16_t thr_black = (uint16_t)(maxv - range / 4);

    g_black_level_threshold  = thr_black;
    g_front_mark_threshold   = thr_line;
    g_side_mark_threshold    = thr_line;

    ESP_LOGI(TAG, "=== CALIB QRE (invertido) ===");
    ESP_LOGI(TAG, "MIN (branco aprox) = %u", minv);
    ESP_LOGI(TAG, "MAX (preto aprox)  = %u", maxv);
    ESP_LOGI(TAG, "BLACK_LEVEL_THRESHOLD  = %u", g_black_level_threshold);
    ESP_LOGI(TAG, "FRONT_MARK_THRESHOLD   = %u", g_front_mark_threshold);
    ESP_LOGI(TAG, "SIDE_MARK_THRESHOLD    = %u", g_side_mark_threshold);
}

static void qre_calib_global_print(void)
{
    uint16_t minv = g_qre_min_all;
    uint16_t maxv = g_qre_max_all;
    uint16_t thr  = (uint16_t)((minv + maxv) / 2);

    ESP_LOGI(TAG, "=== CALIBRACAO GLOBAL QRE ===");
    ESP_LOGI(TAG, "MIN (preto aprox)  = %u", minv);
    ESP_LOGI(TAG, "MAX (branco aprox) = %u", maxv);
    ESP_LOGI(TAG, "THRESHOLD (meio)   = %u", thr);
    ESP_LOGI(TAG, "Sugestao BLACK_LEVEL_THRESHOLD  ~= %u", minv + (maxv - minv) / 4);
    ESP_LOGI(TAG, "Sugestao FRONT/SIDE_MARK_THRESHOLD ~= %u", thr);
}

static void qre_init(void)
{
    mux_init();
    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = ADC_UNIT_1;
    init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit falhou: %s", esp_err_to_name(err));
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan_cfg.atten = ADC_ATTEN_DB_12;

    err = adc_oneshot_config_channel(s_adc_handle, MUX_ADC_CH, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel falhou: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "QRE + MUX inicializados (ADC oneshot).");
}

static void qre_read_front(uint16_t *front_values)
{
    for (int i = 0; i < NUM_FRONT_SENSORS; i++) {
        uint8_t ch = front_mux_channel[i];
        mux_set_channel(ch);

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, FRONT_MUX_ADC_CH, &raw);
        if (err != ESP_OK || raw < 0) {
            raw = 0;
        }
        front_values[i] = (uint16_t)raw;
    }
}

static void qre_read_side(uint16_t *side_values)
{
    for (int i = 0; i < NUM_SIDE_SENSORS; i++) {
        uint8_t ch = side_mux_channel[i];
        mux_set_channel(ch);

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, SIDE_MUX_ADC_CH, &raw);
        if (err != ESP_OK || raw < 0) {
            raw = 0;
        }
        side_values[i] = (uint16_t)raw;
    }
}

// ==================== DETECÇÃO DE MARCAÇÕES ====================
static lane_marks_t detect_lane_marks(const uint16_t *front_values,
                                      const uint16_t *side_values)
{
    static uint16_t side_hist[NUM_SIDE_SENSORS][MOVING_AVG_WINDOW];
    static uint32_t side_sum[NUM_SIDE_SENSORS];
    static uint16_t front_hist[NUM_FRONT_SENSORS][MOVING_AVG_WINDOW];
    static uint32_t front_sum[NUM_FRONT_SENSORS];
    static int hist_index = 0;

    float side_avg[NUM_SIDE_SENSORS] = {0};
    float front_avg[NUM_FRONT_SENSORS] = {0};

    lane_marks_t marks{};

    for (int i = 0; i < NUM_SIDE_SENSORS; i++) {
        side_sum[i] -= side_hist[i][hist_index];
        side_hist[i][hist_index] = side_values[i];
        side_sum[i] += side_values[i];
        side_avg[i] = side_sum[i] / (float)MOVING_AVG_WINDOW;
    }

    for (int j = 0; j < NUM_FRONT_SENSORS; j++) {
        front_sum[j] -= front_hist[j][hist_index];
        front_hist[j][hist_index] = front_values[j];
        front_sum[j] += front_values[j];
        front_avg[j] = front_sum[j] / (float)MOVING_AVG_WINDOW;
    }

    hist_index = (hist_index + 1) % MOVING_AVG_WINDOW;

    float avg_side_0 = (NUM_SIDE_SENSORS > 0) ? side_avg[0] : 0.0f;
    float avg_side_1 = (NUM_SIDE_SENSORS > 1) ? side_avg[1] : 0.0f;
    float avg_side_2 = (NUM_SIDE_SENSORS > 2) ? side_avg[2] : 0.0f;
    float avg_side_3 = (NUM_SIDE_SENSORS > 3) ? side_avg[3] : 0.0f;

    marks.left_mark  = (avg_side_0 < g_side_mark_threshold) ||
                       (avg_side_1 < g_side_mark_threshold);

    marks.right_mark = (avg_side_2 < g_side_mark_threshold) ||
                       (avg_side_3 < g_side_mark_threshold);

    int active_front = 0;
    for (int j = 0; j < NUM_FRONT_SENSORS; j++) {
        if (front_avg[j] < g_front_mark_threshold) {
            active_front++;
        }
    }

    marks.is_crossing = (marks.left_mark &&
                         marks.right_mark &&
                         active_front >= MIN_FRONT_ON_FOR_CROSS);

    return marks;
}

// ==================== POSIÇÃO DA LINHA ====================
static float compute_line_position(const uint16_t *front_values)
{
    float num = 0.0f;
    float den = 0.0f;

    // usa g_qre_min_all / g_qre_max_all para inverter
    uint16_t minv = g_qre_min_all;
    uint16_t maxv = g_qre_max_all;
    if (minv == UINT16_MAX || maxv <= minv) {
        // se calib falhou, usa valor cru mesmo
        minv = 0;
        maxv = 4095;
    }

    for (int i = 0; i < NUM_FRONT_SENSORS; i++) {
        float weight = (float)i - (NUM_FRONT_SENSORS - 1) / 2.0f;

        // valor invertido: branco (baixo) vira alto, preto (alto) vira baixo
        float raw   = (float)front_values[i];
        float value = (float)maxv + (float)minv - raw;
        if (value < 0.0f) value = 0.0f;
        num += weight * value;
        den += value;
    }

    if (den < 1e-3f) {
        return 0.0f; // linha perdida
    }

    float position   = num / den;
    float max_weight = (NUM_FRONT_SENSORS - 1) / 2.0f;
    return position / max_weight;  // ~[-1, 1]
}

// ==================== MOTORES (LEDC + VNH5050) ====================
static void set_left_dir(int sign)
{
    if (sign > 0) {
        gpio_set_level(DIR_ML1_GPIO, 1);
        gpio_set_level(DIR_ML2_GPIO, 0);
        s_encoder_left_sign = 1;
    } else if (sign < 0) {
        gpio_set_level(DIR_ML1_GPIO, 0);
        gpio_set_level(DIR_ML2_GPIO, 1);
        s_encoder_left_sign = -1;
    } else {
        gpio_set_level(DIR_ML1_GPIO, 0);
        gpio_set_level(DIR_ML2_GPIO, 0);
        s_encoder_left_sign = 0;
    }
}

static void set_right_dir(int sign)
{
    if (sign > 0) {
        gpio_set_level(DIR_MR1_GPIO, 0);
        gpio_set_level(DIR_MR2_GPIO, 1);
        s_encoder_right_sign = 1;
    } else if (sign < 0) {
        gpio_set_level(DIR_MR1_GPIO, 1);
        gpio_set_level(DIR_MR2_GPIO, 0);
        s_encoder_right_sign = -1;
    } else {
        gpio_set_level(DIR_MR1_GPIO, 0);
        gpio_set_level(DIR_MR2_GPIO, 0);
        s_encoder_right_sign = 0;
    }
}

static void motors_init(void)
{
    // DIR pinos do VNH5050
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << DIR_ML1_GPIO) |
                           (1ULL << DIR_ML2_GPIO) |
                           (1ULL << DIR_MR1_GPIO) |
                           (1ULL << DIR_MR2_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    // CS (corrente) como entrada (opcional)
    gpio_set_direction(CS_ML_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(CS_MR_GPIO, GPIO_MODE_INPUT);

    // Timer LEDC
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_MODE_MOTOR;
    timer_cfg.duty_resolution = MOTOR_PWM_RESOLUTION;
    timer_cfg.timer_num = LEDC_TIMER_MOTOR;
    timer_cfg.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer_cfg);

    // Canal esquerdo
    ledc_channel_config_t ch_left = {};
    ch_left.gpio_num = GPIO_MOTOR_LEFT_PWM;
    ch_left.speed_mode = LEDC_MODE_MOTOR;
    ch_left.channel = LEDC_CHANNEL_LEFT;
    ch_left.intr_type = LEDC_INTR_DISABLE;
    ch_left.timer_sel = LEDC_TIMER_MOTOR;
    ch_left.duty = 0;
    ch_left.hpoint = 0;

    ledc_channel_config(&ch_left);

    // Canal direito
    ledc_channel_config_t ch_right = {};
    ch_right.gpio_num = GPIO_MOTOR_RIGHT_PWM;
    ch_right.speed_mode = LEDC_MODE_MOTOR;
    ch_right.channel = LEDC_CHANNEL_RIGHT;
    ch_right.intr_type = LEDC_INTR_DISABLE;
    ch_right.timer_sel = LEDC_TIMER_MOTOR;
    ch_right.duty = 0;
    ch_right.hpoint = 0;

    ledc_channel_config(&ch_right);

    ESP_LOGI(TAG, "Motores inicializados (LEDC em IO3 e IO38).");
}

// left/right ∈ [-1.0, 1.0]
static void motor_set_speed_norm(float left, float right)
{
    if (left > MOTOR_CMD_MAX)  left = MOTOR_CMD_MAX;
    if (left < MOTOR_CMD_MIN)  left = MOTOR_CMD_MIN;
    if (right > MOTOR_CMD_MAX) right = MOTOR_CMD_MAX;
    if (right < MOTOR_CMD_MIN) right = MOTOR_CMD_MIN;

    int sign_left  = (left  > 0.0f) - (left  < 0.0f);
    int sign_right = (right > 0.0f) - (right < 0.0f);

    float abs_left  = fabsf(left);
    float abs_right = fabsf(right);

    uint32_t max_duty = (1u << MOTOR_PWM_RESOLUTION) - 1u;
    uint32_t duty_left  = (uint32_t)(abs_left  * max_duty + 0.5f);
    uint32_t duty_right = (uint32_t)(abs_right * max_duty + 0.5f);

    set_left_dir(sign_left);
    set_right_dir(sign_right);

    ledc_set_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_LEFT, duty_left);
    ledc_update_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_LEFT);

    ledc_set_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_RIGHT, duty_right);
    ledc_update_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_RIGHT);
}

// ==================== SUCÇÃO ====================

static void suction_init(void)
{
    // Usa mesmo timer, canal separado
    ledc_channel_config_t ch_suction = {};
    ch_suction.gpio_num = GPIO_SUCTION_PWM;
    ch_suction.speed_mode = LEDC_MODE_MOTOR;
    ch_suction.channel = LEDC_CHANNEL_SUCTION;
    ch_suction.intr_type = LEDC_INTR_DISABLE;
    ch_suction.timer_sel = LEDC_TIMER_MOTOR;
    ch_suction.duty = 0;
    ch_suction.hpoint = 0;

    ledc_channel_config(&ch_suction);

    suction_set(0.0f);
    ESP_LOGI(TAG, "Sucção inicializada (LEDC em IO11).");
}

// level em 0.0 .. 1.0
static void suction_set(float level)
{
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;

    uint32_t max_duty = (1u << MOTOR_PWM_RESOLUTION) - 1u;
    uint32_t duty = (uint32_t)(level * max_duty + 0.5f);

    ledc_set_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_SUCTION, duty);
    ledc_update_duty(LEDC_MODE_MOTOR, LEDC_CHANNEL_SUCTION);
}

// ==================== ENCODERS (GPIO ISR) ====================

static void IRAM_ATTR encoder_left_isr(void *arg)
{
    (void)arg;
    s_encoder_left_counts += s_encoder_left_sign;
}

static void IRAM_ATTR encoder_right_isr(void *arg)
{
    (void)arg;
    s_encoder_right_counts += s_encoder_right_sign;
}

static void encoders_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << GPIO_ENC_L1) | (1ULL << GPIO_ENC_R1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;

    gpio_config(&io_conf);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service falhou: %s", esp_err_to_name(err));
        return;
    }

    gpio_isr_handler_add(GPIO_ENC_L1, encoder_left_isr,  NULL);
    gpio_isr_handler_add(GPIO_ENC_R1, encoder_right_isr, NULL);

    s_encoder_left_counts  = 0;
    s_encoder_right_counts = 0;
    s_encoder_left_sign    = 0;
    s_encoder_right_sign   = 0;

    ESP_LOGI(TAG, "Encoders inicializados (GPIO ISR em L1/R1).");
}

static int16_t encoder_get_counts_left(void)
{
    int32_t val = s_encoder_left_counts;
    if (val > INT16_MAX)  val = INT16_MAX;
    if (val < INT16_MIN)  val = INT16_MIN;
    return (int16_t)val;
}

static int16_t encoder_get_counts_right(void)
{
    int32_t val = s_encoder_right_counts;
    if (val > INT16_MAX)  val = INT16_MAX;
    if (val < INT16_MIN)  val = INT16_MIN;
    return (int16_t)val;
}

static void encoder_clear_counts(void)
{
    s_encoder_left_counts  = 0;
    s_encoder_right_counts = 0;
}

// ==================== IMU LSM6DSRTR (I2C) ====================

static esp_err_t imu_i2c_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = IMU_I2C_SDA_GPIO;
    conf.scl_io_num = IMU_I2C_SCL_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = IMU_I2C_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(IMU_I2C_NUM, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(IMU_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    gpio_set_direction(IMU_INT1_GPIO, GPIO_MODE_INPUT);
    return ESP_OK;
}

static esp_err_t lsm6dsr_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t lsm6dsr_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t imu_init(void)
{
    esp_err_t err = imu_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "imu_i2c_init falhou: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t who = 0;
    err = lsm6dsr_read_regs(LSM6DSR_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler WHO_AM_I da IMU");
        return err;
    }
    ESP_LOGI(TAG, "LSM6DSR WHO_AM_I = 0x%02X", who);

    // Reset suave
    lsm6dsr_write_reg(LSM6DSR_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // CTRL3_C: IF_INC=1 (auto-incremento), BDU=1
    lsm6dsr_write_reg(LSM6DSR_REG_CTRL3_C, 0x44);

    // CTRL1_XL: 104 Hz, ±2g
    lsm6dsr_write_reg(LSM6DSR_REG_CTRL1_XL, 0x60);

    // CTRL2_G: 104 Hz, ±2000 dps
    lsm6dsr_write_reg(LSM6DSR_REG_CTRL2_G, 0x6C);

    ESP_LOGI(TAG, "IMU LSM6DSR inicializada.");
    return ESP_OK;
}

static esp_err_t imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                              int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    esp_err_t ret;

    // Gyro
    ret = lsm6dsr_read_regs(LSM6DSR_REG_OUTX_L_G, buf, 6);
    if (ret != ESP_OK) return ret;
    int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);

    // Accel
    ret = lsm6dsr_read_regs(LSM6DSR_REG_OUTX_L_XL, buf, 6);
    if (ret != ESP_OK) return ret;
    int16_t ax_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az_raw = (int16_t)((buf[5] << 8) | buf[4]);

    if (ax) *ax = ax_raw;
    if (ay) *ay = ay_raw;
    if (az) *az = az_raw;
    if (gx) *gx = gx_raw;
    if (gy) *gy = gy_raw;
    if (gz) *gz = gz_raw;

    return ESP_OK;
}

// ============================================================
// =============== SERIAL BLE SIMPLES "M0"/"M1" ================
// ============================================================
//
// - Externo a implementar com BLE do ESP32-S3 (NimBLE/Bluedroid):
//   extern void bt_serial_send(const uint8_t *data, size_t len);
// - No callback de escrita da característica BLE, chame:
//   bt_serial_rx_push(data, len);
// - Cada escrita BLE já é tratada como um comando completo (o app manda um
//   write() por comando) — não há mais fila/parse byte-a-byte por '\n'.
// - Comandos aceitos:
//   "M0" -> g_run_enabled = false  (para o robô)
//   "M1" -> g_run_enabled = true   (robô anda)
//   qualquer outro -> resposta "ERR\n"
//

extern "C" void bt_serial_send(const uint8_t *data, size_t len);

#define BT_SERIAL_RX_QUEUE_LENGTH    64
#define BT_SERIAL_LINE_BUF_LEN       32

static const char *BT_TAG = "BT_SERIAL";

// Trim de espaços/tab/CR/LF nas pontas (in-place)
static char *bt_trim_spaces(char *s)
{
    char *end;

    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }

    if (*s == 0) {
        return s;
    }

    end = s + strlen(s) - 1;
    while (end > s && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
        end--;
    }

    *(end + 1) = '\0';
    return s;
}

// printf via bt_serial_send
static void bt_serial_printf(const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) {
        return;
    }
    if ((size_t)len > sizeof(buf)) {
        len = sizeof(buf);
    }

    bt_serial_send((const uint8_t *)buf, (size_t)len);
}

// Parser de uma linha: aceita só "M0" ou "M1"
static void bt_serial_process_line(char *line)
{
    ESP_LOGI(TAG, "BT RX line: '%s'", line);

    char *cmd = bt_trim_spaces(line);
    if (*cmd == '\0') {
        // linha vazia -> ignora
        return;
    }

    // Comandos de parâmetros do robô (GET_PARAMS / SET_PARAMS:...), salvos
    // na flash (NVS) — ver ble_params.cpp. Se for um desses, já responde e
    // não cai no parser de "M0"/"M1" abaixo.
    if (ble_params_handle_command(cmd)) {
        return;
    }

    // Comandos da tabela de encoder (GET_TABLE / SET_TABLE_BEGIN / _CHUNK /
    // _END), também na flash — ver ble_table.cpp.
    if (ble_table_handle_command(cmd)) {
        return;
    }

    // Comando deve ser exatamente "M0" ou "M1"
    if (strcmp(cmd, "M0") == 0 || strcmp(cmd, "M1") == 0) {
        bool run = (strcmp(cmd, "M1") == 0);
        g_run_enabled = run;
        ESP_LOGI(BT_TAG, "g_run_enabled = %d", run ? 1 : 0);
        bt_serial_send((const uint8_t *)"OK\n", 3);
    } else {
        ESP_LOGW(BT_TAG, "Comando invalido: '%s'", cmd);
        bt_serial_send((const uint8_t *)"ERR\n", 4);
    }
}

// Enfileiramento de bytes (chamar do callback BLE) — agora processa direto,
// cada escrita BLE já chega como um comando completo.
extern "C" void bt_serial_rx_push(const uint8_t *data, uint16_t len)
{
    ESP_LOGI(TAG, "BT RX push: %.*s", len, data);

    std::string datastr = std::string((const char*)data, len);
    bt_serial_process_line((char*)datastr.c_str());
}

// Task principal (atualmente ociosa — processamento é direto em
// bt_serial_rx_push, chamado a partir do callback BLE)
static void bt_serial_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "bt_serial_task iniciada");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Inicialização da serial BLE (chamar em app_main, após BLE)
void bt_serial_init(void)
{
    BaseType_t res = xTaskCreate(
        bt_serial_task,
        "bt_serial_task",
        4096,
        NULL,
        5,
        NULL
    );
    if (res != pdPASS) {
        ESP_LOGW(BT_TAG, "Falha ao criar bt_serial_task");
    }
}

// ==================== TASK: SEGUIDOR DE LINHA ====================

static void line_follow_task(void *arg)
{
    uint16_t front_values[NUM_FRONT_SENSORS];
    uint16_t side_values[NUM_SIDE_SENSORS];

    float prev_error_dir = 0.0f;
    float integral_vel   = 0.0f;

    static bool prev_left_mark  = false;
    static bool prev_right_mark = false;
    static int  all_black_counter = 0;

    // acumuladores de pulsos desde o início
    int32_t enc_l_total = 0;
    int32_t enc_r_total = 0;
    int     right_mark_count = 0;

    TickType_t last_wake = xTaskGetTickCount();

    qre_calib_global_init();
    ESP_LOGI(TAG, "Calibracao QRE iniciada (5s). Mova o robo sobre PRETO e BRANCO.");

    TickType_t start = xTaskGetTickCount();
    const TickType_t duration = pdMS_TO_TICKS(5000); // 5 s
    TickType_t last_cal = start;

    while ((xTaskGetTickCount() - start) < duration) {
        vTaskDelayUntil(&last_cal, pdMS_TO_TICKS(50)); // 20 Hz
        qre_read_front(front_values);
        qre_read_side(side_values);
        qre_calib_global_update(front_values, side_values);
    }

    qre_calib_global_finish_and_apply();
    ESP_LOGI(TAG, "Calibracao QRE concluida, aguardando comando BLE (M0/M1).");

    // Loop principal
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)CONTROL_LOOP_PERIOD_MS));

        // Se não habilitado via BLE: garante robô parado e continua
        if (!g_run_enabled) {
            motor_set_speed_norm(0.0f, 0.0f);
            suction_set(0.0f);
            // zera integrais para evitar "tranco" ao religar
            prev_error_dir = 0.0f;
            integral_vel   = 0.0f;
            continue;
        }

        // Habilitado: liga sucção
        suction_set(0.75f);

        qre_read_front(front_values);
        qre_read_side(side_values);

        // Todos em preto?
        bool all_black = true;
        for (int i = 0; i < NUM_FRONT_SENSORS; i++) {
            if (front_values[i] < g_black_level_threshold) {
                all_black = false;
                break;
            }
        }
        if (all_black) {
            for (int i = 0; i < NUM_SIDE_SENSORS; i++) {
                if (side_values[i] < g_black_level_threshold) {
                    all_black = false;
                    break;
                }
            }
        }

        if (all_black) {
            all_black_counter++;
        } else {
            all_black_counter = 0;
        }

        if (all_black_counter >= ALL_BLACK_REQUIRED) {
            motor_set_speed_norm(0.0f, 0.0f);
            suction_set(0.0f);
            ESP_LOGW(TAG, "TODOS os sensores em preto por %d leituras. Parando o robô.",
                     all_black_counter);
            continue;
        }

        lane_marks_t marks = detect_lane_marks(front_values, side_values);

        bool left_rising  = (!prev_left_mark  && marks.left_mark);
        bool right_rising = (!prev_right_mark && marks.right_mark);

        prev_left_mark  = marks.left_mark;
        prev_right_mark = marks.right_mark;

        if (!marks.is_crossing) {
            if (left_rising) {
                // printa pulsos acumulados quando marca lateral esquerda sobe
                ESP_LOGI(TAG,
                        "LEFT MARK: ENC_L=%" PRId32 " ENC_R=%" PRId32,
                        enc_l_total, enc_r_total);
            }

            if (right_rising) {
                right_mark_count++;
                ESP_LOGI(TAG,
                        "RIGHT MARK %d: ENC_L=%" PRId32 " ENC_R=%" PRId32,
                        right_mark_count, enc_l_total, enc_r_total);

                if (right_mark_count == 1) {
                    encoder_clear_counts();
                    enc_l_total = 0;
                    enc_r_total = 0;
                }
                if (right_mark_count >= 2) {
                    // para o robô após 2 marcações laterais direitas
                    motor_set_speed_norm(0.0f, 0.0f);
                    suction_set(0.0f);
                    ESP_LOGI(TAG,
                            "Duas marcacoes laterais direitas detectadas. Parando o robo.");
                }
            }
        }

        g_kp_dir = ble_params_get("kp_std");
        g_kd_dir = ble_params_get("kd_std");
        float error_dir = compute_line_position(front_values);
        //ESP_LOGI(TAG, "Erro: %.3f" , (double)error_dir);

        float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
        float deriv_dir = (error_dir - prev_error_dir);
        float u_dir = g_kp_dir * error_dir + g_kd_dir * deriv_dir;
        prev_error_dir = error_dir;

        int16_t counts_left  = encoder_get_counts_left();
        int16_t counts_right = encoder_get_counts_right();
        encoder_clear_counts();

        // acumula pulsos totais (agora com sinal)
        enc_l_total += counts_left;
        enc_r_total += counts_right;

        float speed_meas = (counts_left + counts_right) * 0.5f;

        float error_vel = g_target_speed_counts - speed_meas;
        integral_vel += error_vel * dt;
        float u_vel = g_kp_vel * error_vel + g_ki_vel * integral_vel;

        float base = 0.1f;
        // if (g_target_speed_counts > 1e-3f) {
        //     base = u_vel / g_target_speed_counts;
        // }

        float v_left  = base + u_dir;
        float v_right = base - u_dir;

        //ESP_LOGI(TAG, "V_LEFT=%.3f V_RIGHT=%.3f", (double)v_left, (double)v_right);

        motor_set_speed_norm(v_left, v_right);
    }
}

// ==================== TASK: TESTE QRE ====================

static void qre_test_task(void *arg)
{
    uint16_t front[NUM_FRONT_SENSORS];
    uint16_t side[NUM_SIDE_SENSORS];

    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100)); // ~10 Hz

        qre_read_front(front);
        qre_read_side(side);

        ESP_LOGI(TAG, "QRE FRONT:");
        for (int i = 0; i < NUM_FRONT_SENSORS; i++) {
            printf("%4u ", front[i]);
        }
        printf("\n");
    }
}

// ==================== TASK: TESTE IMU ====================

static void imu_test_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100)); // 10 Hz

        int16_t ax, ay, az, gx, gy, gz;
        if (imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            ESP_LOGI(TAG, "IMU ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d",
                     ax, ay, az, gx, gy, gz);
        } else {
            ESP_LOGW(TAG, "Falha na leitura da IMU");
        }
    }
}

// ==================== TASK: TESTE MOTORES ====================

static void motors_test_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    float cmd = 0.0f;
    float step = 0.05f;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000)); // a cada 1 s

        motor_set_speed_norm(cmd, cmd);  // mesmo valor nos 2 motores
        suction_set(cmd);  // mesmo valor na sucção
        ESP_LOGI(TAG, "MOTORS TEST cmd=%.3f", (double)cmd);

        cmd += step;
        if (cmd > 1.0f) {
            cmd = 1.0f;
        }
    }
}

// ==================== TASK: TESTE ENCODERS ====================

static void encoders_test_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(200)); // 5 Hz

        int16_t l = encoder_get_counts_left();
        int16_t r = encoder_get_counts_right();
        ESP_LOGI(TAG, "ENC L=%6d R=%6d", l, r);
    }
}

// ==================== TASK: TESTE LED ====================

static void led_test_task(void *arg)
{
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << GPIO_TEST_LED);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io);

    bool on = false;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
        on = !on;
        gpio_set_level(GPIO_TEST_LED, on);
        ESP_LOGI(TAG, "LED %s", on ? "ON" : "OFF");
    }
}

// ==================== app_main ====================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando...");

    // NVS precisa ser inicializado antes de usar parâmetros/tabela na flash
    // (ble_params.cpp e ble_table.cpp). Sem isso, nvs_open() falha sempre.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    nimble_uart_init();

    qre_init();
    motors_init();
    suction_init();
    encoders_init();
    imu_init();

    // Inicializa task de comunicação Serial BLE (0/1)
    //bt_serial_init();

    // Task dedicada que envia os pedaços (chunks) de uma leitura de tabela
    // (GET_TABLE) aos poucos, sem travar o resto do sistema — ver ble_table.cpp
    ble_table_init();

    switch (CURRENT_TEST) {
    case TEST_MODE_LINE_FOLLOW:
        ESP_LOGI(TAG, "Modo: SEGUIDOR DE LINHA");
        xTaskCreate(line_follow_task, "line_follow_task", 4096, NULL, 5, NULL);
        break;

    case TEST_MODE_QRE:
        ESP_LOGI(TAG, "Modo: TESTE QRE");
        xTaskCreate(qre_test_task, "qre_test_task", 4096, NULL, 5, NULL);
        break;

    case TEST_MODE_IMU:
        ESP_LOGI(TAG, "Modo: TESTE IMU");
        xTaskCreate(imu_test_task, "imu_test_task", 4096, NULL, 5, NULL);
        break;

    case TEST_MODE_MOTORS:
        ESP_LOGI(TAG, "Modo: TESTE MOTORES");
        xTaskCreate(motors_test_task, "motors_test_task", 4096, NULL, 5, NULL);
        break;

    case TEST_MODE_ENCODERS:
        ESP_LOGI(TAG, "Modo: TESTE ENCODERS");
        xTaskCreate(encoders_test_task, "encoders_test_task", 4096, NULL, 5, NULL);
        break;

    case TEST_MODE_LED:
        ESP_LOGI(TAG, "Modo: TESTE LED");
        xTaskCreate(led_test_task, "led_test_task", 2048, NULL, 5, NULL);
        break;
    }
}
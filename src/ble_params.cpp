// ble_params.cpp
// Parâmetros do robô (Kp/Ki/Kd/velocidade base) armazenados na flash (NVS),
// via API nativa do ESP-IDF (nvs.h) — sem depender do Arduino Preferences.
//
// Protocolo (comandos recebidos via bt_serial_process_line, em main.cpp):
//   "GET_PARAMS"                              -> lê da flash e responde "PARAMS:..."
//   "SET_PARAMS:kp=1.20;ki=0.30;kd=0.05;..."  -> grava na flash e responde "PARAMS_SAVED"
//
// Ajuste a lista de parâmetros só editando PARAM_DEFAULTS abaixo — o resto
// (leitura, gravação, resposta) já lida com qualquer quantidade de chaves.

#include "ble_params.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "esp_log.h"
#include "nvs.h"

// Implementado em ble_nimble_uart.cpp
extern "C" void bt_serial_send(const uint8_t *data, size_t len);

static const char *TAG = "BLE_PARAMS";
static const char *NVS_NAMESPACE = "robo_params";

struct ParamDefault {
    const char *key;
    float default_value;
};

// Ajuste aqui os parâmetros do robô.
static const ParamDefault PARAM_DEFAULTS[] = {
    {"kp", 0.6f},
    {"kd", 0.5f},
    {"kp_std", 0.6f},
    {"kd_std", 0.5f},
    {"kp_vel", 1.20f},
    {"ki_vel", 0.30f},
    {"kd_vel", 0.05f},
    {"vbase_std", 0.1f},
    {"vbase", 150.0f},
    {"Vaccum_std", 0.5f},
    {"Vaccum", 0.5f}
};
static const size_t PARAM_COUNT = sizeof(PARAM_DEFAULTS) / sizeof(PARAM_DEFAULTS[0]);

static float default_for(const char *key)
{
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(PARAM_DEFAULTS[i].key, key) == 0) {
            return PARAM_DEFAULTS[i].default_value;
        }
    }
    return 0.0f;
}

float ble_params_get(const char *key)
{
    float default_value = default_for(key);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return default_value; // namespace ainda não existe (nunca salvou nada)
    }

    float value = default_value;
    size_t size = sizeof(value);
    err = nvs_get_blob(handle, key, &value, &size);
    nvs_close(handle);

    return (err == ESP_OK) ? value : default_value;
}

void ble_params_set(const char *key, float value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open (RW) falhou: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, key, &value, sizeof(value));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao salvar parametro '%s': %s", key, esp_err_to_name(err));
    }
    nvs_close(handle);
}

static void send_current_params(void)
{
    std::string msg = "PARAMS:";
    char buf[32];

    for (size_t i = 0; i < PARAM_COUNT; i++) {
        float value = ble_params_get(PARAM_DEFAULTS[i].key);
        snprintf(buf, sizeof(buf), "%s=%.2f", PARAM_DEFAULTS[i].key, value);
        msg += buf;
        if (i < PARAM_COUNT - 1) {
            msg += ";";
        }
    }

    bt_serial_send((const uint8_t *)msg.data(), msg.size());
}

static void parse_and_save_params(const std::string &body)
{
    size_t start = 0;
    while (start < body.size()) {
        size_t sep = body.find(';', start);
        std::string pair = (sep == std::string::npos)
            ? body.substr(start)
            : body.substr(start, sep - start);

        size_t eq = pair.find('=');
        if (eq != std::string::npos) {
            std::string key = pair.substr(0, eq);
            std::string value_str = pair.substr(eq + 1);
            ble_params_set(key.c_str(), strtof(value_str.c_str(), nullptr));
        }

        if (sep == std::string::npos) break;
        start = sep + 1;
    }

    static const char *msg = "PARAMS_SAVED";
    bt_serial_send((const uint8_t *)msg, strlen(msg));
}

bool ble_params_handle_command(const char *cmd)
{
    std::string c(cmd);

    if (c == "GET_PARAMS") {
        send_current_params();
        return true;
    }

    static const std::string prefix = "SET_PARAMS:";
    if (c.rfind(prefix, 0) == 0) {
        parse_and_save_params(c.substr(prefix.size()));
        return true;
    }

    // Parâmetro individual: "GET_PARAM:kp" -> responde "PARAM:kp=1.20"
    static const std::string get_one_prefix = "GET_PARAM:";
    if (c.rfind(get_one_prefix, 0) == 0) {
        std::string key = c.substr(get_one_prefix.size());
        float value = ble_params_get(key.c_str());
        char buf[48];
        snprintf(buf, sizeof(buf), "PARAM:%s=%.2f", key.c_str(), value);
        bt_serial_send((const uint8_t *)buf, strlen(buf));
        return true;
    }

    // Parâmetro individual: "SET_PARAM:kp=1.20" -> grava só esse, responde "PARAM_SAVED:kp"
    static const std::string set_one_prefix = "SET_PARAM:";
    if (c.rfind(set_one_prefix, 0) == 0) {
        std::string pair = c.substr(set_one_prefix.size());
        size_t eq = pair.find('=');
        if (eq != std::string::npos) {
            std::string key = pair.substr(0, eq);
            std::string value_str = pair.substr(eq + 1);
            ble_params_set(key.c_str(), strtof(value_str.c_str(), nullptr));

            char buf[48];
            snprintf(buf, sizeof(buf), "PARAM_SAVED:%s", key.c_str());
            bt_serial_send((const uint8_t *)buf, strlen(buf));
        }
        return true;
    }

    return false;
}
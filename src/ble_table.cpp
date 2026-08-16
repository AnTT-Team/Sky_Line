// ble_table.cpp
// Armazena a TABELA de leituras do encoder inteira na flash (NVS), como um
// único blob de texto — separado do envio de leituras em tempo real.
//
// Formato do blob: linhas separadas por "|", cada linha "quantidade,tipo,velocidade"
//   Exemplo: "250,RETA,1.35|300,CURVA,0.90|180,RETA,1.50"
//
// Uma escrita/notificação BLE só carrega o que o MTU da conexão permitir —
// por isso a tabela é transmitida em PEDAÇOS (chunks), tanto para salvar
// quanto para ler, e cada lado remonta a string completa antes de usar.
//
// Protocolo (ver mobile-flutter/lib/ble_controller.dart, método sendChunked):
//   Salvar (App -> ESP32):
//     "SET_TABLE_BEGIN"          -> limpa o buffer de recebimento
//     "SET_TABLE_CHUNK:<pedaço>" -> acrescenta esse pedaço ao buffer (várias vezes)
//     "SET_TABLE_END"            -> grava o buffer inteiro na flash, responde "TABLE_SAVED"
//
//   Ler (App -> ESP32: "GET_TABLE"; ESP32 -> App, via notify):
//     "TABLE_BEGIN"
//     "TABLE_CHUNK:<pedaço>"     (enviados aos poucos, por uma task dedicada)
//     "TABLE_END"

#include "ble_table.h"

#include <cstring>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"

// Implementado em ble_nimble_uart.cpp
extern "C" void bt_serial_send(const uint8_t *data, size_t len);

static const char *TAG = "BLE_TABLE";
static const char *NVS_NAMESPACE = "robo_table";
static const char *NVS_KEY = "encoder_tbl";

// Tamanho de cada pedaço. 150 bytes bate com o padrão usado no app Flutter
// (mobile-flutter/lib/ble_controller.dart, sendChunked).
static const size_t CHUNK_SIZE = 150;
static const int CHUNK_GAP_MS = 50; // intervalo entre notificações

// ---- Recebendo uma tabela grande (SET_TABLE_BEGIN/CHUNK/END) ----
static std::string s_incoming;

// ---- Enviando uma tabela grande (GET_TABLE), consumida pela task ----
static std::string s_outgoing;
static size_t s_outgoing_offset = 0;
static volatile bool s_sending = false;

static std::string load_table_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return ""; // nunca salvou nada ainda
    }

    size_t required_size = 0;
    err = nvs_get_str(handle, NVS_KEY, nullptr, &required_size);
    if (err != ESP_OK || required_size == 0) {
        nvs_close(handle);
        return "";
    }

    std::string result(required_size - 1, '\0'); // required_size inclui o '\0'
    err = nvs_get_str(handle, NVS_KEY, &result[0], &required_size);
    nvs_close(handle);

    return (err == ESP_OK) ? result : "";
}

static void save_table_to_nvs(const std::string &blob)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open (RW) falhou: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_str(handle, NVS_KEY, blob.c_str());
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao salvar tabela: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

bool ble_table_handle_command(const char *cmd)
{
    std::string c(cmd);

    if (c == "GET_TABLE") {
        s_outgoing = load_table_from_nvs();
        s_outgoing_offset = 0;
        s_sending = true;
        static const char *msg = "TABLE_BEGIN";
        bt_serial_send((const uint8_t *)msg, strlen(msg));
        return true;
    }

    if (c == "SET_TABLE_BEGIN") {
        s_incoming.clear();
        return true;
    }

    static const std::string chunk_prefix = "SET_TABLE_CHUNK:";
    if (c.rfind(chunk_prefix, 0) == 0) {
        s_incoming += c.substr(chunk_prefix.size());
        return true;
    }

    if (c == "SET_TABLE_END") {
        save_table_to_nvs(s_incoming);
        s_incoming.clear();
        static const char *msg = "TABLE_SAVED";
        bt_serial_send((const uint8_t *)msg, strlen(msg));
        return true;
    }

    return false;
}

// Task dedicada: envia um pedaço da tabela por vez, respeitando um
// intervalo entre notificações (mandar tudo de uma vez, sem pausa, pode
// fazer o BLE stack descartar notificações).
static void ble_table_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CHUNK_GAP_MS));

        if (!s_sending) {
            continue;
        }
        size_t remaining = s_outgoing.size() - s_outgoing_offset;
        if (remaining == 0) {
            s_sending = false;
            static const char *msg = "TABLE_END";
            bt_serial_send((const uint8_t *)msg, strlen(msg));
            continue;
        }
        
        size_t take = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
        std::string msg = "TABLE_CHUNK:" + s_outgoing.substr(s_outgoing_offset, take);
        ESP_LOGI("LINE_FOLLOWER", "ble_table_task: enviando chunk,  data=%s", msg.c_str());
        bt_serial_send((const uint8_t *)msg.data(), msg.size());
        s_outgoing_offset += take;
    }
}

void ble_table_init(void)
{
    xTaskCreate(ble_table_task, "ble_table_task", 4096, NULL, 4, NULL);
}
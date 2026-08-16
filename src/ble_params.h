#pragma once
#include <cstdint>

// Lê/grava um parâmetro individual na flash (NVS). Útil pra usar direto no
// código do robô, ex: g_kp_dir = ble_params_get("kp");
float ble_params_get(const char *key);
void ble_params_set(const char *key, float value);

// Trata comandos BLE de parâmetros (GET_PARAMS / SET_PARAMS:...).
// Retorna true se o comando foi reconhecido e tratado (resposta já enviada
// via bt_serial_send).
bool ble_params_handle_command(const char *cmd);
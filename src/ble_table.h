#pragma once
#include <cstdint>

// Trata comandos BLE da tabela de encoder (GET_TABLE / SET_TABLE_BEGIN /
// SET_TABLE_CHUNK / SET_TABLE_END). Retorna true se o comando foi tratado.
bool ble_table_handle_command(const char *cmd);

// Cria a task que envia os pedaços (chunks) de uma leitura de tabela
// (GET_TABLE) aos poucos, sem travar o resto do sistema. Chame uma vez em
// app_main, depois de nimble_uart_init().
void ble_table_init(void);
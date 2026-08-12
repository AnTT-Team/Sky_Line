// ble_nimble_uart.c - Configuração NimBLE + UART BLE simples ("0"/"1")

#include "sdkconfig.h"

#include <stdio.h>

#include <string.h>

#include <assert.h>

 

#include "esp_log.h"

#include "esp_nimble_hci.h"

 

#include "nimble/nimble_port.h"

#include "nimble/nimble_port_freertos.h"

 

#include "host/ble_hs.h"

#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"

#include "services/gatt/ble_svc_gatt.h"

#include "os/os_mbuf.h"

 

// Implementado no main.c

extern void bt_serial_rx_push(const uint8_t *data, uint16_t len);

 

static const char *TAG_BLE = "NIMBLE_UART";

 

// UUIDs simples (16 bits) para o serviço UART e características

#define UART_SVC_UUID       0xFFF0

#define UART_CHR_RX_UUID    0xFFF1  // write (app -> ESP32)

#define UART_CHR_TX_UUID    0xFFF2  // notify (ESP32 -> app)

 

static uint16_t g_conn_handle = 0;       // conexão atual

static uint16_t g_uart_tx_val_handle = 0; // handle da característica TX (notify)

 

// ==================== GATT CALLBACK (READ/WRITE) ====================

 

static int

gatt_svr_chr_access_uart(uint16_t conn_handle,

                         uint16_t attr_handle,

                         struct ble_gatt_access_ctxt *ctxt,

                         void *arg)

{

    (void)conn_handle;

    (void)attr_handle;

    (void)arg;

 

    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {

        // Dados escritos na característica RX -> repassa para bt_serial_rx_push

        uint8_t buf[64];

        int len = OS_MBUF_PKTLEN(ctxt->om);

        if (len > (int)sizeof(buf)) {

            len = sizeof(buf);

        }

        if (len > 0) {

            os_mbuf_copydata(ctxt->om, 0, len, buf);

            bt_serial_rx_push(buf, (uint16_t)len);

        }

        return 0;

    }

 

    case BLE_GATT_ACCESS_OP_READ_CHR:

        // Não usamos leitura em nenhuma característica

        return 0;

 

    default:

        return BLE_ATT_ERR_UNLIKELY;

    }

}

 

// ==================== DEFINIÇÃO DO SERVIÇO GATT ====================

 

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {

    {

        // Serviço UART

        .type = BLE_GATT_SVC_TYPE_PRIMARY,

        .uuid = BLE_UUID16_DECLARE(UART_SVC_UUID),

        .characteristics = (struct ble_gatt_chr_def[])

        {

            {   // RX: write / write no response

                .uuid = BLE_UUID16_DECLARE(UART_CHR_RX_UUID),

                .access_cb = gatt_svr_chr_access_uart,

                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,

            },

            {   // TX: notify

                .uuid = BLE_UUID16_DECLARE(UART_CHR_TX_UUID),

                .access_cb = gatt_svr_chr_access_uart,

                .val_handle = &g_uart_tx_val_handle,

                .flags = BLE_GATT_CHR_F_NOTIFY,

            },

            { 0 } // fim das characteristics

        },

    },

 

    { 0 } // fim dos serviços

};

 

// ==================== ADVERTISE ====================

 

static uint8_t g_own_addr_type;

 

static void

ble_app_advertise(void);

 

static int

ble_gap_event_cb(struct ble_gap_event *event, void *arg)

{

    (void)arg;

 

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:

        if (event->connect.status == 0) {

            ESP_LOGI("LINE_FOLLOWER", "Conectado, handle=%d", event->connect.conn_handle);

            g_conn_handle = event->connect.conn_handle;

        } else {

            ESP_LOGW("LINE_FOLLOWER", "Falha na conexao; status=%d. Re-anunciando.", event->connect.status);

            ble_app_advertise();

        }

        return 0;

 

    case BLE_GAP_EVENT_DISCONNECT:

        ESP_LOGI("LINE_FOLLOWER", "Desconectado; motivo=%d", event->disconnect.reason);

        g_conn_handle = 0;

        ble_app_advertise();

        return 0;

 

    case BLE_GAP_EVENT_ADV_COMPLETE:

        ESP_LOGI("LINE_FOLLOWER", "Advertising complete; reiniciando");

        ble_app_advertise();

        return 0;

 

    case BLE_GAP_EVENT_MTU:

        ESP_LOGI("LINE_FOLLOWER", "MTU updated: %d", event->mtu.value);

        return 0;

 

    default:

        return 0;

    }

}

 

static void

ble_app_advertise(void)

{

    struct ble_gap_adv_params adv_params;

    struct ble_hs_adv_fields fields;

    const char *name;

    int rc;

 

    memset(&fields, 0, sizeof(fields));

 

    name = ble_svc_gap_device_name();

    fields.name = (uint8_t *)name;

    fields.name_len = strlen(name);

    fields.name_is_complete = 1;

 

    fields.flags = BLE_HS_ADV_F_DISC_GEN |

                   BLE_HS_ADV_F_BREDR_UNSUP;

 

    rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0) {

        ESP_LOGE("LINE_FOLLOWER", "ble_gap_adv_set_fields falhou; rc=%d", rc);

        return;

    }

 

    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable undirected

    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // general discoverable

 

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,

                           &adv_params, ble_gap_event_cb, NULL);

    if (rc != 0) {

        ESP_LOGE("LINE_FOLLOWER", "ble_gap_adv_start falhou; rc=%d", rc);

    } else {

        ESP_LOGI("LINE_FOLLOWER", "Advertising iniciado");

    }

}

 

// ==================== CALLBACKS DO HOST ====================

 

static void

ble_app_on_reset(int reason)

{

    ESP_LOGW(TAG_BLE, "Reset do host BLE; motivo=%d", reason);

}

 

static void

ble_app_on_sync(void)

{

    int rc;

 

    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);

    assert(rc == 0);

 

    uint8_t addr_val[6] = {0};

    ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);

    ESP_LOGI("LINE_FOLLOWER", "Endereço BLE: %02X:%02X:%02X:%02X:%02X:%02X",

             addr_val[5], addr_val[4], addr_val[3],

             addr_val[2], addr_val[1], addr_val[0]);

 

    ble_svc_gap_device_name_set("ESP32S3-LineFollower");

 

    ble_app_advertise();

}

 

static void

host_task(void *param)

{

    (void)param;

    nimble_port_run(); // só retorna se nimble_port_stop() for chamado

    nimble_port_freertos_deinit();

}

 

// ==================== API PÚBLICA ====================

//

// Chamar nimble_uart_init() no app_main, antes de usar bt_serial.

//

 

void nimble_uart_init(void)

{

    int rc;

 

    ESP_LOGI("LINE_FOLLOWER", "Iniciando NimBLE...");

 

    // NÃO chamar esp_nimble_hci_init / esp_nimble_hci_and_controller_init

    rc = nimble_port_init();

    if (rc != 0) {

        ESP_LOGE("LINE_FOLLOWER", "nimble_port_init falhou; rc=%d", rc);

        return;

    }

 

    // Serviços padrão GAP/GATT

    ble_svc_gap_init();

    ble_svc_gatt_init();

 

    // Registra nossos serviços UART

    rc = ble_gatts_count_cfg(gatt_svr_svcs);

    assert(rc == 0);

 

    rc = ble_gatts_add_svcs(gatt_svr_svcs);

    assert(rc == 0);

 

    // Callbacks do host

    ble_hs_cfg.reset_cb = ble_app_on_reset;

    ble_hs_cfg.sync_cb  = ble_app_on_sync;

 

    // Nome do dispositivo (se ainda não tiver feito isso no on_sync)

    ble_svc_gap_device_name_set("ESP32S3-LineFollower");

 

    // Inicia a task do host NimBLE

    nimble_port_freertos_init(host_task);

 

    ESP_LOGI("LINE_FOLLOWER", "NimBLE iniciado");

}

 

// ==================== IMPLEMENTAÇÃO DE bt_serial_send ====================

//

// Usa notify na característica TX para enviar dados ao cliente BLE.

//

 

void bt_serial_send(const uint8_t *data, size_t len)

{

    if (g_conn_handle == 0 || g_uart_tx_val_handle == 0 || data == NULL || len == 0) {

        return; // sem conexão ou sem handle

    }

 

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);

    if (!om) {

        ESP_LOGW("LINE_FOLLOWER", "Falha ao alocar mbuf para notify");

        return;

    }

 

    int rc = ble_gatts_notify_custom(g_conn_handle, g_uart_tx_val_handle, om);

    if (rc != 0) {

        ESP_LOGW("LINE_FOLLOWER", "ble_gatts_notify_custom falhou; rc=%d", rc);

        os_mbuf_free_chain(om);

    }

}
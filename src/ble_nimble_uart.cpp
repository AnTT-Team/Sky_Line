// ble_nimble_uart.cpp - Configuração NimBLE + UART BLE simples ("M0"/"M1")

#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

// Implementado no main.cpp
extern "C" void bt_serial_rx_push(const uint8_t *data, uint16_t len);

static const char *TAG_BLE = "NIMBLE_UART";

// UUIDs 128-bit do serviço UART e das características
static const ble_uuid128_t UART_SVC_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24,
    0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5,
    0x01, 0x00, 0x40, 0x6E
);

static const ble_uuid128_t UART_CHR_RX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24,
    0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5,
    0x02, 0x00, 0x40, 0x6E
);
// write (app -> ESP32)
static const ble_uuid128_t UART_CHR_TX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24,
    0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5,
    0x03, 0x00, 0x40, 0x6E
);
// notify (ESP32 -> app)

static uint16_t g_conn_handle = 0;
static uint16_t g_uart_tx_val_handle = 0;
static uint16_t mtu  = 64;


// ==================== GATT CALLBACK (READ/WRITE) ====================
static int gatt_svr_chr_access_uart(uint16_t conn_handle,
                                   uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t buf[mtu];
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
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// ==================== DEFINIÇÃO DO SERVIÇO GATT ====================
static struct ble_gatt_chr_def uart_chars[3];
static struct ble_gatt_svc_def gatt_svr_svcs[2];

static void ble_gatt_init_uart_service(void)
{
    memset(uart_chars, 0, sizeof(uart_chars));
    memset(gatt_svr_svcs, 0, sizeof(gatt_svr_svcs));

    uart_chars[0].uuid = (const ble_uuid_t *)&UART_CHR_RX_UUID;
    uart_chars[0].access_cb = gatt_svr_chr_access_uart;
    uart_chars[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;

    uart_chars[1].uuid = (const ble_uuid_t *)&UART_CHR_TX_UUID;
    uart_chars[1].access_cb = gatt_svr_chr_access_uart;
    uart_chars[1].val_handle = &g_uart_tx_val_handle;
    uart_chars[1].flags = BLE_GATT_CHR_F_NOTIFY;

    gatt_svr_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    gatt_svr_svcs[0].uuid = (const ble_uuid_t *)&UART_SVC_UUID;
    gatt_svr_svcs[0].characteristics = uart_chars;
}

// ==================== ADVERTISE ====================

static uint8_t g_own_addr_type;
static void ble_app_advertise(void);

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
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
            mtu = event->mtu.value;
            return 0;

        default:
            return 0;
    }
}

static void ble_app_advertise(void)
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


    // Anuncia o serviço UART
    fields.uuids128 = &UART_SVC_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE("LINE_FOLLOWER", "ble_gap_adv_set_fields falhou; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);

    if (rc != 0) {
        ESP_LOGE("LINE_FOLLOWER", "ble_gap_adv_start falhou; rc=%d", rc);
    } else {
        ESP_LOGI("LINE_FOLLOWER", "Advertising iniciado");
    }
}

// ==================== CALLBACKS DO HOST ====================

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG_BLE, "Reset do host BLE; motivo=%d", reason);
}

static void ble_app_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);

    ESP_LOGI("LINE_FOLLOWER", "Endereço BLE: %02X:%02X:%02X:%02X:%02X:%02X",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    ble_svc_gap_device_name_set("ESP32S3");
    ble_app_advertise();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ==================== API PÚBLICA ====================
extern "C" void nimble_uart_init(void)
{
    int rc;
    ESP_LOGI("LINE_FOLLOWER", "Iniciando NimBLE...");

    rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE("LINE_FOLLOWER", "nimble_port_init falhou; rc=%d", rc);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatt_init_uart_service();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb  = ble_app_on_sync;

    ble_svc_gap_device_name_set("ESP32S3");

    nimble_port_freertos_init(host_task);
    ESP_LOGI("LINE_FOLLOWER", "NimBLE iniciado");
}

// ==================== IMPLEMENTAÇÃO DE bt_serial_send ====================
extern "C" void bt_serial_send(const uint8_t *data, size_t len)
{
    struct ble_gap_conn_desc desc;  
    // Tenta encontrar a conexão ativa pelo handle
    int conn_check = ble_gap_conn_find(g_conn_handle, &desc);

    if (conn_check != 0 || g_uart_tx_val_handle == 0 || data == NULL || len == 0) {
        return;
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
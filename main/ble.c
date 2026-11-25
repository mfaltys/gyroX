/* gyroX — BLE configuration + OTA over NimBLE (ESP-IDF 6.0.1, ESP32-C3)
 *
 * GATT service: 0000A100-0000-1000-8000-00805F9B34FB
 *
 *   0xA101 DEVICE_INFO   READ    static JSON {version, model, ble_ver}
 *   0xA102 CONFIG        WRITE+NOTIFY JSON command/response channel
 *   0xA103 OTA_CTRL      WRITE   JSON {cmd:begin|abort|complete,...}
 *   0xA104 OTA_DATA      WRITE_NO_RSP raw binary firmware chunks
 *
 * The CONFIG channel reuses config_set_from_json() so one JSON schema serves
 * both the HTTP web UI and the BLE client. OTA reuses esp_ota_begin/write/end
 * exactly like main/web.c — same inactive-slot swap, same bootloader fallback.
 *
 * WiFi/BLE coexistence: ble_stop() is called from web_start() before the AP
 * comes up, so BLE and WiFi never run at the same time.
 */

#include "ble.h"
#include "config.h"
#include "gyro.h"
#include "mixing.h"
#include "leds.h"

#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_app_desc.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <nimble/nimble_port.h>
#include <host/ble_hs.h>
#include <host/ble_hs_adv.h>
#include <host/ble_uuid.h>
#include <host/ble_gatt.h>
#include <host/ble_gap.h>
#include <services/gap/ble_svc_gap.h>

static const char *TAG = "BLE";

/* ------------------------------------------------------------------ */
/* GATT UUIDs (128-bit, base 0000xxxx-0000-1000-8000-00805F9B34FB)    */
/* ------------------------------------------------------------------ */

static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x00, 0xA1, 0x00, 0x00);

static const ble_uuid128_t chr_info_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0xA1, 0x00, 0x00);

static const ble_uuid128_t chr_cfg_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0xA1, 0x00, 0x00);

static const ble_uuid128_t chr_otac_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x03, 0xA1, 0x00, 0x00);

static const ble_uuid128_t chr_otad_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x04, 0xA1, 0x00, 0x00);

/* ------------------------------------------------------------------ */
/* Handles                                                            */
/* ------------------------------------------------------------------ */

static uint16_t chr_info_handle;
static uint16_t chr_cfg_handle;
static uint16_t chr_otac_handle;
static uint16_t chr_otad_handle;

static bool s_ble_running = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* ------------------------------------------------------------------ */
/* OTA state                                                          */
/* ------------------------------------------------------------------ */

static SemaphoreHandle_t s_ota_mutex;

static struct {
    bool in_progress;
    esp_ota_handle_t ota;
    const esp_partition_t *part;
    int32_t total_size;
    int32_t written_size;
} s_ota = {0};

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static void notify_json(uint16_t conn_handle, uint16_t attr_handle, const char *json)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, strlen(json));
    if (om) {
        ble_gatts_notify_custom(conn_handle, attr_handle, om);
    }
}
static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uintptr_t chr_id = (uintptr_t)arg;
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (chr_id == 1) { /* DEVICE_INFO */
            const esp_app_desc_t *desc = esp_app_get_description();
            char buf[256];
            int n = snprintf(buf, sizeof(buf),
                             "{\"version\":\"%s\",\"model\":\"gyroX\",\"ble_ver\":1}",
                             desc ? desc->version : "unknown");
            if (n < 0 || n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
            rc = os_mbuf_append(ctxt->om, buf, n);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (chr_id == 2) { /* CONFIG — dump current config as JSON */
            cJSON *r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "gyro_gain", s_cfg.gyro_gain);
            cJSON_AddNumberToObject(r, "gyro_mode", s_cfg.gyro_mode);
            cJSON_AddNumberToObject(r, "gyro_reverse", s_cfg.gyro_reverse);
            cJSON_AddNumberToObject(r, "steer_reverse", s_cfg.steer_reverse);
            cJSON_AddNumberToObject(r, "steer_deadband", s_cfg.steer_deadband);
            cJSON_AddNumberToObject(r, "steer_trim", s_cfg.steer_trim);
            cJSON_AddNumberToObject(r, "servo_min_us", s_cfg.servo_min_us);
            cJSON_AddNumberToObject(r, "servo_center_us", s_cfg.servo_center_us);
            cJSON_AddNumberToObject(r, "servo_max_us", s_cfg.servo_max_us);
            cJSON_AddNumberToObject(r, "hold_p", s_cfg.hold_p);
            cJSON_AddNumberToObject(r, "gyro_z_offset", (double)s_cfg.gyro_z_offset);
            cJSON_AddNumberToObject(r, "gyro_dlpf", s_cfg.gyro_dlpf);
            cJSON_AddNumberToObject(r, "gyro_fs", s_cfg.gyro_fs);
            cJSON_AddNumberToObject(r, "profile", s_cfg.profile);
            cJSON_AddNumberToObject(r, "leds_on", s_cfg.leds_on);
            cJSON_AddNumberToObject(r, "leds_color", (double)s_cfg.leds_color);
            cJSON_AddNumberToObject(r, "leds_bright", s_cfg.leds_bright);
            cJSON_AddStringToObject(r, "web_ap_ssid", s_cfg.web_ap_ssid);
            char *json = cJSON_PrintUnformatted(r);
            cJSON_Delete(r);
            if (!json) return BLE_ATT_ERR_INSUFFICIENT_RES;
            rc = os_mbuf_append(ctxt->om, json, strlen(json));
            free(json);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_ATTR_NOT_FOUND;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t buf[512];
        /* Use the full packet length, not om_len (a large write may span
         * multiple mbufs in the chain), then copy across the chain. */
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == 0 || len >= sizeof(buf)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        buf[len] = '\0';

        if (chr_id == 2) { /* CONFIG — JSON command */
            cJSON *root = cJSON_Parse((char *)buf);
            if (!root || !cJSON_IsObject(root)) {
                if (root) cJSON_Delete(root);
                const char *err = "{\"error\":\"invalid JSON\"}";
                if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    notify_json(conn_handle, chr_cfg_handle, err);
                }
                return 0;
            }

            cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
            const char *cmd_s = cmd && cJSON_IsString(cmd) ? cmd->valuestring : "";

            if (strcmp(cmd_s, "get_config") == 0) {
                cJSON_Delete(root);
                cJSON *r = cJSON_CreateObject();
                cJSON_AddBoolToObject(r, "ok", true);
                cJSON_AddNumberToObject(r, "gyro_gain", s_cfg.gyro_gain);
                cJSON_AddNumberToObject(r, "gyro_mode", s_cfg.gyro_mode);
                cJSON_AddNumberToObject(r, "gyro_reverse", s_cfg.gyro_reverse);
                cJSON_AddNumberToObject(r, "steer_reverse", s_cfg.steer_reverse);
                cJSON_AddNumberToObject(r, "steer_deadband", s_cfg.steer_deadband);
                cJSON_AddNumberToObject(r, "steer_trim", s_cfg.steer_trim);
                cJSON_AddNumberToObject(r, "servo_min_us", s_cfg.servo_min_us);
                cJSON_AddNumberToObject(r, "servo_center_us", s_cfg.servo_center_us);
                cJSON_AddNumberToObject(r, "servo_max_us", s_cfg.servo_max_us);
                cJSON_AddNumberToObject(r, "hold_p", s_cfg.hold_p);
                cJSON_AddNumberToObject(r, "gyro_z_offset", (double)s_cfg.gyro_z_offset);
                cJSON_AddNumberToObject(r, "gyro_dlpf", s_cfg.gyro_dlpf);
                cJSON_AddNumberToObject(r, "gyro_fs", s_cfg.gyro_fs);
                cJSON_AddNumberToObject(r, "profile", s_cfg.profile);
                cJSON_AddNumberToObject(r, "leds_on", s_cfg.leds_on);
                cJSON_AddNumberToObject(r, "leds_color", (double)s_cfg.leds_color);
                cJSON_AddNumberToObject(r, "leds_bright", s_cfg.leds_bright);
                cJSON_AddStringToObject(r, "web_ap_ssid", s_cfg.web_ap_ssid);
                char *json = cJSON_PrintUnformatted(r);
                cJSON_Delete(r);
                if (json) {
                    notify_json(conn_handle, chr_cfg_handle, json);
                    free(json);
                }
                return 0;
            }

            if (strcmp(cmd_s, "set_config") == 0) {
                esp_err_t err = config_set_from_json(root);
                cJSON_Delete(root);
                if (err == ESP_OK) {
                    gyro_apply_sensor_config();
                    leds_apply();
                    const char *resp = "{\"ok\":true}";
                    notify_json(conn_handle, chr_cfg_handle, resp);
                } else {
                    const char *resp = "{\"error\":\"config_save_failed\"}";
                    notify_json(conn_handle, chr_cfg_handle, resp);
                }
                return 0;
            }

            if (strcmp(cmd_s, "get_info") == 0) {
                cJSON_Delete(root);
                const esp_app_desc_t *desc = esp_app_get_description();
                char json[256];
                int n = snprintf(json, sizeof(json),
                                 "{\"ok\":true,\"version\":\"%s\",\"model\":\"gyroX\",\"ble_ver\":1}",
                                 desc ? desc->version : "unknown");
                if (n < 0 || n >= (int)sizeof(json)) n = sizeof(json) - 1;
                notify_json(conn_handle, chr_cfg_handle, json);
                return 0;
            }

            if (strcmp(cmd_s, "calibrate") == 0) {
                cJSON_Delete(root);
                esp_err_t err = gyro_calibrate(GYRO_CAL_SAMPLES);
                if (err == ESP_OK) {
                    notify_json(conn_handle, chr_cfg_handle, "{\"ok\":true}");
                } else {
                    notify_json(conn_handle, chr_cfg_handle, "{\"error\":\"calibrate_failed\"}");
                }
                return 0;
            }

            if (strcmp(cmd_s, "reset_heading") == 0) {
                cJSON_Delete(root);
                mixing_reset_heading();
                notify_json(conn_handle, chr_cfg_handle, "{\"ok\":true}");
                return 0;
            }

            if (strcmp(cmd_s, "get_state") == 0) {
                cJSON_Delete(root);
                const mixing_state_t *m = mixing_get_state();
                cJSON *r = cJSON_CreateObject();
                cJSON_AddNumberToObject(r, "rx", m->rx);
                cJSON_AddNumberToObject(r, "rate_z", m->rate_z);
                cJSON_AddNumberToObject(r, "heading", m->heading);
                cJSON_AddNumberToObject(r, "output", m->output);
                cJSON_AddBoolToObject(r, "rx_valid", m->rx_valid);
                cJSON_AddBoolToObject(r, "gyro_ready", m->gyro_ready);
                char *json = cJSON_PrintUnformatted(r);
                if (json) {
                    notify_json(conn_handle, chr_cfg_handle, json);
                    free(json);
                }
                cJSON_Delete(r);   /* was missing — leaked every 200ms poll until heap exhausted */
                return 0;
            }

            cJSON_Delete(root);
            const char *resp = "{\"error\":\"unknown_cmd\"}";
            notify_json(conn_handle, chr_cfg_handle, resp);
            return 0;
        }

        if (chr_id == 3) { /* OTA_CTRL — JSON control */
            cJSON *root = cJSON_Parse((char *)buf);
            if (!root || !cJSON_IsObject(root)) {
                if (root) cJSON_Delete(root);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
            const char *cmd_s = cmd && cJSON_IsString(cmd) ? cmd->valuestring : "";

            if (strcmp(cmd_s, "begin") == 0) {
                cJSON *sz = cJSON_GetObjectItem(root, "size");
                int32_t size = sz && cJSON_IsNumber(sz) ? (int32_t)sz->valueint : 0;

                if (s_ota.in_progress) {
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
                if (!update || size <= 0 || (int32_t)update->size < size) {
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                esp_ota_handle_t ota = 0;
                esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ota);
                if (err != ESP_OK) {
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                if (s_ota_mutex == NULL) {
                    s_ota_mutex = xSemaphoreCreateMutex();
                }
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                s_ota.in_progress = true;
                s_ota.ota = ota;
                s_ota.part = update;
                s_ota.total_size = size;
                s_ota.written_size = 0;
                xSemaphoreGive(s_ota_mutex);

                ESP_LOGI(TAG, "OTA begin: %d bytes -> %s", (int)size, update->label);
                cJSON_Delete(root);
                return 0;
            }

            if (strcmp(cmd_s, "abort") == 0) {
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                if (s_ota.in_progress) {
                    esp_ota_abort(s_ota.ota);
                    memset(&s_ota, 0, sizeof(s_ota));
                }
                xSemaphoreGive(s_ota_mutex);
                ESP_LOGW(TAG, "OTA aborted by client");
                cJSON_Delete(root);
                return 0;
            }

            if (strcmp(cmd_s, "complete") == 0) {
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                bool in_progress = s_ota.in_progress;
                esp_ota_handle_t ota = s_ota.ota;
                const esp_partition_t *part = s_ota.part;
                memset(&s_ota, 0, sizeof(s_ota));
                xSemaphoreGive(s_ota_mutex);

                if (!in_progress) {
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                esp_err_t err = esp_ota_end(ota);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                err = esp_ota_set_boot_partition(part);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
                    cJSON_Delete(root);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }

                ESP_LOGI(TAG, "OTA complete: %d bytes -> %s, rebooting",
                         (int)part->size, part->label);
                cJSON_Delete(root);

                if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    notify_json(conn_handle, chr_cfg_handle,
                                "{\"ok\":true,\"reboot\":true}");
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                esp_restart();
                return 0;
            }

            cJSON_Delete(root);
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }

        if (chr_id == 4) { /* OTA_DATA — raw binary, WRITE_NO_RSP */
            xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
            bool ok = s_ota.in_progress && s_ota.part != NULL;
            esp_ota_handle_t ota = s_ota.ota;
            int32_t written = s_ota.written_size;
            xSemaphoreGive(s_ota_mutex);

            if (!ok) {
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
            }

            esp_err_t err = esp_ota_write(ota, buf, len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                s_ota.in_progress = false;
                esp_ota_abort(ota);
                memset(&s_ota, 0, sizeof(s_ota));
                xSemaphoreGive(s_ota_mutex);
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
            }

            xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
            s_ota.written_size = written + (int32_t)len;
            xSemaphoreGive(s_ota_mutex);
            return 0; /* no response for WRITE_NO_RSP */
        }

        return BLE_ATT_ERR_ATTR_NOT_FOUND;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* ------------------------------------------------------------------ */
/* GATT registration callback — capture assigned value handles        */
/* ------------------------------------------------------------------ */

static void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (ctxt->op != BLE_GATT_REGISTER_OP_CHR) return;

    if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &chr_info_uuid.u) == 0) {
        chr_info_handle = ctxt->chr.val_handle;
    } else if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &chr_cfg_uuid.u) == 0) {
        chr_cfg_handle = ctxt->chr.val_handle;
    } else if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &chr_otac_uuid.u) == 0) {
        chr_otac_handle = ctxt->chr.val_handle;
    } else if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &chr_otad_uuid.u) == 0) {
        chr_otad_handle = ctxt->chr.val_handle;
    }
}

/* ------------------------------------------------------------------ */
/* GATT service definition                                            */
/* ------------------------------------------------------------------ */

static struct ble_gatt_chr_def gyro_chars[] = {
    {
        .uuid = &chr_info_uuid.u,
        .access_cb = chr_access_cb,
        .arg = (void *)1, /* DEVICE_INFO */
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &chr_cfg_uuid.u,
        .access_cb = chr_access_cb,
        .arg = (void *)2, /* CONFIG */
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        .uuid = &chr_otac_uuid.u,
        .access_cb = chr_access_cb,
        .arg = (void *)3, /* OTA_CTRL */
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        .uuid = &chr_otad_uuid.u,
        .access_cb = chr_access_cb,
        .arg = (void *)4, /* OTA_DATA */
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        0, /* terminator */
    },
};

static struct ble_gatt_svc_def gyro_svc_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = gyro_chars,
    },
    {
        0, /* terminator */
    },
};

/* ------------------------------------------------------------------ */
/* Host task                                                          */
/* ------------------------------------------------------------------ */

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    /* nimble_port_run() only returns on shutdown */
    s_ble_running = false;
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* GAP event callback (forward declaration)                           */
/* ------------------------------------------------------------------ */

static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* ------------------------------------------------------------------ */
/* Advertising                                                        */
/* ------------------------------------------------------------------ */

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields adv_fields = {0};
    const char *name = "gyroX";

    /* Device name + flags in the advertising payload */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN;
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Advertise the 128-bit service UUID so Web Bluetooth can filter on it */
    adv_fields.uuids128 = &svc_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&adv_fields);

    /* CRITICAL: {0} params default to BLE_GAP_CONN_MODE_NON (visible in scans
     * but connection requests are ignored at the controller!). Must be
     * connectable-undirected + general-discoverable for Web Bluetooth. */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x0020;   /* 20 ms  (units of 0.625 ms) */
    adv_params.itvl_max = 0x0040;   /* 40 ms */

    /* GAP event callback is passed directly to ble_gap_adv_start() */
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                      gap_event_cb, NULL);
    ESP_LOGI(TAG, "Advertising as '%s' (connectable)", name);
}

/* ------------------------------------------------------------------ */
/* GAP event callback                                                 */
/* ------------------------------------------------------------------ */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE client connected");
        s_conn_handle = event->connect.conn_handle;
        if (event->connect.status != 0) {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE client disconnected");
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

esp_err_t ble_start(void)
{
    if (s_ble_running) {
        return ESP_OK;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Set device name */
    ble_svc_gap_device_name_set("gyroX");

    /* Register the GATT service */
    int rc = ble_gatts_count_cfg(gyro_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_ERR_INVALID_STATE;
    }
    rc = ble_gatts_add_svcs(gyro_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_ERR_INVALID_STATE;
    }

    /* No passkey display available — use Just Works pairing */
    ble_hs_cfg.sm_io_cap = BLE_SM_IOACT_NONE;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    /* Capture the value handles NimBLE assigns during GATT registration —
     * without these, notify_json() would target handle 0 and every JSON
     * response would be lost. */
    ble_hs_cfg.gatts_register_cb = gatt_register_cb;

    /* Start the host task */
    xTaskCreate(ble_host_task, "nimble_host", 4096, NULL, 5, NULL);

    /* Wait for the host to come up before configuring advertising */
    vTaskDelay(pdMS_TO_TICKS(500));

    ble_advertise();

    s_ble_running = true;
    ESP_LOGI(TAG, "BLE started");
    return ESP_OK;
}

void ble_stop(void)
{
    if (!s_ble_running) {
        return;
    }

    ble_gap_adv_stop();
    s_ble_running = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGI(TAG, "BLE stopped");
}

bool ble_is_running(void)
{
    return s_ble_running;
}


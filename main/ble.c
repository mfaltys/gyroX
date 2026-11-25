/* gyroX - BLE configuration + OTA over NimBLE (ESP-IDF 6.0.1, ESP32-C3)
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
 * exactly like main/web.c - same inactive-slot swap, same bootloader fallback.
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
#include <freertos/queue.h>
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

/* Flow-control: ack every OTA_ACK_SPAN bytes actually written to flash so
 * the client can pace its write window (WNR streaming without acks outruns
 * the flash pipeline and the transfer dies at Chrome's op timeout). */
#define OTA_ACK_SPAN 4096

static struct {
    bool in_progress;
    esp_ota_handle_t ota;
    const esp_partition_t *part;
    int32_t total_size;
    int32_t written_size;
    int32_t acked_size;
} s_ota = {0};

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

/* ---- Deferred notifications --------------------------------------- */
/* The response notification must reach the client AFTER the ATT write
 * response, or Chrome's single-slot GATT operation queue wedges: the
 * writeValue() promise never settles (the write response is lost when the
 * notification races it) and every later GATT op fails with "operation
 * already in progress". The access callback therefore only enqueues the
 * reply here; a worker task sends it a few ms later, once the write
 * response is safely out. */

typedef struct {
    uint16_t conn;
    uint16_t attr;
    char    *json;   /* owned by the worker - freed after send */
} ble_reply_t;

static QueueHandle_t s_reply_queue;

static char *dup_str(const char *s)
{
    size_t n = strlen(s) + 1;
    char *p = malloc(n);
    if (p) memcpy(p, s, n);
    return p;
}

static void reply_task(void *arg)
{
    (void)arg;
    ble_reply_t r;
    while (1) {
        if (xQueueReceive(s_reply_queue, &r, portMAX_DELAY) != pdTRUE) continue;
        /* Small defer so the ATT write response is emitted first. */
        vTaskDelay(pdMS_TO_TICKS(10));
        struct os_mbuf *om = ble_hs_mbuf_from_flat(r.json, strlen(r.json));
        if (om) {
            ble_gatts_notify_custom(r.conn, r.attr, om);
        }
        free(r.json);
    }
}

static void notify_json(uint16_t conn_handle, uint16_t attr_handle, const char *json)
{
    if (s_reply_queue == NULL) return;
    ble_reply_t r = {
        .conn = conn_handle,
        .attr = attr_handle,
        .json = dup_str(json),
    };
    if (r.json == NULL) return;
    /* Non-blocking: a full queue drops the reply rather than stalling the
     * NimBLE host task (which would stall the whole GATT server). */
    if (xQueueSend(s_reply_queue, &r, 0) != pdTRUE) {
        free(r.json);
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
            const char *dname = s_cfg.device_name[0] ? s_cfg.device_name : WEB_AP_SSID;
            int n = snprintf(buf, sizeof(buf),
                             "{\"version\":\"%s\",\"model\":\"gyroX\",\"name\":\"%s\",\"ble_ver\":2}",
                             desc ? desc->version : "unknown", dname);
            if (n < 0 || n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
            rc = os_mbuf_append(ctxt->om, buf, n);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (chr_id == 2) { /* CONFIG - dump current config as JSON */
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
            cJSON_AddStringToObject(r, "device_name", s_cfg.device_name);
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

        if (chr_id == 2) { /* CONFIG - JSON command */
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
                cJSON_AddStringToObject(r, "device_name", s_cfg.device_name);
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
                    /* Keep the GAP-read name in sync without a reboot; the
                     * advertisement rebuilds from s_cfg on next advertise. */
                    ble_svc_gap_device_name_set(s_cfg.device_name[0] ? s_cfg.device_name : WEB_AP_SSID);
                    ESP_LOGI(TAG, "Config saved to NVS");
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
                const char *dname = s_cfg.device_name[0] ? s_cfg.device_name : WEB_AP_SSID;
                int n = snprintf(json, sizeof(json),
                                 "{\"ok\":true,\"version\":\"%s\",\"model\":\"gyroX\",\"name\":\"%s\",\"ble_ver\":2}",
                                 desc ? desc->version : "unknown", dname);
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
                cJSON_AddBoolToObject(r, "gyro_on", m->gyro_on);
                char *json = cJSON_PrintUnformatted(r);
                if (json) {
                    notify_json(conn_handle, chr_cfg_handle, json);
                    free(json);
                }
                cJSON_Delete(r);   /* was missing - leaked every 200ms poll until heap exhausted */
                return 0;
            }

            if (strcmp(cmd_s, "get_slots") == 0) {
                cJSON_Delete(root);
                const esp_partition_t *running = esp_ota_get_running_partition();
                const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
                esp_app_desc_t d;
                char runv[64] = "unknown";
                char othv[64] = "";
                if (running && esp_ota_get_partition_description(running, &d) == ESP_OK) {
                    strncpy(runv, d.version, sizeof(runv) - 1);
                    runv[sizeof(runv) - 1] = '\0';
                }
                if (other && esp_ota_get_partition_description(other, &d) == ESP_OK) {
                    strncpy(othv, d.version, sizeof(othv) - 1);
                    othv[sizeof(othv) - 1] = '\0';
                }
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "running", runv);
                if (othv[0]) cJSON_AddStringToObject(r, "other", othv);
                else cJSON_AddNullToObject(r, "other");
                char *json = cJSON_PrintUnformatted(r);
                if (json) {
                    notify_json(conn_handle, chr_cfg_handle, json);
                    free(json);
                }
                cJSON_Delete(r);
                return 0;
            }

            if (strcmp(cmd_s, "rollback") == 0) {
                cJSON_Delete(root);
                ESP_LOGW(TAG, "Rollback requested via BLE - rebooting into the previous image");
                notify_json(conn_handle, chr_cfg_handle, "{\"ok\":true,\"rollback\":true}");
                /* Let the (deferred) ack reach the client, then invalidate the
                 * running image and reboot - the bootloader picks the other
                 * slot, which still holds the previous firmware. */
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_ota_mark_app_invalid_rollback_and_reboot();
                return 0; /* unreachable */
            }

            cJSON_Delete(root);
            const char *resp = "{\"error\":\"unknown_cmd\"}";
            notify_json(conn_handle, chr_cfg_handle, resp);
            return 0;
        }

        if (chr_id == 3) { /* OTA_CTRL - JSON control */
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
                s_ota.acked_size = 0;
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
                int32_t written = s_ota.written_size;
                int32_t total = s_ota.total_size;
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

                ESP_LOGI(TAG, "OTA complete: %d/%d bytes -> %s, rebooting",
                         (int)written, (int)total, part->label);
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

        if (chr_id == 4) { /* OTA_DATA - raw binary, WRITE_NO_RSP */
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
            int32_t now_written = s_ota.written_size;
            int32_t acked = s_ota.acked_size;
            int32_t total = s_ota.total_size;
            xSemaphoreGive(s_ota_mutex);

            /* Flow-control ack: report actual flash progress every span, and
             * always for the final chunk, so the client never gets more than
             * its window ahead of reality. Deferred through the reply worker
             * like every other notification. */
            if (now_written - acked >= OTA_ACK_SPAN || now_written >= total) {
                char ack[32];
                int n = snprintf(ack, sizeof(ack), "{\"ota\":%d}", (int)now_written);
                if (n > 0 && n < (int)sizeof(ack)) {
                    notify_json(conn_handle, chr_cfg_handle, ack);
                }
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                s_ota.acked_size = now_written;
                xSemaphoreGive(s_ota_mutex);
            }
            return 0; /* no response for WRITE_NO_RSP */
        }

        return BLE_ATT_ERR_ATTR_NOT_FOUND;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* ------------------------------------------------------------------ */
/* GATT registration callback - capture assigned value handles        */
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
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
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
    struct ble_hs_adv_fields rsp_fields = {0};
    /* BLE name = bare device name (gyroX is implied); AP SSID = gyroX-<dev> */
    const char *name = s_cfg.device_name[0] ? s_cfg.device_name : WEB_AP_SSID;
    size_t name_len = strlen(name);
    if (name_len > BLE_HS_ADV_MAX_FIELD_SZ) name_len = BLE_HS_ADV_MAX_FIELD_SZ;

    /* Main packet: flags + 128-bit service UUID (Web Bluetooth filters on
     * this). The device name goes in the SCAN RESPONSE instead - with the
     * UUID in a 31-byte legacy payload, only ~8 name characters would fit.
     * Android merges adv + scan-response, so the picker shows the name and
     * the UUID filter still matches. */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN;
    adv_fields.uuids128 = &svc_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&adv_fields);

    /* Scan response: complete device name */
    rsp_fields.name = (const uint8_t *)name;
    rsp_fields.name_len = name_len;
    rsp_fields.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

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
        /* Abort an in-flight OTA: a stale handle would block the next
         * esp_ota_begin until reboot. Mutex is guarded - it is created in
         * ble_start(), but this path must never assume anything. */
        if (s_ota_mutex != NULL) xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
        if (s_ota.in_progress) {
            esp_ota_abort(s_ota.ota);
            memset(&s_ota, 0, sizeof(s_ota));
            ESP_LOGW(TAG, "OTA aborted - client disconnected mid-transfer");
        }
        if (s_ota_mutex != NULL) xSemaphoreGive(s_ota_mutex);
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
    ble_svc_gap_device_name_set(s_cfg.device_name[0] ? s_cfg.device_name : WEB_AP_SSID);

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

    /* No passkey display available - use Just Works pairing */
    ble_hs_cfg.sm_io_cap = BLE_SM_IOACT_NONE;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    /* Capture the value handles NimBLE assigns during GATT registration -
     * without these, notify_json() would target handle 0 and every JSON
     * response would be lost. */
    ble_hs_cfg.gatts_register_cb = gatt_register_cb;

    /* Deferred reply worker (see notify_json) */
    if (s_reply_queue == NULL) {
        s_reply_queue = xQueueCreate(8, sizeof(ble_reply_t));
        if (s_reply_queue == NULL) {
            ESP_LOGE(TAG, "reply queue alloc failed");
            return ESP_ERR_NO_MEM;
        }
        xTaskCreate(reply_task, "ble_reply", 3072, NULL, 5, NULL);
    }

    /* OTA state mutex - created EAGERLY: the disconnect path takes it even
     * when no OTA was ever started, and a NULL queue handle asserts inside
     * FreeRTOS (crash + reboot on every disconnect). */
    if (s_ota_mutex == NULL) {
        s_ota_mutex = xSemaphoreCreateMutex();
    }

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


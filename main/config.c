#include "config.h"

#include <string.h>
#include <inttypes.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <cJSON.h>

static const char *TAG = "CONFIG";

drift_config_t s_cfg = {0};

static bool s_nvs_ready = false;

esp_err_t config_load(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase, reformatting");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    s_nvs_ready = true;

    nvs_handle_t handle;
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    config_apply_defaults();

    bool dirty = false;

    int32_t v;
    size_t sz;

    if (nvs_get_i32(handle, CFG_VERSION, &v) != ESP_OK) { v = DEF_CFG_VERSION; dirty = true; }
    s_cfg.version = v;
    if (nvs_get_i32(handle, CFG_GYRO_GAIN, &v) != ESP_OK) { v = DEF_GYRO_GAIN; dirty = true; }
    s_cfg.gyro_gain = v;
    if (nvs_get_i32(handle, CFG_GYRO_MODE, &v) != ESP_OK) { v = DEF_GYRO_MODE; dirty = true; }
    s_cfg.gyro_mode = v;
    if (nvs_get_i32(handle, CFG_GYRO_REVERSE, &v) != ESP_OK) { v = DEF_GYRO_REVERSE; dirty = true; }
    s_cfg.gyro_reverse = v;
    if (nvs_get_i32(handle, CFG_STEER_REVERSE, &v) != ESP_OK) { v = DEF_STEER_REVERSE; dirty = true; }
    s_cfg.steer_reverse = v;
    if (nvs_get_i32(handle, CFG_STEER_DEADBAND, &v) != ESP_OK) { v = DEF_STEER_DEADBAND; dirty = true; }
    s_cfg.steer_deadband = v;
    if (nvs_get_i32(handle, CFG_STEER_TRIM, &v) != ESP_OK) { v = DEF_STEER_TRIM; dirty = true; }
    s_cfg.steer_trim = v;
    if (nvs_get_i32(handle, CFG_SERVO_MIN_US, &v) != ESP_OK) { v = DEF_SERVO_MIN_US; dirty = true; }
    s_cfg.servo_min_us = v;
    if (nvs_get_i32(handle, CFG_SERVO_CENTER_US, &v) != ESP_OK) { v = DEF_SERVO_CENTER_US; dirty = true; }
    s_cfg.servo_center_us = v;
    if (nvs_get_i32(handle, CFG_SERVO_MAX_US, &v) != ESP_OK) { v = DEF_SERVO_MAX_US; dirty = true; }
    s_cfg.servo_max_us = v;
    if (nvs_get_i32(handle, CFG_HEADING_HOLD_P, &v) != ESP_OK) { v = DEF_HEADING_HOLD_P; dirty = true; }
    s_cfg.hold_p = v;

    sz = sizeof(s_cfg.gyro_z_offset);
    if (nvs_get_blob(handle, CFG_GYRO_Z_OFFSET, &s_cfg.gyro_z_offset, &sz) != ESP_OK) {
        s_cfg.gyro_z_offset = 0.0f;
        dirty = true;
    }

    sz = sizeof(s_cfg.web_ap_ssid);
    if (nvs_get_str(handle, CFG_WEB_AP_SSID, s_cfg.web_ap_ssid, &sz) != ESP_OK || sz == 0) {
        strncpy(s_cfg.web_ap_ssid, WEB_AP_SSID, sizeof(s_cfg.web_ap_ssid) - 1);
        s_cfg.web_ap_ssid[sizeof(s_cfg.web_ap_ssid) - 1] = '\0';
        dirty = true;
    }

    if (dirty) {
        ESP_LOGI(TAG, "Writing first-boot defaults to NVS");
        nvs_set_i32(handle, CFG_VERSION, s_cfg.version);
        nvs_set_i32(handle, CFG_GYRO_GAIN, s_cfg.gyro_gain);
        nvs_set_i32(handle, CFG_GYRO_MODE, s_cfg.gyro_mode);
        nvs_set_i32(handle, CFG_GYRO_REVERSE, s_cfg.gyro_reverse);
        nvs_set_i32(handle, CFG_STEER_REVERSE, s_cfg.steer_reverse);
        nvs_set_i32(handle, CFG_STEER_DEADBAND, s_cfg.steer_deadband);
        nvs_set_i32(handle, CFG_STEER_TRIM, s_cfg.steer_trim);
        nvs_set_i32(handle, CFG_SERVO_MIN_US, s_cfg.servo_min_us);
        nvs_set_i32(handle, CFG_SERVO_CENTER_US, s_cfg.servo_center_us);
        nvs_set_i32(handle, CFG_SERVO_MAX_US, s_cfg.servo_max_us);
        nvs_set_i32(handle, CFG_HEADING_HOLD_P, s_cfg.hold_p);
        nvs_set_blob(handle, CFG_GYRO_Z_OFFSET, &s_cfg.gyro_z_offset, sizeof(s_cfg.gyro_z_offset));
        nvs_set_str(handle, CFG_WEB_AP_SSID, s_cfg.web_ap_ssid);
        nvs_commit(handle);
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Config loaded: gain=%" PRId32 " mode=%" PRId32 " center=%" PRId32 " us",
             s_cfg.gyro_gain, s_cfg.gyro_mode, s_cfg.servo_center_us);
    return ESP_OK;
}

esp_err_t config_save(void)
{
    if (!s_nvs_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_i32(handle, CFG_GYRO_GAIN, s_cfg.gyro_gain);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_GYRO_MODE, s_cfg.gyro_mode);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_GYRO_REVERSE, s_cfg.gyro_reverse);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_STEER_REVERSE, s_cfg.steer_reverse);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_STEER_DEADBAND, s_cfg.steer_deadband);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_STEER_TRIM, s_cfg.steer_trim);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_SERVO_MIN_US, s_cfg.servo_min_us);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_SERVO_CENTER_US, s_cfg.servo_center_us);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_SERVO_MAX_US, s_cfg.servo_max_us);
    if (ret == ESP_OK) ret = nvs_set_i32(handle, CFG_HEADING_HOLD_P, s_cfg.hold_p);
    if (ret == ESP_OK) ret = nvs_set_blob(handle, CFG_GYRO_Z_OFFSET, &s_cfg.gyro_z_offset, sizeof(s_cfg.gyro_z_offset));
    if (ret == ESP_OK) ret = nvs_set_str(handle, CFG_WEB_AP_SSID, s_cfg.web_ap_ssid);

    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS");
    } else {
        ESP_LOGE(TAG, "Config save failed: %s", esp_err_to_name(ret));
    }
    return ret;
}


void config_apply_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.version = DEF_CFG_VERSION;
    s_cfg.gyro_gain = DEF_GYRO_GAIN;
    s_cfg.gyro_mode = DEF_GYRO_MODE;
    s_cfg.gyro_reverse = DEF_GYRO_REVERSE;
    s_cfg.steer_reverse = DEF_STEER_REVERSE;
    s_cfg.steer_deadband = DEF_STEER_DEADBAND;
    s_cfg.steer_trim = DEF_STEER_TRIM;
    s_cfg.servo_min_us = DEF_SERVO_MIN_US;
    s_cfg.servo_center_us = DEF_SERVO_CENTER_US;
    s_cfg.servo_max_us = DEF_SERVO_MAX_US;
    s_cfg.gyro_z_offset = 0.0f;
    s_cfg.hold_p = DEF_HEADING_HOLD_P;
    strncpy(s_cfg.web_ap_ssid, WEB_AP_SSID, sizeof(s_cfg.web_ap_ssid) - 1);
    s_cfg.web_ap_ssid[sizeof(s_cfg.web_ap_ssid) - 1] = '\0';
}

esp_err_t config_set_from_json(cJSON *root)
{
    if (!root || !cJSON_IsObject(root)) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *item;
    bool changed = false;

#define UPDATE_I32(key, field, minv, maxv)                                              \
    do {                                                                                \
        item = cJSON_GetObjectItem(root, key);                                          \
        if (item && cJSON_IsNumber(item)) {                                             \
            int32_t val = (int32_t)item->valueint;                                      \
            if (val < (minv)) val = (minv);                                             \
            if (val > (maxv)) val = (maxv);                                             \
            if (s_cfg.field != val) { s_cfg.field = val; changed = true; }              \
        }                                                                               \
    } while (0)

    UPDATE_I32(CFG_GYRO_GAIN, gyro_gain, 0, 500);
    UPDATE_I32(CFG_GYRO_MODE, gyro_mode, 0, 1);
    UPDATE_I32(CFG_GYRO_REVERSE, gyro_reverse, 0, 1);
    UPDATE_I32(CFG_STEER_REVERSE, steer_reverse, 0, 1);
    UPDATE_I32(CFG_STEER_DEADBAND, steer_deadband, 0, 50);
    UPDATE_I32(CFG_STEER_TRIM, steer_trim, -100, 100);
    UPDATE_I32(CFG_SERVO_MIN_US, servo_min_us, 100, 4000);
    UPDATE_I32(CFG_SERVO_CENTER_US, servo_center_us, 100, 4000);
    UPDATE_I32(CFG_SERVO_MAX_US, servo_max_us, 100, 4000);
    UPDATE_I32(CFG_HEADING_HOLD_P, hold_p, 0, 500);

#undef UPDATE_I32

    item = cJSON_GetObjectItem(root, CFG_GYRO_Z_OFFSET);
    if (item && cJSON_IsNumber(item)) {
        float val = (float)item->valuedouble;
        if (s_cfg.gyro_z_offset != val) { s_cfg.gyro_z_offset = val; changed = true; }
    }

    item = cJSON_GetObjectItem(root, CFG_WEB_AP_SSID);
    if (item && cJSON_IsString(item)) {
        if (strcmp(s_cfg.web_ap_ssid, item->valuestring) != 0) {
            strncpy(s_cfg.web_ap_ssid, item->valuestring, sizeof(s_cfg.web_ap_ssid) - 1);
            s_cfg.web_ap_ssid[sizeof(s_cfg.web_ap_ssid) - 1] = '\0';
            changed = true;
        }
    }

    if (changed) {
        return config_save();
    }
    return ESP_OK;
}

#include "web.h"
#include "config.h"
#include "gyro.h"
#include "mixing.h"
#include "pwm_in.h"
#include "servo.h"
#include "leds.h"
#include "ble.h"

#include <string.h>
#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <esp_app_desc.h>
#include <nvs_flash.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <arpa/inet.h>
#include <mdns.h>

static const char *TAG = "WEB";

/* AP SSID: gyroX-<device_name>, or plain gyroX when unnamed */
static void get_ap_ssid(char *out, size_t n)
{
    if (s_cfg.device_name[0]) {
        snprintf(out, n, "gyroX-%s", s_cfg.device_name);
    } else {
        strncpy(out, WEB_AP_SSID, n - 1);
        out[n - 1] = '\0';
    }
}

static httpd_handle_t s_server = NULL;
static TaskHandle_t s_dns_task = NULL;
static volatile bool s_dns_running = false;
static int s_dns_sock = -1;
static esp_ip4_addr_t s_ap_ip = { 0 };

#define DNS_PORT 53

/* ---- tiny captive-portal DNS: answer every A query with the AP IP ---- */
typedef struct {
    uint16_t id, flags, qd, an, ns, ar;
} __attribute__((packed)) dns_hdr_t;

typedef struct {
    uint16_t type, class;
} __attribute__((packed)) dns_q_t;

typedef struct {
    uint16_t ptr, type, class;
    uint32_t ttl;
    uint16_t len;
    uint32_t addr;
} __attribute__((packed)) dns_ans_t;

static void dns_task(void *pv)
{
    (void)pv;
    char rx[512], reply[512];
    struct sockaddr_in saddr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(DNS_PORT),
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket failed");
        s_dns_sock = -1;
        s_dns_running = false;
        vTaskDelete(NULL);
    }
    s_dns_sock = sock;
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (bind(sock, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed");
        close(sock);
        s_dns_sock = -1;
        s_dns_running = false;
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Captive DNS on port %d -> " IPSTR, DNS_PORT, IP2STR(&s_ap_ip));

    while (s_dns_running) {
        struct sockaddr_in caddr;
        socklen_t clen = sizeof(caddr);
        int len = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&caddr, &clen);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }
        if (len < (int)sizeof(dns_hdr_t) + sizeof(dns_q_t)) continue;

        dns_hdr_t *h = (dns_hdr_t *)reply;
        memcpy(reply, rx, len);
        h->flags = htons(ntohs(h->flags) | 0x8000); /* QR response */

        uint16_t qd = ntohs(h->qd);
        char *qptr = reply + sizeof(dns_hdr_t);
        char *aptr = reply + len;
        int ans = 0;
        const uint16_t name_ptr = htons(0xC000 | sizeof(dns_hdr_t));

        for (int i = 0; i < qd && aptr + sizeof(dns_ans_t) <= reply + sizeof(reply); i++) {
            /* skip question name */
            while (qptr < reply + len && *qptr) qptr++;
            qptr++; /* null byte */
            qptr += sizeof(dns_q_t); /* skip type+class */

            dns_ans_t *a = (dns_ans_t *)aptr;
            a->ptr = name_ptr;          /* pointer to query name */
            a->type = htons(0x0001);    /* A */
            a->class = htons(0x0001);   /* IN */
            a->ttl = htonl(60);
            a->len = htons(4);
            a->addr = s_ap_ip.addr;
            aptr += sizeof(dns_ans_t);
            ans++;
        }
        h->an = htons(ans);
        int reply_len = (int)(aptr - reply);
        sendto(sock, reply, reply_len, 0, (struct sockaddr *)&caddr, clen);
    }

    close(sock);
    s_dns_sock = -1;
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

static void start_dns(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif) {
        esp_netif_ip_info_t info;
        if (esp_netif_get_ip_info(netif, &info) == ESP_OK) {
            s_ap_ip.addr = info.ip.addr;
        }
    }
    if (s_ap_ip.addr == 0) {
        s_ap_ip.addr = lwip_htonl(0xC0A80401UL); /* 192.168.4.1 */
    }
    s_dns_running = true;
    xTaskCreate(dns_task, "dns", 3072, NULL, 5, &s_dns_task);
}

/* ---- mDNS: advertise the web UI as gyrox.local (+ gyroX.local alias) ---- */
/* Mirrors the mDNS setup used in the scalex project. gyroX runs AP-only with a
 * static AP IP, so the alias address is set once when the AP comes up. */
static void mdns_update_alias_address(void)
{
    esp_netif_ip_info_t info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif == NULL || esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == IPADDR_ANY) {
        info.ip.addr = s_ap_ip.addr; /* fall back to the static AP address */
    }

    mdns_ip_addr_t addr = { 0 };
    addr.addr.type = ESP_IPADDR_TYPE_V4;
    addr.addr.u_addr.ip4 = info.ip;
    addr.next = NULL;

    esp_err_t err = mdns_delegate_hostname_set_address(MDNS_HOSTNAME_ALIAS, &addr);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "mDNS alias %s.local -> " IPSTR, MDNS_HOSTNAME_ALIAS, IP2STR(&info.ip));
    } else {
        ESP_LOGW(TAG, "Failed to set mDNS alias %s.local: %s", MDNS_HOSTNAME_ALIAS, esp_err_to_name(err));
    }
}

static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) {
        ESP_LOGE(TAG, "mDNS init failed");
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE_NAME);
    mdns_service_add(MDNS_INSTANCE_NAME, "_http", "_tcp", 80, NULL, 0);

    if (mdns_delegate_hostname_add(MDNS_HOSTNAME_ALIAS, NULL) == ESP_OK) {
        mdns_update_alias_address();
        ESP_LOGI(TAG, "mDNS ready at %s.local and %s.local", MDNS_HOSTNAME, MDNS_HOSTNAME_ALIAS);
    } else {
        ESP_LOGW(TAG, "mDNS alias %s.local unavailable: %s.local only",
                 MDNS_HOSTNAME_ALIAS, MDNS_HOSTNAME);
        ESP_LOGI(TAG, "mDNS ready at %s.local", MDNS_HOSTNAME);
    }
}

static esp_err_t wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_log_level_set("wifi", ESP_LOG_WARN);

    esp_netif_ip_info_t info = {0};
    inet_pton(AF_INET, WEB_AP_IP, &info.ip);
    inet_pton(AF_INET, WEB_AP_GW, &info.gw);
    inet_pton(AF_INET, WEB_AP_NETMASK, &info.netmask);
    esp_netif_set_ip_info(ap_netif, &info);

    /* AP SSID: gyroX-<device_name>, or plain gyroX when unnamed */
    char ssid[33];
    get_ap_ssid(ssid, sizeof(ssid));

    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, ssid, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len = strlen(ssid);
    ap.ap.channel = WEB_AP_CHANNEL;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 4;
    if (strlen(WEB_AP_PASSWORD) > 0) {
        ap.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        strncpy((char *)ap.ap.password, WEB_AP_PASSWORD, sizeof(ap.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_netif_dhcps_start(ap_netif);

    start_dns();
    mdns_start();
    ESP_LOGI(TAG, "AP '%s' up at " IPSTR, ssid, IP2STR(&s_ap_ip));
    return ESP_OK;
}

/* ---------- embedded web UI ---------- */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[]   asm("_binary_index_html_end");
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, len);
}

static esp_err_t style_get_handler(httpd_req_t *req)
{
    extern const char style_css_start[] asm("_binary_style_css_start");
    extern const char style_css_end[]   asm("_binary_style_css_end");
    size_t len = style_css_end - style_css_start;
    httpd_resp_set_type(req, "text/css");
    return httpd_resp_send(req, style_css_start, len);
}

/* ---------- JSON helpers ---------- */
static esp_err_t json_reply(httpd_req_t *req, cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return err;
}

static esp_err_t json_ok(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    return json_reply(req, root);
}

static esp_err_t json_err(httpd_req_t *req, const char *msg)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "error", msg);
    httpd_resp_set_status(req, "500");
    return json_reply(req, root);
}

/* ---------- captive portal redirects ---------- */
static esp_err_t captive_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

/* ---------- /api/gyro: runtime gyro on/off ----------
 * Deliberately NOT persisted: the gyro always arms at boot; this only flips
 * the live mixer state (same path as the boot-button tap, tweak included). */
static esp_err_t gyro_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 128) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid payload length");
        return ESP_FAIL;
    }
    char body[129];
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read payload");
        return ESP_FAIL;
    }
    body[received] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *on_i = cJSON_GetObjectItem(root, "on");
    bool on = cJSON_IsNumber(on_i) ? (on_i->valueint != 0) : cJSON_IsTrue(on_i);
    cJSON_Delete(root);

    mixing_set_gyro(on);
    return json_ok(req);
}

/* ---------- /api/state ---------- */
static esp_err_t state_get_handler(httpd_req_t *req)
{
    const mixing_state_t *m = mixing_get_state();
    cJSON *r = cJSON_CreateObject();
    cJSON_AddNumberToObject(r, "rx", m->rx);
    cJSON_AddNumberToObject(r, "rate_z", m->rate_z);
    cJSON_AddNumberToObject(r, "heading", m->heading);
    cJSON_AddNumberToObject(r, "output", m->output);
    cJSON_AddBoolToObject(r, "rx_valid", m->rx_valid);
    cJSON_AddBoolToObject(r, "gyro_ready", m->gyro_ready);
    cJSON_AddBoolToObject(r, "gyro_on", m->gyro_on);
    cJSON_AddNumberToObject(r, "gyro_mode", s_cfg.gyro_mode);
    cJSON_AddNumberToObject(r, "gyro_gain", s_cfg.gyro_gain);
    cJSON_AddNumberToObject(r, "gyro_reverse", s_cfg.gyro_reverse);
    cJSON_AddNumberToObject(r, "steer_reverse", s_cfg.steer_reverse);
    cJSON_AddNumberToObject(r, "steer_deadband", s_cfg.steer_deadband);
    cJSON_AddNumberToObject(r, "steer_trim", s_cfg.steer_trim);
    cJSON_AddNumberToObject(r, "servo_min_us", s_cfg.servo_min_us);
    cJSON_AddNumberToObject(r, "servo_center_us", s_cfg.servo_center_us);
    cJSON_AddNumberToObject(r, "servo_max_us", s_cfg.servo_max_us);
    cJSON_AddNumberToObject(r, "hold_p", s_cfg.hold_p);
    cJSON_AddBoolToObject(r, "ap", true);
    return json_reply(req, r);
}

/* ---------- /api/config ---------- */
static esp_err_t config_get_handler(httpd_req_t *req)
{
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
    return json_reply(req, r);
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid payload length");
        return ESP_FAIL;
    }
    char *body = malloc(req->content_len + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read payload");
        return ESP_FAIL;
    }
    body[received] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    esp_err_t err = config_set_from_json(root);
    cJSON_Delete(root);
    if (err != ESP_OK) {
        return json_err(req, "Failed to save config");
    }
    /* Apply settings that take effect live (no reboot needed) */
    gyro_apply_sensor_config();
    leds_apply();
    return json_ok(req);
}

/* ---------- /api/calibrate ---------- */
static esp_err_t calibrate_post_handler(httpd_req_t *req)
{
    esp_err_t err = gyro_calibrate(GYRO_CAL_SAMPLES);
    if (err != ESP_OK) {
        return json_err(req, "Calibration failed");
    }
    return json_ok(req);
}

/* ---------- /api/reset_heading ---------- */
static esp_err_t reset_heading_post_handler(httpd_req_t *req)
{
    mixing_reset_heading();
    return json_ok(req);
}

/* ---------- /api/version ---------- */
static esp_err_t version_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    const char *ver = desc ? desc->version : "unknown";
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "version", ver);
    return json_reply(req, r);
}

/* ---------- /api/ota (firmware upload) ---------- */
/* The browser POSTs the raw .bin as the request body; we stream it into the
 * inactive OTA slot chunk by chunk, validate, switch the boot partition and
 * reboot. The bootloader verifies the image at boot and falls back to the
 * other slot if it is invalid, so a failed upload can't brick the car. */
static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (update == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition in table");
        return ESP_FAIL;
    }
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty upload");
        return ESP_FAIL;
    }
    if ((size_t)req->content_len > update->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image larger than OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char buf[1024];
    int64_t remaining = (int64_t)req->content_len;
    int idle_retries = 0;
    while (remaining > 0) {
        int want = (remaining < (int64_t)sizeof(buf)) ? (int)remaining : (int)sizeof(buf);
        int received = httpd_req_recv(req, buf, want);
        if (received < 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT && idle_retries++ < 60) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Transfer interrupted");
            return ESP_FAIL;
        }
        idle_retries = 0;
        err = esp_ota_write(ota, buf, received);
        if (err != ESP_OK) {
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write failed");
            return ESP_FAIL;
        }
        remaining -= received;
    }

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid image (esp_ota_end failed)");
        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set boot partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA: %d bytes -> %s, rebooting", (int)req->content_len, update->label);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"Update written, rebooting\"}");

    /* Let the response leave the stack, then swap partitions */
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

typedef struct {
    const char *uri;
    httpd_method_t method;
    esp_err_t (*handler)(httpd_req_t *);
} uri_map_t;

static const uri_map_t s_uris[] = {
    { "/",                    HTTP_GET,    root_get_handler },
    { "/style.css",           HTTP_GET,    style_get_handler },
    { "/generate_204",        HTTP_GET,    captive_handler },
    { "/fwlink",              HTTP_GET,    captive_handler },
    { "/hotspot-detect.html", HTTP_GET,    captive_handler },
    { "/ncsi.txt",            HTTP_GET,    captive_handler },
    { "/connecttest.txt",     HTTP_GET,    captive_handler },
    { "/api/state",           HTTP_GET,    state_get_handler },
    { "/api/config",          HTTP_GET,    config_get_handler },
    { "/api/config",          HTTP_POST,   config_post_handler },
    { "/api/gyro",            HTTP_POST,   gyro_post_handler },
    { "/api/calibrate",       HTTP_POST,   calibrate_post_handler },
    { "/api/reset_heading",   HTTP_POST,   reset_heading_post_handler },
    { "/api/version",         HTTP_GET,    version_get_handler },
    { "/api/ota",             HTTP_POST,   ota_post_handler },
};

void web_start(void)
{
    /* BLE and WiFi don't run concurrently on this device: stop BLE before
     * bringing up the softAP. BLE restarts on the next power cycle. */
    ble_stop();

    wifi_ap_start();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 32;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        httpd_uri_t u = {
            .uri = s_uris[i].uri,
            .method = s_uris[i].method,
            .handler = s_uris[i].handler,
            .user_ctx = NULL,
        };
        httpd_register_uri_handler(s_server, &u);
    }

    char ssid[33];
    get_ap_ssid(ssid, sizeof(ssid));
    ESP_LOGI(TAG, "Web UI: http://" IPSTR "/ (AP '%s')", IP2STR(&s_ap_ip), ssid);
}


idf_component_register(SRCS "app_main.c" "config.c" "gyro.c" "pwm_in.c" "servo.c" "mixing.c" "web.c" "leds.c" "cjson.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_wifi esp_http_server esp_event esp_netif nvs_flash freertos esp_timer esp_app_format app_update
                    EMBED_TXTFILES web/index.html
                    EMBED_TXTFILES web/style.css)

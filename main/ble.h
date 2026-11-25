#ifndef GYROX_BLE_H
#define GYROX_BLE_H

#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the NimBLE host, register the gyroX GATT service, and begin
 * advertising. Safe to call at boot; call ble_stop() before starting the
 * softAP so BLE and WiFi don't run concurrently. */
esp_err_t ble_start(void);

/* Stop advertising and shut down the NimBLE host. Call before wifi_ap_start(). */
void ble_stop(void);

/* True while BLE is advertising and accepting connections. */
bool ble_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* GYROX_BLE_H */

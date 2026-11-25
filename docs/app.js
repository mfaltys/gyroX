/* gyroX Config — Web Bluetooth client (mobile-first)
 *
 * Talks to the gyroX BLE service:
 *   Service:  0000A100-0000-1000-8000-00805F9B34FB
 *   0xA101 DEVICE_INFO  READ
 *   0xA102 CONFIG       WRITE + NOTIFY   (JSON command/response)
 *   0xA103 OTA_CTRL     WRITE            (JSON begin/abort/complete)
 *   0xA104 OTA_DATA     WRITE_NO_RSP     (raw binary chunks)
 */

(function () {
  'use strict';

  /* ------------------------------------------------------------------ */
  /* Constants                                                          */
  /* ------------------------------------------------------------------ */

  const SVC_UUID   = '0000a100-0000-1000-8000-00805f9b34fb';
  const CHR_INFO   = '0000a101-0000-1000-8000-00805f9b34fb';
  const CHR_CFG    = '0000a102-0000-1000-8000-00805f9b34fb';
  const CHR_OTAC   = '0000a103-0000-1000-8000-00805f9b34fb';
  const CHR_OTAD   = '0000a104-0000-1000-8000-00805f9b34fb';

  const MTU_CHUNK  = 480;
  const CHUNK_DELAY = 10;

  /* ------------------------------------------------------------------ */
  /* DOM refs                                                           */
  /* ------------------------------------------------------------------ */

  const els = {
    connDot:      document.getElementById('conn-dot'),
    connStatus:   document.getElementById('conn-status'),
    btnConnect:   document.getElementById('btn-connect'),
    btnDisconnect: document.getElementById('btn-disconnect'),
    deviceInfo:   document.getElementById('device-info'),
    configSection: document.getElementById('config-section'),
    otaSection:   document.getElementById('ota-section'),
    actionsSection: document.getElementById('actions-section'),
    infoModel:    document.getElementById('info-model'),
    infoVersion:  document.getElementById('info-version'),
    log:          document.getElementById('log'),
    otaFile:      document.getElementById('f-ota-file'),
    otaUpload:    document.getElementById('btn-ota-upload'),
    otaProgress:  document.getElementById('ota-progress'),
    otaFill:      document.querySelector('.progress-fill'),
    otaPct:       document.getElementById('ota-pct'),
    otaMsg:       document.getElementById('ota-msg'),
    btnGetCfg:    document.getElementById('btn-get-config'),
    btnSaveCfg:   document.getElementById('btn-save-config'),
    btnCalibrate: document.getElementById('btn-calibrate'),
    btnResetHdg:  document.getElementById('btn-reset-heading'),
  };

  const CFG_FIELDS = [
    'gyro_gain','gyro_mode','gyro_reverse','steer_reverse',
    'steer_deadband','steer_trim','servo_min_us','servo_center_us',
    'servo_max_us','hold_p','profile','leds_on','leds_bright',
    'leds_color','gyro_dlpf','gyro_fs','web_ap_ssid'
  ];

  /* ------------------------------------------------------------------ */
  /* BLE state                                                          */
  /* ------------------------------------------------------------------ */

  let device = null;
  let server = null;
  let chrInfo = null;
  let chrCfg  = null;
  let chrOtac = null;
  let chrOtad = null;
  let notifyResolve = null;
  let notifyBuffer = '';
  let isConnected = false;
  let otaInProgress = false;

  /* ------------------------------------------------------------------ */
  /* Logging                                                            */
  /* ------------------------------------------------------------------ */

  function log(msg, type = 'info') {
    const div = document.createElement('div');
    div.className = 'entry';
    const ts = new Date().toLocaleTimeString([], { hour12: false });
    div.innerHTML = `<span class="ts">[${ts}]</span><span class="${type}">${escapeHtml(msg)}</span>`;
    els.log.prepend(div);
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;',
      '"': '&quot;', "'": '&#39;'
    })[c]);
  }

  /* ------------------------------------------------------------------ */
  /* UI helpers                                                         */
  /* ------------------------------------------------------------------ */

  function setConnected(connected) {
    isConnected = connected;
    els.connDot.classList.toggle('connected', connected);
    els.connStatus.textContent = connected ? 'Connected' : 'Not connected';
    els.btnConnect.style.display = connected ? 'none' : 'inline-block';
    els.btnDisconnect.style.display = connected ? 'inline-block' : 'none';
    els.deviceInfo.style.display = connected ? 'block' : 'none';
    els.configSection.style.display = connected ? 'block' : 'none';
    els.otaSection.style.display = connected ? 'block' : 'none';
    els.actionsSection.style.display = connected ? 'block' : 'none';
    els.otaUpload.disabled = !connected;
  }
  /* ------------------------------------------------------------------ */
  /* BLE read/write helpers                                             */
  /* ------------------------------------------------------------------ */

  async function writeCfg(obj) {
    const str = JSON.stringify(obj);
    const enc = new TextEncoder().encode(str);
    await chrCfg.writeValue(enc);
  }

  /* Send a JSON command on the CONFIG characteristic and wait for the
   * device's notification response. The promise is registered BEFORE the
   * write so we can't miss a fast response. */
  async function sendCommand(cmd) {
    return new Promise((resolve, reject) => {
      notifyResolve = (result) => {
        try { resolve(JSON.parse(result)); }
        catch (e) { reject(new Error('Bad JSON from device')); }
      };
      chrCfg.writeValue(new TextEncoder().encode(JSON.stringify(cmd)))
        .catch((e) => { notifyResolve = null; reject(e); });
      setTimeout(() => {
        if (notifyResolve) {
          notifyResolve = null;
          reject(new Error('Device did not respond (timeout)'));
        }
      }, 5000);
    });
  }

  async function readInfo() {
    const data = await chrInfo.readValue();
    const dec = new TextDecoder().decode(data.buffer);
    return JSON.parse(dec);
  }

  function onChrCfgValue(value) {
    const dec = new TextDecoder().decode(value.buffer);
    notifyBuffer += dec;

    try {
      const obj = JSON.parse(notifyBuffer);
      notifyBuffer = '';
      if (notifyResolve) {
        const resolve = notifyResolve;
        notifyResolve = null;
        resolve(JSON.stringify(obj));
      }
    } catch (e) {
      // incomplete JSON, wait for more
    }
  }

  /* ------------------------------------------------------------------ */
  /* Connection                                                         */
  /* ------------------------------------------------------------------ */

  async function connect() {
    if (!navigator.bluetooth) {
      log('Web Bluetooth not supported in this browser', 'err');
      return;
    }

    try {
      log('Requesting device...');
      device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [SVC_UUID] }],
        optionalServices: [SVC_UUID]
      });

      device.addEventListener('gattserverdisconnected', onDisconnected);

      log('Connecting to GATT server...');
      server = await device.gatt.connect();

      log('Discovering service...');
      const svc = await server.getPrimaryService(SVC_UUID);

      log('Discovering characteristics...');
      chrInfo = await svc.getCharacteristic(CHR_INFO);
      chrCfg  = await svc.getCharacteristic(CHR_CFG);
      chrOtac = await svc.getCharacteristic(CHR_OTAC);
      chrOtad = await svc.getCharacteristic(CHR_OTAD);

      /* Start notifications BEFORE any commands so responses are never missed */
      chrCfg.addEventListener('characteristicvaluechanged',
        (ev) => onChrCfgValue(ev.target.value));
      await chrCfg.startNotifications();

      log('Connected! Reading device info...');
      setConnected(true);

      const info = await readInfo();
      els.infoModel.textContent = info.model || '—';
      els.infoVersion.textContent = info.version || '—';
      log(`Device info: ${info.model} v${info.version}`, 'ok');

      await loadConfig();

    } catch (err) {
      log(`Connection failed: ${err.message}`, 'err');
      setConnected(false);
    }
  }

  function onDisconnected() {
    log('Device disconnected', 'err');
    setConnected(false);
    server = null;
    device = null;
  }

  async function disconnect() {
    if (device && device.gatt.connected) {
      device.gatt.disconnect();
    }
    setConnected(false);
  }

  /* ------------------------------------------------------------------ */
  /* Config load / save                                                 */
  /* ------------------------------------------------------------------ */

  async function loadConfig() {
    try {
      log('Reading config...');
      const resp = await sendCommand({ cmd: 'get_config' });
      if (resp.ok) {
        populateConfig(resp);
        log('Config loaded', 'ok');
      } else {
        log(`Read failed: ${resp.error || 'unknown'}`, 'err');
      }
    } catch (err) {
      log(`Read failed: ${err.message}`, 'err');
    }
  }

  function populateConfig(cfg) {
    CFG_FIELDS.forEach(key => {
      const input = document.getElementById('f-' + key);
      if (!input) return;
      const val = cfg[key];
      if (val === undefined) return;
      if (input.tagName === 'SELECT' || input.tagName === 'INPUT') {
        input.value = String(val);
      }
    });
    if (cfg.leds_color !== undefined) {
      const c = document.getElementById('f-leds_color');
      c.value = '#' + String(cfg.leds_color).padStart(6, '0');
    }
  }

  async function saveConfig() {
    try {
      const obj = { cmd: 'set_config' };
      CFG_FIELDS.forEach(key => {
        const input = document.getElementById('f-' + key);
        if (!input) return;
        let val;
        if (input.type === 'color') {
          val = parseInt(input.value.replace('#', ''), 16);
        } else {
          val = input.value === '' ? undefined : parseFloat(input.value);
        }
        if (val !== undefined) obj[key] = val;
      });

      log('Saving config...');
      const resp = await sendCommand(obj);
      if (resp.ok) {
        log('Config saved', 'ok');
      } else {
        log(`Save failed: ${resp.error || 'unknown'}`, 'err');
      }
    } catch (err) {
      log(`Save failed: ${err.message}`, 'err');
    }
  }

  /* ------------------------------------------------------------------ */
  /* Actions                                                            */
  /* ------------------------------------------------------------------ */

  async function calibrate() {
    if (!confirm('Keep the car still. Continue?')) return;
    try {
      log('Calibrating gyro...');
      const resp = await sendCommand({ cmd: 'calibrate' });
      if (resp.ok) log('Calibration complete', 'ok');
      else log(`Calibrate failed: ${resp.error}`, 'err');
    } catch (err) {
      log(`Calibrate failed: ${err.message}`, 'err');
    }
  }

  async function resetHeading() {
    try {
      log('Resetting heading...');
      const resp = await sendCommand({ cmd: 'reset_heading' });
      if (resp.ok) log('Heading reset', 'ok');
      else log(`Reset failed: ${resp.error}`, 'err');
    } catch (err) {
      log(`Reset failed: ${err.message}`, 'err');
    }
  }

  /* ------------------------------------------------------------------ */
  /* OTA                                                                */
  /* ------------------------------------------------------------------ */

  els.otaFile.addEventListener('change', () => {
    els.otaUpload.disabled = !(els.otaFile.files && els.otaFile.files.length);
  });

  async function otaUpload() {
    const file = els.otaFile.files[0];
    if (!file) return;
    if (!confirm(`Flash ${file.name} (${(file.size / 1024).toFixed(1)} KB)?`)) return;

    otaInProgress = true;
    els.otaProgress.style.display = 'block';
    els.otaUpload.disabled = true;
    els.otaFile.disabled = true;

    try {
      const ab = await new Promise((resolve, reject) => {
        const r = new FileReader();
        r.onload = () => resolve(r.result);
        r.onerror = () => reject(r.error);
        r.readAsArrayBuffer(file);
      });

      const bytes = new Uint8Array(ab);
      log(`OTA begin: ${bytes.length} bytes`);

      await writeCtrl({ cmd: 'begin', size: bytes.length });

      let offset = 0;
      while (offset < bytes.length) {
        const end = Math.min(offset + MTU_CHUNK, bytes.length);
        await chrOtad.writeValue(bytes.slice(offset, end));
        offset = end;

        const pct = Math.round((offset / bytes.length) * 100);
        els.otaFill.style.width = pct + '%';
        els.otaPct.textContent = pct + '%';
        els.otaMsg.textContent = `Flashing ${offset}/${bytes.length} bytes...`;

        if (offset < bytes.length) {
          await sleep(CHUNK_DELAY);
        }
      }

      log(`OTA stream complete (${bytes.length} bytes)`);
      await writeCtrl({ cmd: 'complete' });

      log('Waiting for device to acknowledge...');
      await sleep(1500);
      log('OTA complete — device should reboot', 'ok');

    } catch (err) {
      log(`OTA failed: ${err.message}`, 'err');
      try { await writeCtrl({ cmd: 'abort' }); } catch (_) {}
    } finally {
      otaInProgress = false;
      els.otaUpload.disabled = false;
      els.otaFile.disabled = false;
    }
  }

  function writeCtrl(obj) {
    const enc = new TextEncoder().encode(JSON.stringify(obj));
    return chrOtac.writeValue(enc);
  }

  function sleep(ms) {
    return new Promise(r => setTimeout(r, ms));
  }

  /* ------------------------------------------------------------------ */
  /* Event listeners                                                    */
  /* ------------------------------------------------------------------ */

  els.btnConnect.addEventListener('click', connect);
  els.btnDisconnect.addEventListener('click', disconnect);
  els.btnGetCfg.addEventListener('click', loadConfig);
  els.btnSaveCfg.addEventListener('click', saveConfig);
  els.btnCalibrate.addEventListener('click', calibrate);
  els.btnResetHdg.addEventListener('click', resetHeading);
  els.otaUpload.addEventListener('click', otaUpload);

  /* ------------------------------------------------------------------ */
  /* Init                                                               */
  /* ------------------------------------------------------------------ */

  log('gyroX Config ready. Tap Connect.');
  if (!navigator.bluetooth) {
    log('Web Bluetooth not available — use Chrome/Edge on Android', 'err');
  }

})();


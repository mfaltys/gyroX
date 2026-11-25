/* gyroX Config — Web Bluetooth client, styled like the AP web UI.
 *
 * BLE service: 0000A100-0000-1000-8000-00805F9B34FB
 *   0xA101 DEVICE_INFO  READ
 *   0xA102 CONFIG       WRITE + NOTIFY  (JSON command/response)
 *   0xA103 OTA_CTRL     WRITE           (JSON begin/abort/complete)
 *   0xA104 OTA_DATA     WRITE_NO_RSP    (raw binary chunks)
 */
'use strict';

const SVC_UUID = '0000a100-0000-1000-8000-00805f9b34fb';
const CHR_INFO = '0000a101-0000-1000-8000-00805f9b34fb';
const CHR_CFG  = '0000a102-0000-1000-8000-00805f9b34fb';
const CHR_OTAC = '0000a103-0000-1000-8000-00805f9b34fb';
const CHR_OTAD = '0000a104-0000-1000-8000-00805f9b34fb';

const MTU_CHUNK   = 480;  // OTA data bytes per WRITE (device MTU is 517)
const CHUNK_DELAY = 15;   // ms between OTA chunks (flow control)

function el(id) { return document.getElementById(id); }

/* ------------------------- BLE state ------------------------------ */
let device = null, server = null;
let chrInfo = null, chrCfg = null, chrOtac = null, chrOtad = null;
let notifyResolve = null, notifyBuf = '';
let connected = false;
let otaBusy = false;
let pollBusy = false;
let pollGen = 0;      // generation token — kills stale polling loops
let connGen = 0;      // generation token — cancels superseded connect attempts
let stallCount = 0;   // consecutive poll failures → auto-reconnect

/* ---------------------------- Log --------------------------------- */
function esc(s) {
  return String(s).replace(/[&<>"]/g, c => ({ '&': '&', '<': '<', '>': '>', '"': '"' }[c]));
}
function log(msg, type) {
  const d = document.createElement('div');
  d.className = 'entry';
  const ts = new Date().toLocaleTimeString([], { hour12: false });
  d.innerHTML = '<span class="ts">[' + ts + ']</span><span class="' + (type || 'info') + '">' + esc(msg) + '</span>';
  const box = el('log');
  box.prepend(d);
  while (box.children.length > 100) box.removeChild(box.lastChild);
}

function setStatus(text, cls) {
  const s = el('status');
  s.textContent = text;
  s.className = 'msg' + (cls ? ' ' + cls : '');
}

/* -------- Command transport (JSON over CONFIG characteristic) ----- */
/* Promise registered BEFORE the write so a fast response is never
 * missed. Responses arrive as notifications on 0xA102. */
function sendCommand(cmd, timeoutMs, noRsp) {
  if (!connected) return Promise.reject(new Error('Not connected'));
  const tmo = timeoutMs || 5000;
  return new Promise((resolve, reject) => {
    notifyBuf = '';   // start clean — a lost fragment must not poison this response
    notifyResolve = (text) => {
      try { resolve(JSON.parse(text)); }
      catch (e) { reject(new Error('Bad JSON from device')); }
    };
    const payload = new TextEncoder().encode(JSON.stringify(cmd));
    /* Polls use write-without-response: no ATT write response to wait on, so
     * a wedge in the write-response path can't stall the telemetry loop. */
    const p = noRsp ? chrCfg.writeValueWithoutResponse(payload)
                    : chrCfg.writeValue(payload);
    p.catch(e => { notifyResolve = null; reject(e); });
    setTimeout(() => {
      if (notifyResolve) { notifyResolve = null; reject(new Error('Device did not respond (timeout)')); }
    }, tmo);
  });
}

function onChrCfgValue(value) {
  notifyBuf += new TextDecoder().decode(value.buffer);
  /* Responses always start with '{'. If not, a fragment was lost — drop the
   * buffer so the next command's response starts clean instead of every
   * future parse failing on concatenated garbage. */
  if (notifyBuf.charCodeAt(0) !== 0x7b) { notifyBuf = ''; return; }
  try {
    const obj = JSON.parse(notifyBuf);
    notifyBuf = '';
    if (notifyResolve) { const r = notifyResolve; notifyResolve = null; r(JSON.stringify(obj)); }
  } catch (e) {
    if (notifyBuf.length > 600) notifyBuf = '';  // runaway-buffer guard
  }
}

/* ---------------------- Connect / disconnect ---------------------- */
function setConnectedUI(on) {
  connected = on;
  el('btnConn').textContent = on ? 'Disconnect' : 'Connect';
  el('btnConn').classList.toggle('disconnect', on);
  if (!on) {
    el('ver').textContent = 'BLE';
    el('ver').classList.remove('connected');
    setStatus('Not connected — tap Connect and pick gyroX.');
    ['rx', 'rate', 'heading', 'output'].forEach(id => { el(id).textContent = '—'; });
  }
  el('btnSave').disabled = !on;
  el('btnCal').disabled = !on;
  el('btnLoad').disabled = !on;
  el('btnOta').disabled = !on || !el('otaFile').files.length;
}

function cfgNotifyHandler(ev) { onChrCfgValue(ev.target.value); }

async function connect() {
  if (!navigator.bluetooth) { log('Web Bluetooth not available — use Chrome/Edge on Android', 'err'); return; }
  if (connected) { disconnect(); return; }
  try {
    log('Requesting device…');
    device = await navigator.bluetooth.requestDevice({ filters: [{ services: [SVC_UUID] }] });
    device.addEventListener('gattserverdisconnected', onDisconnected);
    await openConnection();
  } catch (e) {
    log('Connection failed: ' + e.message, 'err');
    if (device) try { device.gatt.disconnect(); } catch (_) {}
    device = null; server = null;
    setConnectedUI(false);
  }
}

/* GATT setup shared by first connect and stall auto-reconnect.
 * Retries up to 3 times: Android Chrome sporadically drops the link right
 * after connect() resolves (status-133-style failure) — the error surfaces
 * as "GATT Server is disconnected. Cannot retrieve services." on the very
 * first getPrimaryService(). A plain second attempt almost always lands. */
async function openConnection() {
  const dev = device;
  if (!dev) throw new Error('No device selected');
  const gen = ++connGen;

  let lastErr = null;
  for (let attempt = 1; attempt <= 3; attempt++) {
    if (gen !== connGen) return false;
    try {
      log(attempt === 1 ? 'Connecting GATT…' : 'Connect attempt ' + attempt + '…');
      server = await dev.gatt.connect();
      if (!server.connected) throw new Error('Link dropped during connect');
      const svc = await server.getPrimaryService(SVC_UUID);
      chrInfo = await svc.getCharacteristic(CHR_INFO);
      chrCfg  = await svc.getCharacteristic(CHR_CFG);
      chrOtac = await svc.getCharacteristic(CHR_OTAC);
      chrOtad = await svc.getCharacteristic(CHR_OTAD);
      chrCfg.addEventListener('characteristicvaluechanged', cfgNotifyHandler);
      await chrCfg.startNotifications();
      lastErr = null;
      break;
    } catch (e) {
      lastErr = e;
      if (gen !== connGen) return false;
      if (attempt < 3) {
        log('Attempt ' + attempt + ' failed: ' + e.message + ' — retrying…', 'err');
        await sleep(800);
      }
    }
  }
  if (gen !== connGen) return false;
  if (lastErr) throw lastErr;

  setConnectedUI(true);
  log('Connected', 'ok');
  try {
    const info = JSON.parse(new TextDecoder().decode((await chrInfo.readValue()).buffer));
    if (info.version) el('ver').textContent = 'fw: ' + info.version;
    log('Firmware ' + info.version, 'ok');
  } catch (e) { /* pill stays generic */ }
  stallCount = 0;
  await loadCfg();
  startPolling();
  return true;
}

function onDisconnected() {
  pollGen++;          // stop any polling loops
  pollBusy = false;
  if (!connected) return;   // dropped during setup — the retry loop recovers
  log('Device disconnected', 'err');
  if (otaBusy) { el('otaMsg').textContent = 'Connection lost — device rebooting.'; otaBusy = false; }
  device = null; server = null;
  setConnectedUI(false);
}

function disconnect() {
  connGen++;          // cancel any in-flight (re)connect attempts
  setConnectedUI(false);   // mark down first so the disconnect event is a no-op
  if (device && device.gatt.connected) device.gatt.disconnect();
  device = null; server = null;
  log('Disconnected');
}

/* ------------------ Config load / save / actions ------------------ */
function applyCfg(c) {
  el('mMode').value = c.gyro_mode;
  el('mGain').value = c.gyro_gain; el('vGain').textContent = (c.gyro_gain / 10).toFixed(1);
  el('mGyroRev').checked = c.gyro_reverse === 1;
  el('mSteerRev').checked = c.steer_reverse === 1;
  el('mDeadband').value = c.steer_deadband; el('vDeadband').textContent = c.steer_deadband;
  el('mTrim').value = c.steer_trim; el('vTrim').textContent = c.steer_trim;
  el('mSmin').value = c.servo_min_us;
  el('mScenter').value = c.servo_center_us;
  el('mSmax').value = c.servo_max_us;
  el('mGzoff').value = c.gyro_z_offset;
  el('mDlpf').value = c.gyro_dlpf;
  el('mFs').value = c.gyro_fs;
  el('mProfile').value = (c.profile === undefined ? 0 : c.profile);
  el('rowResetHead').style.display = (c.gyro_mode === 1) ? '' : 'none';
  el('mLedsOn').checked = c.leds_on === 1;
  const hexc = '#' + Number(c.leds_color || 0).toString(16).padStart(6, '0');
  el('mLedsColor').value = hexc;
  el('vLedsColor').textContent = hexc;
  el('mLedsBright').value = (c.leds_bright === undefined ? 100 : c.leds_bright);
  el('vBright').textContent = el('mLedsBright').value;
}

function collectCfg() {
  return {
    gyro_mode: Number(el('mMode').value),
    gyro_gain: Number(el('mGain').value),
    gyro_reverse: el('mGyroRev').checked ? 1 : 0,
    steer_reverse: el('mSteerRev').checked ? 1 : 0,
    steer_deadband: Number(el('mDeadband').value),
    steer_trim: Number(el('mTrim').value),
    servo_min_us: Number(el('mSmin').value),
    servo_center_us: Number(el('mScenter').value),
    servo_max_us: Number(el('mSmax').value),
    gyro_z_offset: Number(el('mGzoff').value),
    gyro_dlpf: Number(el('mDlpf').value),
    gyro_fs: Number(el('mFs').value),
    profile: Number(el('mProfile').value),
    leds_on: el('mLedsOn').checked ? 1 : 0,
    leds_color: parseInt(el('mLedsColor').value.slice(1), 16),
    leds_bright: Number(el('mLedsBright').value),
  };
}


async function saveCfg() {
  try {
    el('mMsg').textContent = 'Saving…';
    const r = await sendCommand(Object.assign({ cmd: 'set_config' }, collectCfg()));
    if (!r.ok) throw new Error(r.error || 'save failed');
    el('mMsg').textContent = 'Saved to NVS';
    setTimeout(() => { el('mMsg').textContent = ''; }, 1500);
    log('Config saved', 'ok');
  } catch (e) {
    el('mMsg').textContent = 'Save failed: ' + e.message;
    log('Save failed: ' + e.message, 'err');
  }
}

async function loadCfg() {
  try {
    const r = await sendCommand({ cmd: 'get_config' });
    if (!r.ok) throw new Error(r.error || 'read failed');
    applyCfg(r);
    log('Config loaded', 'ok');
  } catch (e) {
    log('Config read failed: ' + e.message, 'err');
  }
}

async function calibrate() {
  if (!confirm('Keep the car completely still, then press OK.')) return;
  try {
    el('mMsg').textContent = 'Calibrating gyro…';
    const r = await sendCommand({ cmd: 'calibrate' });
    if (!r.ok) throw new Error(r.error || 'failed');
    el('mMsg').textContent = 'Calibrated';
    loadCfg();
    setTimeout(() => { el('mMsg').textContent = ''; }, 1500);
    log('Gyro calibrated', 'ok');
  } catch (e) {
    el('mMsg').textContent = 'Calibration failed: ' + e.message;
    log('Calibration failed: ' + e.message, 'err');
  }
}

async function resetHead() {
  try {
    await sendCommand({ cmd: 'reset_heading' });
    el('mMsg').textContent = 'Heading-hold target reset';
    setTimeout(() => { el('mMsg').textContent = ''; }, 1000);
    log('Heading reset', 'ok');
  } catch (e) { log('Reset heading failed: ' + e.message, 'err'); }
}

/* slider value labels (same as AP page) */
[['mGain', 'vGain', true], ['mDeadband', 'vDeadband', false], ['mTrim', 'vTrim', false]].forEach(([id, vid, div10]) => {
  el(id).addEventListener('input', () => {
    let v = el(id).value;
    if (div10) v = (Number(v) / 10).toFixed(1);
    el(vid).textContent = v;
  });
});


/* profile presets (same as AP page) */
const PRESETS = {
  0: { name: 'General', gyro_mode: 0, gyro_gain: 100, steer_deadband: 8, steer_trim: 0, servo_min_us: 500, servo_center_us: 1500, servo_max_us: 2500 },
  1: { name: 'Drift',   gyro_mode: 0, gyro_gain: 200, steer_deadband: 4, steer_trim: 0, servo_min_us: 600, servo_center_us: 1500, servo_max_us: 2400 },
  2: { name: 'Bashing', gyro_mode: 0, gyro_gain: 120, steer_deadband: 7, steer_trim: 0, servo_min_us: 500, servo_center_us: 1500, servo_max_us: 2500 },
};
el('mProfile').addEventListener('change', () => {
  const p = Number(el('mProfile').value);
  if (p === 3) return;
  const pre = PRESETS[p];
  if (!pre) return;
  el('mMode').value = pre.gyro_mode;
  el('rowResetHead').style.display = pre.gyro_mode === 1 ? '' : 'none';
  el('mGain').value = pre.gyro_gain;
  el('vGain').textContent = (pre.gyro_gain / 10).toFixed(1);
  el('mDeadband').value = pre.steer_deadband;
  el('vDeadband').textContent = pre.steer_deadband;
  el('mTrim').value = pre.steer_trim;
  el('vTrim').textContent = pre.steer_trim;
  el('mSmin').value = pre.servo_min_us;
  el('mScenter').value = pre.servo_center_us;
  el('mSmax').value = pre.servo_max_us;
  el('mMsg').textContent = pre.name + ' preset loaded — press Save to apply';
});
const flipToCustom = () => { el('mProfile').value = 3; };
['mGain', 'mDeadband', 'mTrim', 'mSmin', 'mScenter', 'mSmax'].forEach(id => {
  el(id).addEventListener('input', flipToCustom);
});
el('mMode').addEventListener('change', () => {
  el('rowResetHead').style.display = el('mMode').value === '1' ? '' : 'none';
  flipToCustom();
});

/* collapsible sections (Steering / Servo / Log) */
document.querySelectorAll('.sep[data-collapse]').forEach(sep => {
  sep.addEventListener('click', () => {
    const open = sep.classList.toggle('open');
    el('collapse-' + sep.dataset.collapse).classList.toggle('open', open);
    sep.querySelector('.chev').textContent = open ? '▾' : '▸';
  });
});

/* tabs (same as AP page) */
const tabs = document.querySelectorAll('.tab');
const panels = document.querySelectorAll('.tabpanel');
tabs.forEach(t => {
  t.addEventListener('click', () => {
    tabs.forEach(x => { x.classList.remove('active'); x.setAttribute('aria-selected', 'false'); });
    panels.forEach(p => p.classList.remove('active'));
    t.classList.add('active');
    t.setAttribute('aria-selected', 'true');
    el('tab-' + t.dataset.tab).classList.add('active');
  });
});

/* LEDs apply live on change (same as AP page) */
el('mLedsOn').addEventListener('change', () => { if (connected) saveCfg(); });
el('mLedsColor').addEventListener('input', () => { el('vLedsColor').textContent = el('mLedsColor').value; });
el('mLedsColor').addEventListener('change', () => { if (connected) saveCfg(); });
el('mLedsBright').addEventListener('input', () => { el('vBright').textContent = el('mLedsBright').value; });
el('mLedsBright').addEventListener('change', () => { if (connected) saveCfg(); });


/* ------------------------ OTA over BLE ---------------------------- */
el('otaFile').addEventListener('change', () => {
  el('btnOta').disabled = !connected || !el('otaFile').files.length;
});

async function otaUpload() {
  const f = el('otaFile').files[0];
  if (!f) { el('otaMsg').textContent = 'Choose a .bin file first'; return; }
  if (!confirm('Flash ' + f.name + ' (' + (f.size / 1024).toFixed(1) + ' KB)? Keep the car parked.')) return;

  otaBusy = true;
  el('btnOta').disabled = true;
  el('otaBar').style.width = '0%';
  el('otaMsg').textContent = 'Starting… keep this tab open!';

  const ctrl = (o) => chrOtac.writeValue(new TextEncoder().encode(JSON.stringify(o)));
  try {
    const ab = await new Promise((res, rej) => {
      const r = new FileReader();
      r.onload = () => res(r.result);
      r.onerror = () => rej(r.error);
      r.readAsArrayBuffer(f);
    });
    const bytes = new Uint8Array(ab);
    log('OTA begin: ' + bytes.length + ' bytes');

    await ctrl({ cmd: 'begin', size: bytes.length });
    log('OTA accepted by device', 'ok');

    for (let off = 0; off < bytes.length; off += MTU_CHUNK) {
      const end = Math.min(off + MTU_CHUNK, bytes.length);
      await chrOtad.writeValue(bytes.slice(off, end));
      const pct = Math.round((end / bytes.length) * 100);
      el('otaBar').style.width = pct + '%';
      el('otaMsg').textContent = 'Flashing ' + pct + '% — do not power off!';
      if (end < bytes.length) await new Promise(r => setTimeout(r, CHUNK_DELAY));
    }
    log('OTA stream complete, finalizing…');

    await ctrl({ cmd: 'complete' });
    el('otaBar').style.width = '100%';
    el('otaMsg').textContent = 'Update written — rebooting into drive mode (hold boot 3 s to return here)';
    log('OTA complete — device rebooting', 'ok');
  } catch (e) {
    el('otaMsg').textContent = 'OTA failed: ' + e.message;
    log('OTA failed: ' + e.message, 'err');
    try { await ctrl({ cmd: 'abort' }); } catch (_) {}
  } finally {
    otaBusy = false;
    el('btnOta').disabled = !connected;
  }
}

/* ---------------------- Telemetry polling ------------------------- */
async function pollOnce() {
  const s = await sendCommand({ cmd: 'get_state' }, 1500, true);
  el('rx').textContent = s.rx;
  el('rate').textContent = s.rate_z;
  el('heading').textContent = Number(s.heading).toFixed(1);
  el('output').textContent = s.output;
  setStatus((s.rx_valid ? 'RX OK' : 'NO RX!') + '  ·  gyro ' + (s.gyro_ready ? 'OK' : '---'),
            s.rx_valid ? '' : 'err');
}

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

/* Single loop per connection "generation". The gen token guarantees stale
 * loops exit after a reconnect instead of multiplying chains. */
async function pollingLoop(gen) {
  while (connected && gen === pollGen) {
    if (!pollBusy) {
      pollBusy = true;
      try {
        await pollOnce();
        stallCount = 0;
      } catch (e) {
        stallCount++;
        if (stallCount === 1) log('Poll missed (' + e.message + ') — watching…', 'err');
        if (stallCount >= 5) {
          /* Link is wedged (e.g. silent drop) — GATT ops queue forever and
           * telemetry freezes. Drop and reconnect automatically. */
          stallCount = 0;
          pollBusy = false;
          log('Connection stalled — reconnecting…', 'err');
          const dev = device;
          if (dev && dev.gatt.connected) try { dev.gatt.disconnect(); } catch (_) {}
          setTimeout(() => {
            if (!dev) return;
            device = dev;
            openConnection().catch(() => {
              log('Reconnect failed — tap Connect to retry.', 'err');
              setConnectedUI(false);
            });
          }, 1000);
          return;
        }
      }
      pollBusy = false;
    }
    await sleep(200);
  }
}

function startPolling() {
  pollGen++;
  pollingLoop(pollGen);
}

/* --------------------------- Wiring ------------------------------- */
el('btnConn').addEventListener('click', connect);
el('btnSave').addEventListener('click', saveCfg);
el('btnLoad').addEventListener('click', loadCfg);
el('btnCal').addEventListener('click', calibrate);
el('btnResetHead').addEventListener('click', resetHead);
el('btnOta').addEventListener('click', otaUpload);

/* ---------------------------- Init -------------------------------- */
setConnectedUI(false);
log('Ready. Tap Connect and pick gyroX.');
if (!navigator.bluetooth) log('Web Bluetooth not available in this browser — use Chrome/Edge on Android.', 'err');


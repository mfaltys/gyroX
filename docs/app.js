/* gyroX Config - Web Bluetooth client, styled like the AP web UI.
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
/* Measured over BLE (write-with-response): 1,248,368 bytes in ~409 s.
 * Windowed write-without-response measured: 1,228,480 bytes in ~296 s
 * (~4.15 KB/s) - the rate is set by the device's flash pipeline
 * (~4 KB sector erases), which the flow-control window tracks exactly. */
const OTA_BYTES_PER_SEC = 4150;
/* Flow control: the device acks every ~4 KB actually written to flash.
 * Never more than OTA_WINDOW_BYTES of unacked data in flight, else the
 * backlog grows behind the device and Chrome's ~30 s GATT op timeout
 * kills the link mid-transfer. */
const OTA_WINDOW_BYTES = 16384;
const OTA_ACK_TIMEOUT  = 8000;   // ms to wait for a progress ack

function el(id) { return document.getElementById(id); }

/* ------------------------- BLE state ------------------------------ */
let device = null, server = null;
let chrInfo = null, chrCfg = null, chrOtac = null, chrOtad = null;
let notifyResolve = null, notifyBuf = '';
let connected = false;
let otaBusy = false;
let connGen = 0;      // generation token - cancels superseded connect attempts
let deviceVersion = null;   // firmware version reported by the device
let releases = [];          // GitHub releases, newest first
let bleVer = 1;             // device BLE protocol version (2 = acks + WNR OTA)
let otaSent = 0;            // bytes queued to the radio this OTA
let otaAcked = 0;           // bytes the device reports actually written

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
function sendCommand(cmd, timeoutMs) {
  if (!connected) return Promise.reject(new Error('Not connected'));
  const tmo = timeoutMs || 5000;
  return new Promise((resolve, reject) => {
    notifyBuf = '';   // start clean - a lost fragment must not poison this response
    notifyResolve = (text) => {
      try { resolve(JSON.parse(text)); }
      catch (e) { reject(new Error('Bad JSON from device')); }
    };
    /* ALL JSON writes use write-without-response: there is no ATT write
     * response to race or lose, so Chrome's single-slot GATT op queue can
     * never wedge on a zombie write. The device's notification is the one
     * completion signal; the timeout below is the backstop. */
    const p = chrCfg.writeValueWithoutResponse(new TextEncoder().encode(JSON.stringify(cmd)));
    p.catch(e => { notifyResolve = null; reject(e); });
    setTimeout(() => {
      if (notifyResolve) { notifyResolve = null; reject(new Error('Device did not respond (timeout)')); }
    }, tmo);
  });
}

function onChrCfgValue(value) {
  notifyBuf += new TextDecoder().decode(value.buffer);
  /* Responses always start with '{'. If not, a fragment was lost - drop the
   * buffer so the next command's response starts clean instead of every
   * future parse failing on concatenated garbage. */
  if (notifyBuf.charCodeAt(0) !== 0x7b) { notifyBuf = ''; return; }
  try {
    const obj = JSON.parse(notifyBuf);
    notifyBuf = '';
    /* OTA flow-control acks arrive between commands - route by shape and
     * update the counter unconditionally so no ack is ever dropped. */
    if (obj && typeof obj.ota === 'number') { otaAcked = obj.ota; return; }
    if (notifyResolve) { const r = notifyResolve; notifyResolve = null; r(JSON.stringify(obj)); }
  } catch (e) {
    if (notifyBuf.length > 600) notifyBuf = '';  // runaway-buffer guard
  }
}

/* ---------------------- Connect / disconnect ---------------------- */
function setConnectedUI(on) {
  connected = on;
  document.getElementById('shell').classList.toggle('off', !on);
  el('btnConn').textContent = on ? 'Disconnect' : 'Connect';
  el('btnConn').classList.toggle('disconnect', on);
  if (!on) {
    deviceVersion = null;
    bleVer = 1;
    el('ver').textContent = 'BLE';
    el('ver').classList.remove('connected');
    el('otaInstalled').textContent = '-';
    setStatus('Not connected - tap Connect and pick gyroX.');
  }
  el('btnSave').disabled = !on;
  el('btnCal').disabled = !on;
  el('btnLoad').disabled = !on;
  el('btnRollback').disabled = !on;
  updateOtaStatus();
}

function cfgNotifyHandler(ev) { onChrCfgValue(ev.target.value); }

async function connect() {
  if (!navigator.bluetooth) { log('Web Bluetooth not available - use Chrome/Edge on Android', 'err'); return; }
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
 * after connect() resolves (status-133-style failure) - the error surfaces
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
        log('Attempt ' + attempt + ' failed: ' + e.message + ' - retrying…', 'err');
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
    deviceVersion = info.version || null;
    bleVer = info.ble_ver || 1;
    if (deviceVersion) {
      el('ver').textContent = deviceVersion;
      el('otaInstalled').textContent = deviceVersion;
    }
    log('Firmware ' + deviceVersion + ' (ble protocol v' + bleVer + ')', 'ok');
  } catch (e) { /* pill stays generic */ }
  await loadCfg();
  /* One-shot health check - a snapshot, not a live feed (see refreshState). */
  try { await refreshState(); }
  catch (e) { log('State read failed: ' + e.message, 'err'); }
  /* OTA: populate the releases dropdown + rollback row (both non-fatal) */
  loadOtaVersions();
  getSlots();
  return true;
}

function onDisconnected() {
  if (!connected) return;   // dropped during setup - the retry loop recovers
  log('Device disconnected', 'err');
  if (otaBusy) { el('otaMsg').textContent = 'Connection lost - device rebooting.'; otaBusy = false; }
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
  el('mGzoff').textContent = Number(c.gyro_z_offset || 0).toFixed(3);
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
  el('mMsg').textContent = pre.name + ' preset loaded - press Save to apply';
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


/* ------------------ Releases / versioning -------------------------- */
/* Firmware binaries live on GitHub Releases (no manifest file): one API
 * call lists every release with tag, size, date and notes. Assets are
 * fetched via the API's octet-stream endpoint, which carries CORS headers
 * (the plain browser_download_url does NOT - never fetch that from a page). */

const REPO = 'mfaltys/gyroX';
/* Firmware binaries are served from the unixvoid-builds S3 bucket (the
 * .s3.amazonaws.com form implies us-east-1 - same bucket scalex uses).
 * GitHub release-asset downloads are NOT CORS-accessible from a browser,
 * so Releases act as the version index and S3 serves the bytes. */
const FIRMWARE_BASE_URL = 'https://unixvoid-builds.s3.amazonaws.com/gyrox';

function parseSemver(s) {
  const m = /v?(\d+)\.(\d+)\.(\d+)/.exec(s || '');
  return m ? [+m[1], +m[2], +m[3]] : null;
}
function cmpSemver(a, b) {
  for (let i = 0; i < 3; i++) { if (a[i] !== b[i]) return a[i] - b[i]; }
  return 0;
}

async function fetchReleases(force) {
  let cached = null;
  try { cached = JSON.parse(localStorage.getItem('gyroX_releases') || 'null'); } catch (e) { /* ignore */ }
  /* cache is only a fallback: force a fresh fetch at every connect so a
   * newly published release shows up right away (localStorage survives
   * hard refreshes, which is why a TTL alone is not enough) */
  if (!force && cached && Date.now() - cached.t < 60 * 1000) return cached.list;
  const res = await fetch('https://api.github.com/repos/' + REPO + '/releases?per_page=20',
                          { headers: { Accept: 'application/vnd.github+json' } });
  if (!res.ok) throw new Error('GitHub API ' + res.status);
  const raw = await res.json();
  const list = raw
    .filter(r => !r.draft && (r.assets || []).some(a => a.name.endsWith('.bin')))
    .sort((a, b) => {
      const A = parseSemver(a.tag_name), B = parseSemver(b.tag_name);
      if (A && B) return cmpSemver(B, A);   // newest semver first
      return new Date(b.published_at) - new Date(a.published_at);
    })
    .map(r => ({
      tag: r.tag_name,
      date: (r.published_at || '').slice(0, 10),
      size: r.assets.find(a => a.name.endsWith('.bin')).size,
      prerelease: !!r.prerelease,
      notes: (r.body || '').split('\n').filter(l => l.trim()).slice(0, 2).join(' · '),
    }));
  try { localStorage.setItem('gyroX_releases', JSON.stringify({ t: Date.now(), list })); } catch (e) { /* ignore */ }
  return list;
}

async function loadOtaVersions() {
  const sel = el('otaVersion');
  try {
    releases = await fetchReleases(true);   // fresh fetch at every connect
  } catch (e) {
    /* GitHub unreachable or rate-limited: fall back to the cached list so
     * the page still works offline. */
    try { releases = (JSON.parse(localStorage.getItem('gyroX_releases') || 'null') || {}).list || []; }
    catch (err) { releases = []; }
    if (releases.length) {
      log('GitHub unreachable - using cached release list', 'err');
    } else {
      sel.innerHTML = '<option>Release list unavailable</option>';
      log('Release list failed: ' + e.message, 'err');
      updateOtaStatus();
      return;
    }
  }
  if (!releases.length) {
    sel.innerHTML = '<option>No releases published yet</option>';
    updateOtaStatus();
    return;
  }
  sel.innerHTML = '';
  releases.forEach((r, i) => {
    const o = document.createElement('option');
    o.value = String(i);
    o.textContent = r.tag + (i === 0 ? '  (latest)' : '') + (r.prerelease ? '  (pre-release)' : '');
    sel.appendChild(o);
  });
  sel.value = '0';
  updateOtaStatus();
}

function updateOtaStatus() {
  const btn = el('btnOta');
  const box = el('otaBox');
  const latest = releases[0];
  el('otaLatest').textContent = latest ? latest.tag : '-';

  const localFile = el('otaFile').files[0];
  const sel = releases[Number(el('otaVersion').value || 0)];

  /* Force-flash button in the advanced section: works for any selected
   * version, including a re-flash of the installed one. */
  el('btnFlashVersion').disabled = !connected || !sel;

  if (!connected) {
    box.className = 'ota-box';
    el('otaState').textContent = 'Connect to check for updates.';
    btn.style.display = 'none';
    return;
  }
  if (!latest || !sel) {
    box.className = 'ota-box';
    el('otaState').textContent = 'No releases published yet.';
    btn.style.display = 'none';
    return;
  }

  const dev = parseSemver(deviceVersion);
  const rel = parseSemver(sel.tag);
  box.classList.remove('ok', 'update');
  let actionable = false;
  if (localFile) {
    btn.textContent = 'Flash local file: ' + localFile.name;
    actionable = true;
  } else if (dev && rel) {
    const c = cmpSemver(rel, dev);
    if (c > 0) {
      box.classList.add('update');
      el('otaState').textContent = 'Update available - ' + sel.tag;
      btn.textContent = 'Update to ' + sel.tag;
      actionable = true;
    } else if (c === 0) {
      box.classList.add('ok');
      el('otaState').textContent = deviceVersion === sel.tag ? 'Up to date' : 'Up to date (dev build of ' + sel.tag + ')';
      /* a force re-flash lives in the advanced section (Flash Version) */
      btn.textContent = 'Up to date';
    } else {
      box.classList.add('update');
      el('otaState').textContent = 'Selected version is older than installed';
      btn.textContent = 'Downgrade to ' + sel.tag;
      actionable = true;
    }
  } else {
    box.classList.add('update');
    el('otaState').textContent = 'Installed version unknown - update recommended';
    btn.textContent = 'Flash ' + sel.tag;
    actionable = true;
  }
  btn.style.display = actionable ? '' : 'none';
  btn.disabled = !connected || !actionable;
  el('otaMin').textContent = String(Math.max(1, Math.round(sel.size / OTA_BYTES_PER_SEC / 60)));
  el('otaDl').href = FIRMWARE_BASE_URL + '/' + sel.tag + '/app.bin';
}

async function downloadFirmware(rel) {
  const url = `${FIRMWARE_BASE_URL}/${rel.tag}/app.bin`;
  const res = await fetch(url);
  if (!res.ok) throw new Error('HTTP ' + res.status);
  return new Uint8Array(await res.arrayBuffer());
}

async function verifySha256(bytes, rel) {
  const res = await fetch(`${FIRMWARE_BASE_URL}/${rel.tag}/app.bin.sha256`);
  if (!res.ok) throw new Error('SHA fetch HTTP ' + res.status);
  const expected = (await res.text()).trim().split(/\s+/)[0].toLowerCase();
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  const actual = [...new Uint8Array(digest)].map(b => b.toString(16).padStart(2, '0')).join('');
  return actual === expected;
}

async function getSlots() {
  try {
    const s = await sendCommand({ cmd: 'get_slots' }, 2000);
    el('otaOther').textContent = s.other || 'none';
    el('btnRollback').textContent = s.other ? 'Flash ' + s.other : 'Flash';
    el('btnRollback').disabled = !connected || !s.other;
  } catch (e) {
    el('otaOther').textContent = 'unknown';
  }
}

async function rollbackNow() {
  if (!connected) return;
  const other = el('otaOther').textContent;
  if (!confirm('Roll back to ' + other + '? The device reboots into the firmware already stored in the other slot - nothing is downloaded.')) return;
  el('btnRollback').disabled = true;
  try { await sendCommand({ cmd: 'rollback' }, 3000); }
  catch (e) { /* the reboot races the ack - expected */ }
  el('otaMsg').textContent = 'Rollback requested - device rebooting into ' + other;
  log('Rollback requested', 'ok');
}

/* ------------------------ OTA over BLE ---------------------------- */
el('otaFile').addEventListener('change', updateOtaStatus);

/* Waits until the device reports >= target bytes actually written to flash.
 * Polls instead of parking a resolver on the notification handler, so an
 * ack can never be dropped between the handler and a waiter. Throws on
 * timeout - the caller aborts the OTA. */
async function waitUntilAcked(target) {
  const deadline = Date.now() + OTA_ACK_TIMEOUT;
  while (otaAcked < target) {
    if (Date.now() > deadline) {
      throw new Error('Device stopped acking at ' + otaAcked + '/' + target + ' bytes');
    }
    await sleep(20);
  }
}

async function otaUpload() {
  if (otaBusy) return;
  let bytes, label;
  const f = el('otaFile').files[0];
  if (f) {
    /* Local fallback file */
    label = f.name;
    const ab = await new Promise((res, rej) => {
      const r = new FileReader();
      r.onload = () => res(r.result);
      r.onerror = () => rej(r.error);
      r.readAsArrayBuffer(f);
    });
    bytes = new Uint8Array(ab);
  } else {
    /* Selected GitHub release */
    const rel = releases[Number(el('otaVersion').value || 0)];
    if (!rel) { el('otaMsg').textContent = 'No release selected'; return; }
    label = rel.tag;
    el('otaMsg').textContent = 'Downloading ' + rel.tag + '…';
    try {
      bytes = await downloadFirmware(rel);
    } catch (e) {
      el('otaMsg').textContent = 'Download failed: ' + e.message;
      log('Download failed: ' + e.message, 'err');
      return;
    }
    /* CI always uploads a .sha256 sidecar next to the .bin - verify it */
    el('otaMsg').textContent = 'Verifying SHA-256…';
    try {
      if (!(await verifySha256(bytes, rel))) {
        el('otaMsg').textContent = 'SHA-256 mismatch - download corrupted, aborted.';
        log('SHA-256 mismatch - aborted', 'err');
        return;
      }
      log('SHA-256 verified', 'ok');
    } catch (e) {
      el('otaMsg').textContent = 'SHA-256 check failed: ' + e.message;
      log('SHA check failed: ' + e.message, 'err');
      return;
    }
  }
  if (!confirm('Flash ' + label + ' (' + (bytes.length / 1048576).toFixed(2) + ' MB, about ' +
               Math.max(1, Math.round(bytes.length / OTA_BYTES_PER_SEC / 60)) +
               ' min over BLE)? Do not power off the device.')) return;

  otaBusy = true;
  el('btnOta').disabled = true;
  el('otaBar').style.width = '0%';
  el('otaMsg').textContent = 'Starting… keep this tab open!';

  const ctrl = (o) => chrOtac.writeValue(new TextEncoder().encode(JSON.stringify(o)));
  try {
    log('OTA begin: ' + bytes.length + ' bytes');

    await ctrl({ cmd: 'begin', size: bytes.length });
    log('OTA accepted by device', 'ok');

    otaSent = 0;
    otaAcked = 0;
    /* ble protocol v2 supports flow-controlled write-without-response
     * streaming (fast). v1 devices get write-with-response, which self-
     * paces to the device via the ATT response and cannot be outrun. */
    const useWnr = bleVer >= 2;
    log('Streaming via ' + (useWnr ? 'windowed write-without-response' : 'write-with-response'));

    for (let off = 0; off < bytes.length; off += MTU_CHUNK) {
      const end = Math.min(off + MTU_CHUNK, bytes.length);
      if (useWnr && end - otaAcked > OTA_WINDOW_BYTES) {
        /* Device flash pipeline is behind - hold until it drains back
         * inside the window, or abort on timeout. */
        await waitUntilAcked(end - OTA_WINDOW_BYTES);
      }
      if (useWnr) {
        await chrOtad.writeValueWithoutResponse(bytes.slice(off, end));
      } else {
        await chrOtad.writeValue(bytes.slice(off, end));
      }
      otaSent = end;
      const pct = Math.round((end / bytes.length) * 100);
      el('otaBar').style.width = pct + '%';
      el('otaMsg').textContent = 'Flashing ' + pct + '% - do not power off!';
    }
    if (useWnr) {
      /* Every byte must actually be written to flash before finalizing:
       * otherwise `complete` queues behind the drain and Chrome's ~30 s
       * op timeout kills the link before the device can finalize. */
      await waitUntilAcked(bytes.length);
    }
    log('OTA stream complete, finalizing…');

    await ctrl({ cmd: 'complete' });
    el('otaBar').style.width = '100%';
    el('otaMsg').textContent = 'Update written - rebooting into drive mode (hold boot 3 s to return here)';
    log('OTA complete - device rebooting', 'ok');
  } catch (e) {
    el('otaMsg').textContent = 'OTA failed: ' + e.message;
    log('OTA failed: ' + e.message, 'err');
    try { await ctrl({ cmd: 'abort' }); } catch (_) {}
  } finally {
    otaBusy = false;
    el('btnOta').disabled = !connected;
    updateOtaStatus();
  }
}

/* ------------------ Telemetry (one-shot snapshot) ------------------ */
/* No continuous polling: the GATT channel goes quiet between user actions,
 * which keeps Android Chrome's flaky GATT plumbing out of the picture. The
 * status line is a connect-time health snapshot; live tuning telemetry
 * lives on the AP-mode web UI over HTTP. */
async function refreshState() {
  const s = await sendCommand({ cmd: 'get_state' }, 2000);
  setStatus((s.rx_valid ? 'RX OK' : 'NO RX!') + '  ·  gyro ' + (s.gyro_ready ? 'OK' : '---'),
            s.rx_valid ? '' : 'err');
}

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

/* --------------------------- Wiring ------------------------------- */
el('btnConn').addEventListener('click', connect);
el('btnSave').addEventListener('click', saveCfg);
el('btnLoad').addEventListener('click', loadCfg);
el('btnCal').addEventListener('click', calibrate);
el('btnResetHead').addEventListener('click', resetHead);
el('btnOta').addEventListener('click', otaUpload);
el('btnFlashVersion').addEventListener('click', otaUpload);
el('otaVersion').addEventListener('change', updateOtaStatus);
el('btnRollback').addEventListener('click', rollbackNow);

/* ---------------------------- Init -------------------------------- */
setConnectedUI(false);
log('Ready. Tap Connect and pick gyroX.');
if (!navigator.bluetooth) log('Web Bluetooth not available in this browser - use Chrome/Edge on Android.', 'err');


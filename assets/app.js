// Watchmen – Weboberfläche (Live-Bild, Radar, Kalibrierung, Einstellungen)
'use strict';

const $ = (id) => document.getElementById(id);
const socket = io(window.location.origin, { path: '/socket.io', transports: ['polling', 'websocket'] });

const CALIB_STATES = ['kalibrierung', 'kalibrierung2', 'kalibrierung2_manuell'];
const CALIB_FIELDS = ['tilt_min', 'tilt_level', 'tilt_max', 'radar_min', 'radar_level', 'radar_max',
  'radar_comp_factor', 'radar_comp_enabled', 'pan_min', 'pan_center', 'pan_max', 'stepper_invert',
  'servo_us_min', 'servo_us_max', 'homing_max_steps'];
const LABEL_KEYS = { person: 'text_person', car: 'text_car', marker: 'text_marker' };

let state = null;
let config = null;
let activeTab = 'live';
let calibDirty = false;
let manualDirty = false;
let settingsDirty = false;
let sliderBusy = { tilt: false, radar: false };
let lastCalibState = '';
let radarData = { targets: [], ok: false, max_range: 8000, hfov: 62.2, wake_distance: 3000 };

// ------------------------------------------------------------------ Verbindung ----
socket.on('connect', () => { $('conn').classList.add('on'); });
socket.on('disconnect', () => { $('conn').classList.remove('on'); $('stateBadge').textContent = 'Keine Verbindung'; });
socket.on('config', (cfg) => {
  config = cfg;
  if (!calibDirty) fillCalibForm();
  if (!settingsDirty) fillSettingsForm();
  if (!manualDirty) fillManualDirs();
});
socket.on('state', (s) => { state = s; renderState(); });
socket.on('radar', (r) => { radarData = r; renderRadarTable(); });
socket.on('calib_error', (e) => {
  const box = $('calibErrors');
  box.textContent = 'Nicht gespeichert: ' + (e.errors || []).join(' · ');
  box.classList.remove('hidden');
});

function send(cmd, data) { socket.emit(cmd, data || {}); }

// ------------------------------------------------------------------ Reiter --------
document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));

function showTab(name) {
  activeTab = name;
  document.querySelectorAll('.tab').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tabpane').forEach((p) => p.classList.toggle('active', p.id === 'tab-' + name));
  updateStreams();
  if (name === 'marker' && !$('markerImg').src) showMarker();
}

function setStream(img, on) {
  if (on) {
    if (!img.getAttribute('src')) img.src = 'stream?t=' + Date.now();
  } else if (img.getAttribute('src')) {
    img.removeAttribute('src');
  }
}

function updateStreams() {
  setStream($('video'), activeTab === 'live');
  setStream($('video2'), activeTab === 'calib');
}
['video', 'video2'].forEach((id) => {
  $(id).addEventListener('error', () => {
    const img = $(id);
    img.removeAttribute('src');
    setTimeout(updateStreams, 2000);
  });
});

// ------------------------------------------------------------------ Befehle -------
document.querySelectorAll('[data-cmd]').forEach((b) => b.addEventListener('click', () => send(b.dataset.cmd)));
document.querySelectorAll('[data-jog]').forEach((b) =>
  b.addEventListener('click', () => send('cal_jog', { steps: parseInt(b.dataset.jog, 10) })));

// ------------------------------------------------------------------ Status --------
function yesNo(v, yes = 'ja', no = 'nein') {
  return `<span class="${v ? 'yes' : 'no'}">${v ? yes : no}</span>`;
}

function renderState() {
  if (!state) return;
  const badge = $('stateBadge');
  badge.textContent = state.state_text + (state.error ? ' – ' + state.error : '');
  badge.className = 'badge s-' + state.state;

  const lb = $('labelBox');
  const key = LABEL_KEYS[state.label];
  lb.textContent = key && config ? config[key] : '–';
  lb.className = 'label-box ' + (state.label || 'none');
  $('detList').innerHTML = (state.detections || [])
    .map((d) => `<span class="chip">${escapeHtml(d.label)} ${d.confidence}%</span>`).join('');
  $('fps').textContent = state.camera ? `${state.fps} Bilder/s` : 'Kamera aus';

  const m = state.mcu;
  $('kvState').textContent = state.state_text;
  $('kvSleep').textContent = state.sleep_in === null ? '–' : `${state.sleep_in} s`;
  $('kvMount').textContent = state.flipped ? 'Decke (Bild gedreht)' : 'normal';
  $('kvMotion').innerHTML = yesNo(state.motion_enabled, 'aktiv', 'aus');
  $('kvMcu').innerHTML = yesNo(m.online, 'verbunden', 'getrennt');
  $('kvMotors').innerHTML = yesNo(m.motors_present, 'angeschlossen', 'nicht angeschlossen');
  $('kvHomed').innerHTML = yesNo(m.homed, 'bekannt', 'unbekannt');
  $('kvPan').textContent = `${m.pan_pos} Halbschritte${m.pan_moving ? ' (fährt)' : ''}`;
  $('kvTilt').textContent = `${m.tilt.toFixed(1)}°`;
  $('kvRadarServo').textContent = `${m.radar.toFixed(1)}°`;
  $('kvSwitch').textContent = m.switch ? 'gedrückt' : 'offen';
  $('kvMpu').innerHTML = m.mpu_ok ? `<span class="yes">ok</span> (${m.acc.join(' / ')} mg)` : yesNo(false, '', 'fehlt');
  $('kvRadar').innerHTML = yesNo(m.radar_ok, 'liefert Daten', 'keine Daten');

  $('events').innerHTML = (state.events || [])
    .map((e) => `<li><time>${e.t}</time>${escapeHtml(e.text)}</li>`).join('');

  renderCalib();
}

// ------------------------------------------------------------------ Kalibrierung --
function renderCalib() {
  const m = state.mcu;
  const inCalib = CALIB_STATES.includes(state.state);
  $('calibNoMotors').classList.toggle('hidden', m.motors_present || !m.online);
  $('calibClosed').classList.toggle('hidden', inCalib || !m.motors_present);
  $('calibBody').classList.toggle('hidden', !inCalib);
  $('calibStateText').textContent = state.state_text;

  if (inCalib && lastCalibState !== state.state && !calibDirty) fillCalibForm();
  if (inCalib && !CALIB_STATES.includes(lastCalibState)) {
    syncSliders(true);
  }
  lastCalibState = state.state;
  syncSliders(false);

  const running = state.state === 'kalibrierung2';
  document.querySelectorAll('#calibBody .col input, #calibBody .col button').forEach((el) => {
    if (el.dataset.cmd !== 'cal_stop') el.disabled = running;
  });
  renderCalibSummary();

  $('calPan').textContent = `${m.pan_pos}${m.pan_moving ? ' (fährt)' : ''}`;
  $('calHomed').innerHTML = yesNo(m.homed, 'bekannt', 'unbekannt – Referenzfahrt nötig');
  $('calSwitch').textContent = m.switch ? 'gedrückt' : 'offen';

  const c1 = config && config.calib1_done;
  const c2 = config && config.calib2_done;
  const s1 = $('step1'), s2 = $('step2'), s3 = $('step3');
  s1.className = 'step ' + (c1 ? 'done' : (state.state === 'kalibrierung' ? 'active' : ''));
  s2.className = 'step ' + (c2 ? 'done' : (c1 && inCalib ? 'active' : ''));
  s3.className = 'step ' + (c1 && c2 ? 'done' : '');

  const canStep2 = c1 && (state.state === 'kalibrierung' || state.state === 'kalibrierung2_manuell');
  $('btnStart2').disabled = !canStep2;
  $('btnSave2').disabled = !canStep2;

  const c = state.calib2 || {};
  let html = '';
  if (state.state === 'kalibrierung2') html += `<p><b>Läuft:</b> ${escapeHtml(c.message || '')}</p>`;
  else if (c.message) html += `<p>${escapeHtml(c.message)}</p>`;
  if (c.result && (c.result.pan || c.result.tilt)) {
    const row = (name, r, dirText) => {
      if (!r) return '';
      const checks = Object.entries(r.checks || {}).map(([k, v]) => `${v ? '✓' : '✗'} ${k}`).join(', ');
      return `<tr><td>${name}</td><td>${r.ok ? '<span class="yes">erkannt</span>' : '<span class="no">unsicher</span>'}</td><td>${dirText}</td><td class="muted">${checks}</td></tr>`;
    };
    const pd = c.result.pan_dir === 1 ? 'nach rechts' : c.result.pan_dir === -1 ? 'nach links' : '–';
    const td = c.result.tilt_dir === 1 ? 'nach unten' : c.result.tilt_dir === -1 ? 'nach oben' : '–';
    html += `<table><tr><th>Achse</th><th>Ergebnis</th><th>Richtung</th><th>Prüfungen</th></tr>
      ${row('Drehen', c.result.pan, 'positive Schritte ' + pd)}
      ${row('Neigen', c.result.tilt, 'größerer Winkel ' + td)}</table>`;
    if (!manualDirty) {
      if (c.result.pan_dir) setRadio('pan_dir', c.result.pan_dir);
      if (c.result.tilt_dir) setRadio('tilt_dir', c.result.tilt_dir);
    }
  }
  $('calib2Status').innerHTML = html;
}

function renderCalibSummary() {
  if (!config) return;
  const c = config;
  const dirPan = c.pan_dir >= 0 ? 'rechts' : 'links';
  const dirTilt = c.tilt_dir >= 0 ? 'unten' : 'oben';
  const done = c.calib1_done && c.calib2_done;
  $('calibSummary').innerHTML = `<table>
    <tr><td>Status</td><td>${done ? '<span class="yes">vollständig kalibriert</span>' : '<span class="no">nicht vollständig kalibriert</span>'}</td></tr>
    <tr><td>Servo 1 (Kamera)</td><td>${c.tilt_min}° … ${c.tilt_max}°, waagerecht ${c.tilt_level}°</td></tr>
    <tr><td>Servo 2 (Radar)</td><td>${c.radar_min}° … ${c.radar_max}°, waagerecht ${c.radar_level}°, Ausgleich ${c.radar_comp_enabled ? 'Faktor ' + c.radar_comp_factor : 'aus'}</td></tr>
    <tr><td>Drehachse</td><td>${c.pan_min} … ${c.pan_max} Halbschritte, Grundstellung ${c.pan_center}${c.stepper_invert ? ', Richtung umgekehrt' : ''}</td></tr>
    <tr><td>Richtungen</td><td>+Schritte → Kamera nach ${dirPan}, größerer Winkel → Kamera nach ${dirTilt}${c.calib2_method ? ' (' + c.calib2_method + ')' : ''}</td></tr>
  </table>`;
}

function syncSliders(force) {
  if (!state) return;
  const m = state.mcu;
  if (force || !sliderBusy.tilt) {
    if (force) { $('tiltSlider').value = m.tilt; }
    $('tiltOut').textContent = `${m.tilt.toFixed(1)}°`;
  }
  if (force || !sliderBusy.radar) {
    if (force) { $('radarSlider').value = m.radar; }
    $('radarOut').textContent = `${m.radar.toFixed(1)}°`;
  }
}

function throttle(fn, ms) {
  let last = 0, timer = null, lastArgs = null;
  return (...args) => {
    lastArgs = args;
    const now = Date.now();
    if (now - last >= ms) { last = now; fn(...args); }
    else if (!timer) {
      timer = setTimeout(() => { timer = null; last = Date.now(); fn(...lastArgs); }, ms - (now - last));
    }
  };
}

const sendTilt = throttle((v) => send('cal_tilt', { angle: v }), 80);
const sendRadar = throttle((v) => send('cal_radar', { angle: v }), 80);

$('tiltSlider').addEventListener('input', (e) => {
  sliderBusy.tilt = true;
  $('tiltOut').textContent = `${parseFloat(e.target.value).toFixed(1)}°`;
  sendTilt(parseFloat(e.target.value));
});
$('tiltSlider').addEventListener('change', () => { setTimeout(() => { sliderBusy.tilt = false; }, 1500); });
$('radarSlider').addEventListener('input', (e) => {
  sliderBusy.radar = true;
  $('radarOut').textContent = `${parseFloat(e.target.value).toFixed(1)}°`;
  sendRadar(parseFloat(e.target.value));
});
$('radarSlider').addEventListener('change', () => { setTimeout(() => { sliderBusy.radar = false; }, 1500); });

document.querySelectorAll('[data-take]').forEach((b) => b.addEventListener('click', () => {
  if (!state) return;
  const m = state.mcu;
  const src = b.dataset.src;
  const value = src === 'tilt' ? m.tilt : src === 'radar' ? m.radar : m.pan_pos;
  $(b.dataset.take).value = src === 'pan' ? Math.round(value) : value.toFixed(1);
  calibDirty = true;
}));

CALIB_FIELDS.forEach((f) => {
  const el = $(f);
  el.addEventListener('input', () => { calibDirty = true; });
});

function readCalibForm() {
  const out = {};
  CALIB_FIELDS.forEach((f) => {
    const el = $(f);
    out[f] = el.type === 'checkbox' ? el.checked : parseFloat(el.value);
  });
  return out;
}

function fillCalibForm() {
  if (!config) return;
  CALIB_FIELDS.forEach((f) => {
    const el = $(f);
    if (el.type === 'checkbox') el.checked = !!config[f];
    else el.value = config[f];
  });
}

$('stepper_invert').addEventListener('change', () => send('cal_preview', readCalibForm()));
$('btnCompTest').addEventListener('click', () => {
  send('cal_preview', readCalibForm());
  send('cal_radar', { angle: null });
  sliderBusy.radar = false;
});
$('btnSave1').addEventListener('click', () => {
  $('calibErrors').classList.add('hidden');
  send('cal_save1', readCalibForm());
  calibDirty = false;
});
$('btnStart2').addEventListener('click', () => send('cal_start2'));
$('btnSave2').addEventListener('click', () => {
  const pan = getRadio('pan_dir');
  const tilt = getRadio('tilt_dir');
  if (pan === null || tilt === null) { alert('Bitte beide Richtungen auswählen.'); return; }
  send('cal_save2', { pan_dir: pan, tilt_dir: tilt });
  manualDirty = false;
});
$('btnReset').addEventListener('click', () => {
  if (confirm('Kalibrierung wirklich zurücksetzen? Danach müssen Bereiche und Richtungen neu eingestellt werden.')) {
    send('cal_reset');
  }
});

document.querySelectorAll('input[name=pan_dir], input[name=tilt_dir]').forEach((r) =>
  r.addEventListener('change', () => { manualDirty = true; }));

function setRadio(name, value) {
  const el = document.querySelector(`input[name=${name}][value="${value}"]`);
  if (el) el.checked = true;
}
function getRadio(name) {
  const el = document.querySelector(`input[name=${name}]:checked`);
  return el ? parseInt(el.value, 10) : null;
}
function fillManualDirs() {
  if (!config) return;
  setRadio('pan_dir', config.pan_dir >= 0 ? 1 : -1);
  setRadio('tilt_dir', config.tilt_dir >= 0 ? 1 : -1);
}

// ------------------------------------------------------------------ Einstellungen -
const form = $('settingsForm');
form.addEventListener('input', () => { settingsDirty = true; $('settingsMsg').textContent = 'ungespeicherte Änderungen'; });
form.addEventListener('submit', (e) => {
  e.preventDefault();
  const out = {};
  form.querySelectorAll('[name]').forEach((el) => {
    if (el.type === 'checkbox') out[el.name] = el.checked;
    else if (el.type === 'number') out[el.name] = parseFloat(el.value);
    else out[el.name] = el.value;
  });
  send('settings_save', out);
  settingsDirty = false;
  $('settingsMsg').textContent = 'gespeichert';
});

function fillSettingsForm() {
  if (!config) return;
  form.querySelectorAll('[name]').forEach((el) => {
    if (!(el.name in config)) return;
    if (el.type === 'checkbox') el.checked = !!config[el.name];
    else el.value = config[el.name];
  });
}

// ------------------------------------------------------------------ Marker --------
function showMarker() {
  const id = Math.max(0, parseInt($('markerId').value || '0', 10));
  $('markerImg').src = `marker.png?id=${id}&t=${Date.now()}`;
}
$('btnMarkerShow').addEventListener('click', showMarker);
$('btnMarkerPrint').addEventListener('click', () => { showMarker(); setTimeout(() => window.print(), 400); });

// ------------------------------------------------------------------ Radar ---------
const canvas = $('radar');
const ctx = canvas.getContext('2d');
let sweep = 0;

function renderRadarTable() {
  const rows = (radarData.targets || []).map((t, i) =>
    `<tr${t.selected ? ' style="font-weight:600"' : ''}><td>${i + 1}${t.selected ? ' ★' : ''}</td><td>${(t.x / 1000).toFixed(2)}</td><td>${(t.y / 1000).toFixed(2)}</td>` +
    `<td>${(t.distance / 1000).toFixed(2)}</td><td>${t.angle.toFixed(0)}°</td><td>${t.speed}</td></tr>`).join('');
  $('radarTable').innerHTML = '<tr><th>Ziel</th><th>x [m]</th><th>y [m]</th><th>Abstand [m]</th><th>Winkel</th><th>v [cm/s]</th></tr>' +
    (rows || '<tr><td colspan="6" class="muted">keine Lebewesen erfasst</td></tr>');
  $('radarInfo').textContent = radarData.ok ? `${(radarData.targets || []).length} Ziel(e)` : 'keine Radar-Daten';
}

function drawRadar() {
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H - 18, R = Math.min(W / 2 - 14, H - 34);
  const maxR = radarData.max_range || 8000;
  const toPx = (x, y) => [cx + (x / maxR) * R, cy - (y / maxR) * R];
  const ang = (deg) => (-90 + deg) * Math.PI / 180;   // 0° = nach vorne (oben)

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#0f1f1e';
  ctx.fillRect(0, 0, W, H);

  // Halbkreis
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, R, Math.PI, 2 * Math.PI); ctx.closePath();
  ctx.fillStyle = '#122826'; ctx.fill();

  // Kamera-Sichtfeld
  const half = (radarData.hfov || 62) / 2;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, R, ang(-half), ang(half)); ctx.closePath();
  ctx.fillStyle = 'rgba(62,224,143,0.12)'; ctx.fill();

  // Entfernungsringe
  ctx.strokeStyle = '#2e5a55'; ctx.fillStyle = '#8fc9c2'; ctx.font = '11px system-ui'; ctx.lineWidth = 1;
  const stepM = maxR > 6000 ? 2000 : 1000;
  for (let r = stepM; r <= maxR; r += stepM) {
    ctx.beginPath(); ctx.arc(cx, cy, (r / maxR) * R, Math.PI, 2 * Math.PI); ctx.stroke();
    ctx.fillText(`${r / 1000} m`, cx + 4, cy - (r / maxR) * R + 12);
  }
  // Winkellinien
  [-90, -60, -30, 0, 30, 60, 90].forEach((d) => {
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + R * Math.cos(ang(d)), cy + R * Math.sin(ang(d)));
    ctx.strokeStyle = Math.abs(d) === 60 ? '#3f7d76' : '#24423f'; ctx.stroke();
    if (Math.abs(d) < 90) {
      ctx.fillText(`${d}°`, cx + (R + 2) * Math.cos(ang(d)) - 10, cy + (R + 2) * Math.sin(ang(d)) - 2);
    }
  });

  // Weckabstand
  if (radarData.wake_distance) {
    ctx.setLineDash([6, 5]); ctx.strokeStyle = '#ffb347';
    ctx.beginPath(); ctx.arc(cx, cy, Math.min(1, radarData.wake_distance / maxR) * R, Math.PI, 2 * Math.PI); ctx.stroke();
    ctx.setLineDash([]);
  }

  // Sweep
  sweep = (sweep + 1.6) % 180;
  const sd = -90 + sweep;
  const grad = ctx.createLinearGradient(cx, cy, cx + R * Math.cos(ang(sd)), cy + R * Math.sin(ang(sd)));
  grad.addColorStop(0, 'rgba(62,224,143,0.0)'); grad.addColorStop(1, 'rgba(62,224,143,0.45)');
  ctx.strokeStyle = grad; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R * Math.cos(ang(sd)), cy + R * Math.sin(ang(sd))); ctx.stroke();
  ctx.lineWidth = 1;

  // Ziele
  (radarData.targets || []).forEach((t) => {
    const [px, py] = toPx(t.x, t.y);
    ctx.beginPath(); ctx.arc(px, py, t.selected ? 9 : 7, 0, 2 * Math.PI);
    ctx.fillStyle = t.selected ? '#ffd23f' : '#3ee08f'; ctx.fill();
    if (t.selected) { ctx.strokeStyle = '#ff5a4a'; ctx.lineWidth = 3; ctx.stroke(); ctx.lineWidth = 1; }
    ctx.fillStyle = '#dff7f3'; ctx.font = '12px system-ui';
    ctx.fillText(`${(t.distance / 1000).toFixed(1)} m`, px + 11, py + 4);
  });

  // Sensor
  ctx.fillStyle = '#3ee08f';
  ctx.fillRect(cx - 7, cy - 3, 14, 6);

  if (!radarData.ok) {
    ctx.fillStyle = 'rgba(15,31,30,0.65)'; ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = '#ffb347'; ctx.font = '16px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('Keine Radar-Daten (RD-03D prüfen)', cx, H / 2); ctx.textAlign = 'start';
  }
  requestAnimationFrame(drawRadar);
}

// ------------------------------------------------------------------ Hilfen --------
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

renderRadarTable();
requestAnimationFrame(drawRadar);
updateStreams();

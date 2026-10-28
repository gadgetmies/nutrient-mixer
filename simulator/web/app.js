// Simulator UI: connects to the simulator over a WebSocket and keeps the views up to date.
import { FrontPanel } from './display.js';
import { SystemDiagram } from './system.js';
import { BoardView } from './board.js';
import { Charts } from './charts.js';

const $ = (id) => document.getElementById(id);
let socket = null;
let lastState = null;
let paramDefinitions = [];

export function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

const frontPanel = new FrontPanel($('frontPanel'), send);
const system = new SystemDiagram($('systemDiagram'), send);
const board = new BoardView($('boardView'), $('boardHover'), frontPanel);
const charts = new Charts($('charts'));

// CONNECTION

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  socket.onopen = () => {
    $('connection').textContent = 'connected';
    $('connection').classList.add('online');
  };
  socket.onclose = () => {
    $('connection').textContent = 'offline – reconnecting';
    $('connection').classList.remove('online');
    setTimeout(connect, 700);
  };
  socket.onmessage = (event) => {
    if (typeof event.data !== 'string') {
      const bytes = new Uint8Array(event.data);
      if (bytes[0] === 1) frontPanel.setPanel(bytes);
      return;
    }
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      paramDefinitions = message.params;
      buildParams();
      appendConsole('\n— connected to the simulator —\n', 'ok');
    } else if (message.type === 'serial') {
      appendConsole(message.text);
    } else if (message.type === 'state') {
      if (lastState && message.time < lastState.time - 0.5) appendConsole('\n— ESP32 restarted —\n', 'ok');
      lastState = message;
      update(message);
    }
  };
}

function update(state) {
  updateHeader(state);
  updateStatus(state);
  frontPanel.update(state);
  system.update(state);
  board.update(state);
  charts.add(state);
  updateScreenImages();
  updateParams(state);
  updateWorld(state);
  updateWifi(state);
  updateWarnings(state);
}

// Copies of the display for the diagram and the board view (a few times per second)
let screenVersion = -1;
let screenCopiedAt = 0;
function updateScreenImages() {
  const now = performance.now();
  if (frontPanel.screenVersion === screenVersion || now - screenCopiedAt < 250) return;
  screenVersion = frontPanel.screenVersion;
  screenCopiedAt = now;
  const url = frontPanel.backlight ? frontPanel.screen.toDataURL() : '';
  system.setScreen(url);
  board.setScreen(url);
}

// HEADER

function formatDuration(seconds) {
  seconds = Math.floor(seconds);
  const h = Math.floor(seconds / 3600), m = Math.floor(seconds / 60) % 60, s = seconds % 60;
  return (h ? h + ':' + String(m).padStart(2, '0') : m) + ':' + String(s).padStart(2, '0');
}

function updateHeader(state) {
  const fw = state.firmware;
  $('fwStateName').textContent = fw.state;
  let sub = '';
  if (fw.state === 'ADJUST_TDS' || fw.state === 'ADJUST_ALL') sub = `${fw.tdsState} · round ${fw.tdsRound}/${fw.maxRounds}`;
  if (fw.state === 'ADJUST_PH') sub = `${fw.phState} · round ${fw.phRound}/${fw.maxRounds}`;
  if (fw.countDown != null) sub += (sub ? ' · ' : '') + `${fw.countDown.toFixed(1)} s left`;
  $('fwSubState').textContent = sub;
  $('fwState').classList.toggle('busy', fw.state !== 'IDLE');
  $('uptime').textContent = formatDuration(state.time);
  $('actualSpeed').textContent = state.paused ? 'paused' : `${state.actualSpeed.toFixed(state.actualSpeed < 10 ? 1 : 0)}×`;
  $('pauseButton').classList.toggle('paused', state.paused);
  $('pauseButton').textContent = state.paused ? '▶' : '❚❚';
  for (const button of $('speedButtons').children) {
    button.classList.toggle('active', Number(button.dataset.speed) === state.speed);
  }
}

$('pauseButton').onclick = () => send({ type: 'pause', value: !(lastState && lastState.paused) });
$('stepButton').onclick = () => send({ type: 'step', seconds: 1 });
for (const button of $('speedButtons').children) {
  button.onclick = () => send({ type: 'speed', value: Number(button.dataset.speed) });
}
$('resetDevice').onclick = () => send({ type: 'device', action: 'reset' });
$('eraseFlash').onclick = () => {
  if (confirm('Erase the NVS flash (targets, pump amounts, calibration, WiFi credentials) and restart the ESP32?')) {
    send({ type: 'device', action: 'eraseFlash' });
  }
};

// FIRMWARE STATUS

function stat(label, value, note = '') {
  return `<div class="stat"><div class="k">${label}</div><div class="v">${value}</div>${note ? `<div class="k">${note}</div>` : ''}</div>`;
}

function updateStatus(state) {
  const fw = state.firmware;
  const progress = fw.countDown != null && fw.countDownTotal ? 100 * (1 - fw.countDown / fw.countDownTotal) : 0;
  const pumpTag = (name, pump) =>
    `<span class="tag ${pump.blocked ? 'blocked' : pump.requested ? 'on' : ''}">${name}${pump.blocked ? ' blocked by float' : pump.requested ? ' requested' : ''}</span>`;
  $('statusPanel').innerHTML = `
    <div class="status-grid">
      ${stat('pH (firmware)', escapeHtml(fw.ph), `target ${fw.targetPh.toFixed(2)}`)}
      ${stat('TDS (firmware)', escapeHtml(fw.tds), `target ${Math.round(fw.targetTds)} ppm`)}
      ${stat('Sensor powered', fw.poweredSensor)}
      ${stat('WiFi', escapeHtml(fw.wifi))}
    </div>
    <div class="progress" title="Current timed step"><div style="width:${progress}%"></div></div>
    <div class="status-line">
      ${pumpTag('Input pump', fw.inputPump)} ${pumpTag('Output pump', fw.outputPump)}
      <span>Doses: acid ${fw.acidSeconds} s · A ${fw.nutrientASeconds} s · B ${fw.nutrientBSeconds} s</span>
      <span>Fill ${fw.fillSeconds} s · empty ${fw.emptySeconds} s · circulate ${fw.circulateSeconds} s</span>
      <span class="mono">595: ${fw.register.toString(2).padStart(8, '0')}</span>
      ${fw.editMode ? '<span class="tag on">menu edit mode</span>' : ''}
    </div>`;
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
}

// CONSOLE

const consoleOutput = $('consoleOutput');
let consoleBuffer = '';
let consolePending = '';
const maxConsoleLines = 4000;

function classify(line) {
  if (line.startsWith('*wm:')) return 'wm';
  if (line.startsWith('Setting shift register') || line.startsWith('GPIO') || line === '') return 'dbg';
  if (line.startsWith('Received command')) return 'cmd';
  if (/error|failed|rejected|saturated|noisy|giving up|triggered|Unknown|Invalid|must be/i.test(line)) return 'err';
  if (/reached|done|configured|Connected/i.test(line)) return 'ok';
  return '';
}

function appendConsole(text, forcedClass) {
  consolePending += text;
  const lines = consolePending.split('\n');
  consolePending = lines.pop();
  const atBottom = consoleOutput.scrollTop + consoleOutput.clientHeight >= consoleOutput.scrollHeight - 30;
  const hideDebug = $('hideDebug').checked;
  const fragment = document.createDocumentFragment();
  for (let line of lines) {
    line = line.replace(/\r$/, '');
    const cls = forcedClass || classify(line);
    if (hideDebug && cls === 'dbg') continue;
    const div = document.createElement('div');
    div.textContent = line || ' ';
    if (cls) div.className = cls;
    fragment.appendChild(div);
  }
  consoleOutput.appendChild(fragment);
  while (consoleOutput.childElementCount > maxConsoleLines) consoleOutput.firstChild.remove();
  if (atBottom) consoleOutput.scrollTop = consoleOutput.scrollHeight;
}

const history = [];
let historyIndex = 0;
$('consoleForm').onsubmit = (event) => {
  event.preventDefault();
  const text = $('consoleInput').value;
  send({ type: 'serial', text });
  if (text && history[history.length - 1] !== text) history.push(text);
  historyIndex = history.length;
  $('consoleInput').value = '';
};
$('consoleInput').onkeydown = (event) => {
  if (event.key === 'ArrowUp' && historyIndex > 0) {
    $('consoleInput').value = history[--historyIndex];
    event.preventDefault();
  } else if (event.key === 'ArrowDown') {
    historyIndex = Math.min(history.length, historyIndex + 1);
    $('consoleInput').value = history[historyIndex] || '';
    event.preventDefault();
  }
};
$('clearConsole').onclick = () => (consoleOutput.innerHTML = '');
const chips = [
  ['?', 'List commands'], ['m', 'Measure pH and TDS'], ['a', 'Adjust TDS, then pH, to the targets'],
  ['t', 'Adjust TDS'], ['p', 'Adjust pH'], ['c', 'Circulate'], ['f', 'Fill (input pump)'], ['e', 'Empty (output pump)'],
  ['n', 'Pump nutrients A and B'], ['u', 'Pump acid'], ['l 30', 'Plot sensor values for 30 s'], ['k', 'Show calibration'],
  ['w', 'WiFi status'], ['!', 'Abort'],
];
for (const [command, title] of chips) {
  const button = document.createElement('button');
  button.className = 'mono';
  button.textContent = command;
  button.title = title;
  button.onclick = () => send({ type: 'serial', text: command });
  $('commandChips').appendChild(button);
}

// PARAMETERS

function buildParams() {
  const panel = $('paramsPanel');
  panel.innerHTML = '';
  const groups = new Map();
  for (const p of paramDefinitions) {
    if (!groups.has(p.group)) groups.set(p.group, []);
    groups.get(p.group).push(p);
  }
  for (const [group, params] of groups) {
    const details = document.createElement('details');
    details.className = 'param-group';
    details.open = group === 'Faults' || group === 'Probes';
    details.innerHTML = `<summary>${group}</summary>`;
    for (const p of params) {
      const row = document.createElement('div');
      row.className = 'param';
      row.title = p.help;
      row.dataset.key = p.key;
      const id = 'param-' + p.key.replace(/\W/g, '-');
      if (p.options) {
        row.innerHTML = `<label for="${id}">${p.label}</label><div class="ctl"><select id="${id}">${p.options
          .map((o, i) => `<option value="${i}">${escapeHtml(o)}</option>`)
          .join('')}</select></div>`;
        row.querySelector('select').onchange = (e) => send({ type: 'param', key: p.key, value: Number(e.target.value) });
      } else {
        row.innerHTML = `<label for="${id}">${p.label}</label><div class="ctl"><input id="${id}" type="number" min="${p.min}" max="${p.max}" step="${p.step}"><span class="unit">${p.unit}</span></div>`;
        row.querySelector('input').onchange = (e) => send({ type: 'param', key: p.key, value: Number(e.target.value) });
      }
      details.appendChild(row);
    }
    panel.appendChild(details);
  }
}

function updateParams(state) {
  for (const p of paramDefinitions) {
    const row = document.querySelector(`.param[data-key="${CSS.escape(p.key)}"]`);
    if (!row) continue;
    const control = row.querySelector('input, select');
    const value = state.params[p.key];
    if (document.activeElement !== control && String(control.value) !== String(value)) control.value = value;
    row.classList.toggle('changed', value !== p.default);
  }
}
$('resetParams').onclick = () => send({ type: 'resetParams' });

// WORLD ACTIONS

function buildWorld() {
  $('worldActions').innerHTML = `
    <div class="row-actions"><span class="label">Start from</span>
      <select id="worldPreset"><option value="0">Used solution (~600 ppm)</option><option value="1">Fresh tap water</option><option value="2">Empty mixer reservoir</option></select>
      <button id="worldReset">Reset world</button></div>
    <div class="row-actions"><span class="label">Mixer</span>
      <input id="mixerVolume" type="number" min="0" step="0.5" style="width:70px"> L <button data-volume="mixer">Set</button></div>
    <div class="row-actions"><span class="label">Plant</span>
      <input id="plantVolume" type="number" min="0" step="1" style="width:70px"> L <button data-volume="plant">Set</button></div>
    <div class="row-actions"><span class="label">Bottles</span>
      <button data-refill="0">Refill pH Down</button><button data-refill="1">Refill A</button><button data-refill="2">Refill B</button></div>
    <div class="row-actions"><button id="clearSpill">Mop up spills</button></div>`;
  $('worldReset').onclick = () => send({ type: 'world', action: 'reset', preset: Number($('worldPreset').value) });
  for (const button of document.querySelectorAll('[data-volume]')) {
    button.onclick = () => {
      const tank = button.dataset.volume;
      send({ type: 'world', action: 'volume', tank, litres: Number($(tank + 'Volume').value) });
    };
  }
  for (const button of document.querySelectorAll('[data-refill]')) {
    button.onclick = () => send({ type: 'world', action: 'refill', bottle: Number(button.dataset.refill) });
  }
  $('clearSpill').onclick = () => send({ type: 'world', action: 'clearSpill' });
}
buildWorld();

function updateWorld(state) {
  for (const tank of ['mixer', 'plant']) {
    const input = $(tank + 'Volume');
    if (document.activeElement !== input) input.value = state.world[tank].volume.toFixed(1);
  }
}

// WIFI AND PHONE

let wifiBuilt = false;
function buildWifi(state) {
  $('wifiPanel').innerHTML = `
    <div id="wifiStatus" class="muted"></div>
    <div id="networkList"></div>
    <div class="phone">
      <div><strong>📱 Phone</strong> <span id="phoneStatus" class="muted"></span></div>
      <div class="msg" id="phoneMessage"></div>
      <div class="row-actions"><button id="phoneScan" title="Point the camera at the QR code on the T-Display">Scan QR code</button>
        <button id="phoneLeave">Leave hotspot</button></div>
      <div class="form">
        <div class="row-actions"><input id="joinSsid" placeholder="Hotspot (Mix-XXXX)" style="flex:1"><input id="joinPassword" placeholder="Password" style="width:100px"><button id="phoneJoin">Join</button></div>
        <div id="portalForm" class="row-actions"><span class="label">Setup page</span>
          <select id="portalSsid" style="flex:1"></select><input id="portalPassword" placeholder="Password" style="width:100px"><button id="portalSave">Save</button></div>
      </div>
    </div>`;
  $('phoneScan').onclick = () => send({ type: 'wifi', action: 'phoneScan' });
  $('phoneLeave').onclick = () => send({ type: 'wifi', action: 'phoneLeave' });
  $('phoneJoin').onclick = () => send({ type: 'wifi', action: 'phoneJoin', ssid: $('joinSsid').value, password: $('joinPassword').value });
  $('portalSave').onclick = () => send({ type: 'wifi', action: 'phoneSubmit', ssid: $('portalSsid').value, password: $('portalPassword').value });
  $('portalSsid').innerHTML = state.wifi.networks.map((n) => `<option>${escapeHtml(n.ssid)}</option>`).join('');
  wifiBuilt = true;
}

function updateWifi(state) {
  if (!wifiBuilt) buildWifi(state);
  const w = state.wifi;
  $('wifiStatus').innerHTML = `Station: <strong>${escapeHtml(w.station)}</strong>${w.address ? ' · ' + w.address : ''}${
    w.savedSsid ? ' · saved network ' + escapeHtml(w.savedSsid) : ' · no saved network'}<br>Setup hotspot: ${
    w.apActive ? '<strong>' + escapeHtml(w.apSsid) + '</strong> (192.168.4.1)' : 'off'}`;
  const list = $('networkList');
  const html = w.networks
    .map((n) => `<div class="network"><span class="ssid">📶 ${escapeHtml(n.ssid)} <span class="muted">${n.rssi} dBm · password “${escapeHtml(n.password)}”</span></span>
      <label><input type="checkbox" data-network="${escapeHtml(n.ssid)}" ${n.on ? 'checked' : ''}> router on</label></div>`)
    .join('');
  if (list.dataset.html !== html) {
    list.innerHTML = html;
    list.dataset.html = html;
    for (const box of list.querySelectorAll('input[data-network]')) {
      box.onchange = () => send({ type: 'wifi', action: 'network', ssid: box.dataset.network, on: box.checked });
    }
  }
  $('phoneStatus').textContent = w.phoneJoined ? `joined ${w.apSsid}` : 'not on the hotspot';
  $('phoneMessage').textContent = w.phoneMessage || (w.apActive ? 'The setup hotspot is up. Scan the QR code on the display, or join with the password shown there.' : '');
  $('portalForm').style.opacity = w.phoneJoined ? 1 : 0.45;
}

// WARNINGS

function updateWarnings(state) {
  const html = state.world.warnings.map((w) => `<div class="warning">${escapeHtml(w)}</div>`).join('');
  if ($('warnings').dataset.html !== html) {
    $('warnings').innerHTML = html;
    $('warnings').dataset.html = html;
  }
}

// TABS

for (const tab of document.querySelectorAll('.tabs button')) {
  tab.onclick = () => {
    for (const t of document.querySelectorAll('.tabs button')) t.classList.toggle('active', t === tab);
    for (const panel of document.querySelectorAll('.tab-panel')) panel.classList.toggle('active', panel.id === 'tab-' + tab.dataset.tab);
    if (tab.dataset.tab === 'charts') charts.draw();
    try { localStorage.setItem('tab', tab.dataset.tab); } catch (e) { /* storage unavailable */ }
  };
}
try {
  const saved = localStorage.getItem('tab');
  if (saved) document.querySelector(`.tabs button[data-tab="${saved}"]`)?.click();
} catch (e) { /* storage unavailable */ }

// KEYBOARD: the knob and buttons of the front panel

document.addEventListener('keydown', (event) => {
  if (event.target.closest('input, select, textarea') || event.metaKey || event.ctrlKey || event.altKey) return;
  if (event.key === 'p' || event.key === 'P') {
    send({ type: 'pause', value: !(lastState && lastState.paused) });
    event.preventDefault();
    return;
  }
  if (frontPanel.handleKey(event)) event.preventDefault();
});
document.addEventListener('keyup', (event) => {
  if (event.target.closest('input, select, textarea')) return;
  if (frontPanel.handleKeyUp(event)) event.preventDefault();
});

connect();

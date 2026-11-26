// System diagram: dosing bottles and peristaltic pumps, the mixer reservoir, the self-priming and submersible
// pumps, the plant reservoir, and the electronics that drive them, with the water and nutrients flowing.
import { svgElement as el } from './display.js';

const BOTTLE_COLORS = { acid: '#e0524a', nutrientA: '#d9a514', nutrientB: '#2aa198' };
const PIPE = '#59616a';
const EMPTY_PIPE = '#2a2f33';

function lerpColor(a, b, t) {
  const pa = [1, 3, 5].map((i) => parseInt(a.substr(i, 2), 16));
  const pb = [1, 3, 5].map((i) => parseInt(b.substr(i, 2), 16));
  return '#' + pa.map((v, i) => Math.round(v + (pb[i] - v) * t).toString(16).padStart(2, '0')).join('');
}

// Clean water is blue; nutrient solution turns it yellowish green
function waterColor(tds) {
  const t = Math.max(0, Math.min(1, tds / 2200));
  return t < 0.5 ? lerpColor('#3f93dc', '#4fae9a', t * 2) : lerpColor('#4fae9a', '#a8a43c', (t - 0.5) * 2);
}

function fmt(value, digits = 1) {
  return value == null || !isFinite(value) ? '–' : Number(value).toFixed(digits);
}

export class SystemDiagram {
  constructor(container, send) {
    this.send = send;
    this.svg = el('svg', { viewBox: '0 0 1200 830', role: 'img', 'aria-label': 'Plumbing and electronics' }, container);
    this.pipes = {};
    this.build();
  }

  // A pipe is a grey tube with the liquid drawn inside; the liquid moves (dashes) while it flows
  pipe(name, d, width = 9) {
    el('path', { d, fill: 'none', stroke: PIPE, 'stroke-width': width, 'stroke-linecap': 'round', 'stroke-linejoin': 'round' }, this.pipeLayer);
    const liquid = el('path', { d, fill: 'none', stroke: EMPTY_PIPE, 'stroke-width': width - 4, 'stroke-linecap': 'round', 'stroke-linejoin': 'round' }, this.pipeLayer);
    this.pipes[name] = liquid;
    return liquid;
  }

  setFlow(name, flowing, color, speed = 1) {
    const pipe = this.pipes[name];
    pipe.setAttribute('stroke', flowing ? color : EMPTY_PIPE);
    pipe.classList.toggle('flow', flowing);
    if (flowing) pipe.style.animationDuration = `${Math.max(0.15, 1.2 / Math.max(0.1, speed))}s`;
  }

  wire(d, color = '#8c96a0', width = 2) {
    return el('path', { d, fill: 'none', stroke: color, 'stroke-width': width, 'stroke-linecap': 'round', 'stroke-linejoin': 'round', opacity: 0.8 }, this.wireLayer);
  }

  label(parent, x, y, text, cls = 'svg-small', anchor = 'start') {
    const t = el('text', { x, y, class: cls, 'text-anchor': anchor }, parent);
    t.textContent = text;
    return t;
  }

  build() {
    const svg = this.svg;
    const defs = el('defs', {}, svg);
    defs.innerHTML = `
      <linearGradient id="tankGlass" x1="0" x2="1"><stop offset="0" stop-color="#ffffff" stop-opacity="0.10"/><stop offset="0.5" stop-color="#ffffff" stop-opacity="0.03"/><stop offset="1" stop-color="#ffffff" stop-opacity="0.12"/></linearGradient>
      <linearGradient id="heatsink" x1="0" x2="1"><stop offset="0" stop-color="#2b2b2b"/><stop offset="1" stop-color="#4a4a4a"/></linearGradient>
      <filter id="ledGlow" x="-200%" y="-200%" width="500%" height="500%"><feGaussianBlur stdDeviation="3"/></filter>
      <clipPath id="mixerClip"><rect x="482" y="282" width="216" height="316" rx="6"/></clipPath>
      <clipPath id="plantClip"><rect x="862" y="392" width="296" height="246" rx="6"/></clipPath>`;
    this.wireLayer = el('g', {}, svg);
    this.pipeLayer = el('g', {}, svg);
    const tanks = el('g', {}, svg);
    const parts = el('g', {}, svg);

    // BOTTLES (fixed positions; tubes follow the dosing tube wiring parameter)
    this.bottles = [];
    const bottleX = [70, 190, 310];
    ['pH Down', 'Nutrient A', 'Nutrient B'].forEach((name, i) => {
      const g = el('g', { transform: `translate(${bottleX[i]},40)`, class: 'clickable' }, parts);
      el('title', {}, g).textContent = `${name}: click to refill`;
      el('rect', { x: -10, y: -14, width: 20, height: 14, rx: 2, fill: '#3a3f44' }, g);
      el('path', { d: 'M-34,10 Q-34,0 -22,0 L22,0 Q34,0 34,10 L34,120 Q34,128 26,128 L-26,128 Q-34,128 -34,120 Z', fill: 'rgba(255,255,255,0.06)', stroke: '#9aa3ab', 'stroke-width': 2 }, g);
      const liquid = el('rect', { x: -32, y: 10, width: 64, height: 116, rx: 4, fill: '#888', opacity: 0.85 }, g);
      el('rect', { x: -24, y: 52, width: 48, height: 34, rx: 3, fill: '#f2f2ee' }, g);
      this.label(g, 0, 66, name, 'svg-tiny', 'middle').setAttribute('style', 'fill:#222;font-weight:600');
      const amount = this.label(g, 0, 80, '', 'svg-tiny', 'middle');
      amount.setAttribute('style', 'fill:#444');
      g.addEventListener('click', () => this.send({ type: 'world', action: 'refill', bottle: i }));
      this.bottles.push({ liquid, amount, x: bottleX[i] });
    });

    // PERISTALTIC PUMPS P1..P4 (L298N #1: P1 = OUT1/2, P2 = OUT3/4; L298N #2: P3, P4)
    this.peristaltic = [];
    const pumpX = [60, 160, 270, 370];
    for (let i = 0; i < 4; ++i) {
      const x = pumpX[i], y = 238;
      const g = el('g', { transform: `translate(${x},${y})` }, parts);
      el('rect', { x: -30, y: -26, width: 60, height: 52, rx: 6, fill: '#1d2023', stroke: '#454a50' }, g);
      el('circle', { r: 21, fill: '#e9edf0', 'fill-opacity': 0.18, stroke: '#9aa3ab', 'stroke-width': 1.5 }, g);
      const rotor = el('g', { class: 'rotor' }, g);
      el('circle', { r: 5, fill: '#cfd4d8' }, rotor);
      for (let k = 0; k < 3; ++k) {
        const a = (k * 2 * Math.PI) / 3;
        el('line', { x1: 0, y1: 0, x2: Math.cos(a) * 13, y2: Math.sin(a) * 13, stroke: '#cfd4d8', 'stroke-width': 3 }, rotor);
        el('circle', { cx: Math.cos(a) * 13, cy: Math.sin(a) * 13, r: 4.5, fill: '#f4f6f7' }, rotor);
      }
      const name = this.label(g, 0, 42, `P${i + 1}`, 'svg-label', 'middle');
      const sub = this.label(g, 0, 56, '', 'svg-tiny', 'middle');
      this.peristaltic.push({ rotor, name, sub, x, y });
    }
    // Tubes from the pumps to the mixer reservoir
    for (let i = 0; i < 4; ++i) {
      const x = pumpX[i];
      const inletX = 530 + i * 14;
      this.pipe(`dose${i}`, `M${x + 30},${238} L${x + 44},${238} Q${x + 52},${238} ${x + 52},${230} L${x + 52},${190 - i * 6} L${inletX},${190 - i * 6} L${inletX},${300}`, 6);
    }
    this.drips = [];
    for (let i = 0; i < 4; ++i) {
      const drip = el('circle', { cx: 530 + i * 14, cy: 306, r: 3, fill: '#fff', opacity: 0 }, parts);
      this.drips.push(drip);
    }

    // L298N MOTOR DRIVERS
    this.l298n = [this.drawL298n(parts, 40, 320, 'L298N #1', 'JP3 · nutrients'), this.drawL298n(parts, 250, 320, 'L298N #2', 'JP4 · acid')];
    // Motor wires (red/black pairs) from the drivers' outputs to the pumps
    for (let i = 0; i < 4; ++i) {
      const driver = i < 2 ? 0 : 1;
      const fromX = (driver === 0 ? 40 : 250) + (i % 2 === 0 ? 18 : 132);
      this.peristaltic[i].wire = this.wire(`M${fromX},320 C${fromX},300 ${pumpX[i]},300 ${pumpX[i]},264`, '#c0392b', 2.5);
    }

    // MIXER RESERVOIR
    this.mixer = this.drawTank(tanks, 480, 280, 220, 320, 'mixerClip', 'Mixer reservoir');
    // PLANT RESERVOIR
    this.plant = this.drawTank(tanks, 860, 390, 300, 250, 'plantClip', 'Plant reservoir');
    this.drawPlants(parts, 860, 390, 300);

    // SUBMERSIBLE PUMP (output) at the bottom of the mixer
    const sub = el('g', { transform: 'translate(520,568)' }, parts);
    el('rect', { x: -26, y: -22, width: 52, height: 44, rx: 8, fill: '#1f2326', stroke: '#5b636b' }, sub);
    this.submersibleImpeller = el('g', {}, sub);
    for (let k = 0; k < 4; ++k) el('rect', { x: -2, y: -14, width: 4, height: 14, rx: 2, fill: '#9aa3ab', transform: `rotate(${k * 90})` }, this.submersibleImpeller);
    this.label(sub, 0, -28, 'submersible', 'svg-tiny', 'middle');
    // SELF-PRIMING PUMP (input) between the reservoirs
    const sp = el('g', { transform: 'translate(800,700)' }, parts);
    el('rect', { x: -42, y: -26, width: 84, height: 52, rx: 10, fill: '#22262a', stroke: '#5b636b' }, sp);
    el('rect', { x: -60, y: -12, width: 18, height: 24, rx: 3, fill: '#3a4046' }, sp);
    this.selfPrimingImpeller = el('g', { transform: 'translate(10,0)' }, sp);
    el('circle', { r: 15, fill: '#2f353a', stroke: '#9aa3ab' }, this.selfPrimingImpeller);
    for (let k = 0; k < 5; ++k) el('rect', { x: -1.5, y: -13, width: 3, height: 13, fill: '#9aa3ab', transform: `rotate(${k * 72})` }, this.selfPrimingImpeller);
    this.label(sp, 0, 44, 'self-priming pump', 'svg-tiny', 'middle');
    this.selfPrimingStatus = this.label(sp, 0, 57, '', 'svg-tiny', 'middle');

    // Water pipes: mixer -> plant (output) and plant -> mixer (input)
    this.pipe('output', 'M520,546 L520,232 L1010,232 L1010,400', 11);
    this.pipe('inputIntake', 'M920,620 L920,350 Q920,334 904,334 L856,334 Q840,334 840,350 L840,688', 11);
    this.pipe('inputOutlet', 'M780,690 L740,690 L740,262 L660,262 L660,300', 11);
    // Hose end in the plant reservoir
    this.hoseEnd = el('rect', { x: 913, y: 600, width: 14, height: 22, rx: 3, fill: '#59616a' }, parts);

    // FLOAT SWITCHES
    this.mixerFloat = this.drawFloat(parts, 680, this.mixer);
    this.plantFloat = this.drawFloat(parts, 1135, this.plant);

    // PROBES in the mixer (or elsewhere, see parameters)
    this.phProbe = this.drawProbe(parts, 584, 'pH');
    this.tdsProbe = this.drawProbe(parts, 624, 'TDS');

    // SENSOR BOARDS
    this.phBoard = this.drawSensorBoard(parts, 300, 418, 'PH-4502C', 'pH board · JP6', '#1f5fa8');
    this.tdsBoard = this.drawSensorBoard(parts, 300, 528, 'TDS Meter V1.0', 'Keyestudio · JP5', '#1c1c1c');
    this.wire('M440,448 C520,448 520,430 584,430 L584,446', '#b0b7be', 2.5);
    this.wire('M440,558 C540,558 560,420 624,420 L624,446', '#b0b7be', 2.5);

    // RELAYS
    this.relay1 = this.drawRelay(parts, 330, 662, 'Relay 1 · GPIO27', 'DR21A01 · JP1 → output');
    this.relay2 = this.drawRelay(parts, 600, 708, 'Relay 2 · GPIO26', 'DR21A01 · JP2 → input');
    this.relay1.wire = this.wire('M440,682 C470,682 470,600 496,590', '#c0392b', 3);
    this.relay2.wire = this.wire('M710,730 C740,730 740,712 758,706', '#c0392b', 3);

    // MIXER BOARD with the T-Display
    this.drawMixerBoard(parts, 30, 640);
    // Ribbon cables from the mixer board
    const ribbon = '#7d8790';
    this.wire('M120,640 C120,560 80,520 90,410', ribbon, 3);
    this.wire('M160,640 C160,540 300,520 300,410', ribbon, 3);
    this.wire('M290,697 C310,697 310,697 330,697', ribbon, 3);
    this.wire('M290,740 C450,760 560,745 600,745', ribbon, 3);
    this.wire('M240,640 C260,600 270,500 300,470', ribbon, 3);
    this.wire('M260,640 C280,620 280,600 300,585', ribbon, 3);
    // Float switch cables
    this.wire('M680,300 C690,250 720,620 290,770', '#d0d4d8', 1.5);
    this.wire('M1135,400 C1180,380 1190,790 290,780', '#d0d4d8', 1.5);

    // 12 V supply
    const psu = el('g', { transform: 'translate(330,750)' }, parts);
    el('rect', { x: 0, y: 0, width: 110, height: 44, rx: 5, fill: '#2b2f33', stroke: '#5b636b' }, psu);
    this.label(psu, 55, 19, '12 V supply', 'svg-small', 'middle');
    this.psuLabel = this.label(psu, 55, 34, 'J1', 'svg-tiny', 'middle');
  }

  drawTank(parent, x, y, width, height, clip, name) {
    const g = el('g', {}, parent);
    const clipped = el('g', { 'clip-path': `url(#${clip})` }, g);
    const water = el('rect', { x, y: y + height, width, height: 0, fill: '#3f93dc', opacity: 0.75 }, clipped);
    const surface = el('path', { d: '', fill: 'none', stroke: '#ffffff', 'stroke-opacity': 0.5, 'stroke-width': 2 }, clipped);
    el('rect', { x, y, width, height, rx: 8, fill: 'url(#tankGlass)', stroke: '#a7b1ba', 'stroke-width': 3 }, g);
    // Scale
    for (let i = 1; i < 10; ++i) {
      const ty = y + height - (height * i) / 10;
      el('line', { x1: x + 4, y1: ty, x2: x + (i % 5 === 0 ? 18 : 11), y2: ty, stroke: '#a7b1ba', 'stroke-opacity': 0.6 }, g);
    }
    // Labels below the tank, clear of the pipes
    this.label(g, x, y + height + 22, name, 'svg-label');
    const volume = this.label(g, x + width, y + height + 22, '', 'svg-value', 'end');
    const chemistry = this.label(g, x, y + height + 40, '', 'svg-small');
    const flows = this.label(g, x, y + height + 56, '', 'svg-small');
    const spill = el('path', { d: `M${x + width},${y + height + 4} q20,8 50,4 q20,-4 40,6`, fill: 'none', stroke: '#3f93dc', 'stroke-width': 6, 'stroke-linecap': 'round', opacity: 0 }, g);
    return { x, y, width, height, water, surface, volume, chemistry, flows, spill };
  }

  drawPlants(parent, x, y, width) {
    const g = el('g', {}, parent);
    el('rect', { x: x - 6, y: y - 14, width: width + 12, height: 14, rx: 3, fill: '#3a4148', stroke: '#59626b' }, g);
    for (let i = 0; i < 4; ++i) {
      const cx = x + 45 + i * 70;
      el('path', { d: `M${cx - 14},${y - 14} L${cx - 10},${y + 12} L${cx + 10},${y + 12} L${cx + 14},${y - 14} Z`, fill: '#202427', stroke: '#454c52' }, g);
      for (let k = 0; k < 4; ++k) el('line', { x1: cx - 6 + k * 4, y1: y + 12, x2: cx - 10 + k * 7, y2: y + 40, stroke: '#e8e0c8', 'stroke-opacity': 0.55 }, g);
      const leaf = (dx, dy, rotate, color) =>
        el('ellipse', { cx: cx + dx, cy: y - 26 + dy, rx: 17, ry: 9, fill: color, transform: `rotate(${rotate} ${cx + dx} ${y - 26 + dy})` }, g);
      leaf(-12, 0, -30, '#3e9e45');
      leaf(12, 0, 30, '#47b04f');
      leaf(0, -10, 90, '#5cc363');
      leaf(-6, -4, -70, '#4fb456');
    }
  }

  drawFloat(parent, x, tank) {
    const g = el('g', {}, parent);
    el('line', { x1: x, y1: tank.y - 4, x2: x, y2: tank.y + tank.height * 0.5, stroke: '#c4cbd1', 'stroke-width': 3 }, g);
    const body = el('rect', { x: x - 9, y: 0, width: 18, height: 26, rx: 7, fill: '#e8ecef', stroke: '#6c757d' }, g);
    const label = this.label(g, x + 14, 0, '', 'svg-tiny');
    el('title', {}, g).textContent = 'Float switch: pulls its input low when the water lifts it';
    return { body, label, tank, x };
  }

  drawProbe(parent, x, name) {
    const g = el('g', {}, parent);
    const body = el('g', {}, g);
    el('rect', { x: x - 6, y: 446, width: 12, height: 110, rx: 5, fill: name === 'pH' ? '#d8dde2' : '#1e2225', stroke: '#6c757d' }, body);
    if (name === 'pH') el('circle', { cx: x, cy: 556, r: 7, fill: '#e9f3f8', stroke: '#8aa' }, body);
    else {
      el('line', { x1: x - 3, y1: 556, x2: x - 3, y2: 568, stroke: '#c9b26b', 'stroke-width': 2 }, body);
      el('line', { x1: x + 3, y1: 556, x2: x + 3, y2: 568, stroke: '#c9b26b', 'stroke-width': 2 }, body);
    }
    this.label(g, x, 440, name, 'svg-tiny', 'middle');
    return { body, x };
  }

  drawSensorBoard(parent, x, y, name, sub, color) {
    const g = el('g', { transform: `translate(${x},${y})` }, parent);
    el('rect', { x: 0, y: 0, width: 140, height: 70, rx: 4, fill: color, stroke: '#0d2240' }, g);
    if (name.startsWith('PH')) {
      el('circle', { cx: 112, cy: 35, r: 17, fill: 'url(#heatsink)', stroke: '#bbb' }, g);
      el('circle', { cx: 112, cy: 35, r: 7, fill: '#d9d9d9' }, g);
      for (const px of [40, 66]) {
        el('rect', { x: px - 9, y: 22, width: 18, height: 18, fill: '#2a62c9', stroke: '#9cc' }, g);
        el('circle', { cx: px, cy: 31, r: 5, fill: '#e8c66a' }, g);
      }
    } else {
      el('rect', { x: 92, y: 18, width: 34, height: 26, rx: 2, fill: '#f2f2f2' }, g);
      el('rect', { x: 30, y: 26, width: 30, height: 18, fill: '#111' }, g);
    }
    const led = el('circle', { cx: 12, cy: 12, r: 4, fill: '#333' }, g);
    const glow = el('circle', { cx: 12, cy: 12, r: 8, fill: '#ff4433', filter: 'url(#ledGlow)', opacity: 0 }, g);
    this.label(g, 22, 16, name, 'svg-tiny').setAttribute('style', 'fill:#fff;font-weight:600');
    this.label(g, 0, 84, sub, 'svg-tiny');
    const reading = this.label(g, 0, 98, '', 'svg-small');
    return { led, glow, reading };
  }

  drawRelay(parent, x, y, name, sub) {
    const g = el('g', { transform: `translate(${x},${y})` }, parent);
    el('rect', { x: 0, y: 0, width: 110, height: 60, rx: 4, fill: '#1a4fa3', stroke: '#0b2c63' }, g);
    el('rect', { x: 10, y: 10, width: 46, height: 38, rx: 2, fill: '#2f6fd6', stroke: '#173f86' }, g);
    this.label(g, 33, 33, 'SRD', 'svg-tiny', 'middle').setAttribute('style', 'fill:#e6eefc');
    el('rect', { x: 66, y: 14, width: 34, height: 30, rx: 2, fill: '#1f8a4c' }, g);
    for (let k = 0; k < 3; ++k) el('circle', { cx: 73 + k * 10, cy: 29, r: 3.2, fill: '#c9c9c9' }, g);
    const contact = el('line', { x1: 73, y1: 40, x2: 83, y2: 46, stroke: '#ffd479', 'stroke-width': 2.5 }, g);
    const led = el('circle', { cx: 10, cy: 54, r: 3.5, fill: '#333' }, g);
    const glow = el('circle', { cx: 10, cy: 54, r: 8, fill: '#ff4433', filter: 'url(#ledGlow)', opacity: 0 }, g);
    this.label(g, 0, -6, name, 'svg-small');
    this.label(g, 0, 74, sub, 'svg-tiny');
    return { led, glow, contact };
  }

  drawL298n(parent, x, y, name, sub) {
    const g = el('g', { transform: `translate(${x},${y})` }, parent);
    el('rect', { x: 0, y: 0, width: 150, height: 92, rx: 4, fill: '#b3261e', stroke: '#6e130e' }, g);
    el('rect', { x: 46, y: 6, width: 58, height: 52, rx: 2, fill: 'url(#heatsink)' }, g);
    for (let k = 0; k < 7; ++k) el('line', { x1: 50 + k * 8, y1: 8, x2: 50 + k * 8, y2: 56, stroke: '#1b1b1b' }, g);
    el('rect', { x: 4, y: 8, width: 30, height: 22, rx: 2, fill: '#1e66c4' }, g);
    el('rect', { x: 116, y: 8, width: 30, height: 22, rx: 2, fill: '#1e66c4' }, g);
    for (let k = 0; k < 6; ++k) el('rect', { x: 44 + k * 10, y: 72, width: 6, height: 10, fill: '#111' }, g);
    const channelA = this.label(g, 19, 46, 'A', 'svg-small', 'middle');
    const channelB = this.label(g, 131, 46, 'B', 'svg-small', 'middle');
    channelA.setAttribute('style', 'fill:#fff;font-weight:700');
    channelB.setAttribute('style', 'fill:#fff;font-weight:700');
    const led = el('circle', { cx: 140, cy: 80, r: 3, fill: '#ff5a4f' }, g);
    this.label(g, 0, 106, `${name} · ${sub}`, 'svg-tiny');
    return { channelA, channelB, led };
  }

  drawMixerBoard(parent, x, y) {
    const g = el('g', { transform: `translate(${x},${y})` }, parent);
    el('rect', { x: 0, y: 0, width: 260, height: 150, rx: 6, fill: '#14532f', stroke: '#0b3a20' }, g);
    // T-Display (live screen)
    el('rect', { x: 46, y: 14, width: 168, height: 82, rx: 4, fill: '#111316' }, g);
    this.screenImage = el('image', { x: 62, y: 22, width: 136, height: 76.5, preserveAspectRatio: 'none', style: 'image-rendering:pixelated' }, g);
    // Debug LEDs D4..D9 (shift register outputs) and LED1..3
    this.boardLeds = {};
    const leds = [['D4', 'P1'], ['D5', 'P2'], ['D6', 'P3'], ['D7', 'P4'], ['D8', 'A'], ['D9', 'B'], ['LED1', 'ERR'], ['LED3', 'pH'], ['LED2', 'TDS']];
    leds.forEach(([id, text], i) => {
      const lx = 18 + i * 27;
      const color = id === 'LED1' ? '#ff3b30' : id.startsWith('LED') ? '#4cd964' : '#ffb340';
      const glow = el('circle', { cx: lx, cy: 116, r: 8, fill: color, filter: 'url(#ledGlow)', opacity: 0 }, g);
      const body = el('rect', { x: lx - 5, y: 112, width: 10, height: 7, rx: 1, fill: '#3a3a3a' }, g);
      this.label(g, lx, 134, text, 'svg-tiny', 'middle').setAttribute('style', 'fill:#cfe8d5');
      this.boardLeds[id] = { glow, body, color };
    });
    this.label(g, 0, 166, 'Mixer board · 74HC595 · T-Display ESP32', 'svg-tiny');
  }

  update(state) {
    const world = state.world;
    const board = state.board;

    // Bottles
    world.bottles.forEach((bottle, i) => {
      const b = this.bottles[i];
      const fraction = Math.max(0, Math.min(1, bottle.ml / bottle.capacity));
      b.liquid.setAttribute('height', 116 * fraction);
      b.liquid.setAttribute('y', 10 + 116 * (1 - fraction));
      b.liquid.setAttribute('fill', BOTTLE_COLORS[bottle.contents]);
      b.amount.textContent = `${Math.round(bottle.ml)} mL`;
    });

    // Peristaltic pumps and their tubes
    const pumpBottleNames = ['pH Down', 'Nutrient A', 'Nutrient B'];
    world.peristaltic.forEach((pump, i) => {
      const p = this.peristaltic[i];
      p.rotor.classList.toggle('spin', pump.on);
      p.rotor.classList.toggle('reverse', pump.direction < 0);
      const contents = pump.bottle >= 0 ? world.bottles[pump.bottle].contents : null;
      p.sub.textContent = pump.bottle >= 0 ? pumpBottleNames[pump.bottle] : 'spare';
      p.wire.setAttribute('stroke-width', pump.on ? 3.5 : 2);
      p.wire.setAttribute('opacity', pump.on ? 1 : 0.45);
      const color = contents ? BOTTLE_COLORS[contents] : '#888';
      this.setFlow(`dose${i}`, pump.flow > 0, color, 1);
      this.drips[i].setAttribute('fill', color);
      this.drips[i].classList.toggle('drip', pump.flow > 0);
      this.drips[i].setAttribute('opacity', pump.flow > 0 ? 1 : 0);
      // Route the tube from its bottle
      this.routeBottleTube(i, pump.bottle, color);
    });

    // L298N channels
    board.l298n.forEach((driver, i) => {
      const show = (label, name, out) => {
        label.textContent = name + (out > 0 ? ' ▶' : out < 0 ? ' ◀' : '');
        label.setAttribute('style', `fill:${out ? '#ffe08a' : '#fff'};font-weight:700`);
      };
      show(this.l298n[i].channelA, 'A', driver.outA);
      show(this.l298n[i].channelB, 'B', driver.outB);
    });

    // Tanks
    const drawTank = (tank, t) => {
      const level = Math.max(0, Math.min(1, t.level));
      const top = tank.y + tank.height * (1 - level - (t.ripple || 0));
      tank.water.setAttribute('y', top);
      tank.water.setAttribute('height', tank.y + tank.height - top);
      tank.water.setAttribute('fill', waterColor(t.tds));
      const wave = t.inflow > 0 ? 3 : 0.5;
      const phase = (performance.now() / 300) % (2 * Math.PI);
      let d = `M${tank.x},${top}`;
      for (let k = 0; k <= 12; ++k) {
        const x = tank.x + (tank.width * k) / 12;
        d += ` L${x},${top + Math.sin(phase + k) * wave}`;
      }
      tank.surface.setAttribute('d', level > 0.001 ? d : '');
      tank.volume.textContent = `${fmt(t.volume)} / ${fmt(t.capacity, 0)} L`;
      tank.chemistry.textContent = `TDS ${fmt(t.tds, 0)} ppm · pH ${fmt(t.ph, 2)}`;
      tank.flows.textContent = [t.inflow > 0 ? `in ${fmt(t.inflow)} L/min` : '', t.outflow > 0 ? `out ${fmt(t.outflow)} L/min` : ''].filter(Boolean).join(' · ');
      tank.spill.setAttribute('opacity', t.spilled > 0 ? 0.9 : 0);
    };
    drawTank(this.mixer, world.mixer);
    drawTank(this.plant, world.plant);

    // Floats: raised when the water lifts them to the switch point
    const drawFloat = (float, t) => {
      const switchY = float.tank.y + float.tank.height * (1 - t.floatHeight);
      const waterY = float.tank.y + float.tank.height * (1 - t.level);
      const y = Math.max(switchY - 8, Math.min(switchY + 20, waterY)) - 13;
      float.body.setAttribute('y', y);
      float.body.setAttribute('fill', t.floatClosed ? '#e5534b' : '#e8ecef');
      float.label.setAttribute('y', switchY + 4);
      float.label.textContent = t.floatClosed ? 'closed' : '';
      float.label.setAttribute('style', 'fill:#ff8f86;font-weight:600');
    };
    drawFloat(this.mixerFloat, world.mixer);
    drawFloat(this.plantFloat, world.plant);

    // Main pumps
    const mixColor = waterColor(world.mixer.tds);
    const plantColor = waterColor(world.plant.tds);
    this.submersibleImpeller.classList.toggle('spin', world.outputPump.on);
    this.selfPrimingImpeller.classList.toggle('spin', world.inputPump.on);
    this.setFlow('output', world.outputPump.flow > 0, mixColor, world.outputPump.flow / 3);
    const inputFlowing = world.inputPump.flow > 0;
    const priming = world.inputPump.on && !inputFlowing && !world.inputPump.dry;
    this.setFlow('inputIntake', inputFlowing || priming, plantColor, inputFlowing ? world.inputPump.flow / 3 : 0.3);
    this.setFlow('inputOutlet', inputFlowing, plantColor, world.inputPump.flow / 3);
    this.selfPrimingStatus.textContent = world.inputPump.on
      ? inputFlowing ? `${fmt(world.inputPump.flow)} L/min` : world.inputPump.dry ? 'sucking air' : `priming ${fmt(world.inputPump.primed)} s`
      : '';
    const hoseLevel = state.params['plant.hoseIntake'] / world.plant.capacity;
    this.hoseEnd.setAttribute('y', this.plant.y + this.plant.height * (1 - hoseLevel) - 11);

    // Relays
    const drawRelay = (relay, on) => {
      relay.led.setAttribute('fill', on ? '#ff4433' : '#333');
      relay.glow.setAttribute('opacity', on ? 0.9 : 0);
      relay.contact.setAttribute('x2', on ? 93 : 83);
      relay.contact.setAttribute('y2', on ? 40 : 46);
      relay.wire.setAttribute('stroke-width', on ? 4 : 2.5);
      relay.wire.setAttribute('opacity', on ? 1 : 0.4);
    };
    drawRelay(this.relay1, board.relays[0]);
    drawRelay(this.relay2, board.relays[1]);

    // Sensor boards and probes
    const drawSensor = (sensor, powered, text) => {
      sensor.led.setAttribute('fill', powered ? '#ff4433' : '#333');
      sensor.glow.setAttribute('opacity', powered ? 0.9 : 0);
      sensor.reading.textContent = text;
    };
    const phLocation = ['', '', 'in pH 4.00 buffer', 'in pH 7.00 buffer', 'in pH 10.00 buffer', 'in air'];
    const tdsLocation = ['', '', 'in 707 ppm standard', 'in distilled water', 'in air'];
    drawSensor(this.phBoard, board.phBoard.powered, `probe: pH ${fmt(world.phProbe.ph, 2)} ${phLocation[world.phProbe.location] || ''}`);
    drawSensor(this.tdsBoard, board.tdsBoard.powered, `probe: ${fmt(world.tdsProbe.tds, 0)} ppm ${tdsLocation[world.tdsProbe.location] || ''}`);
    const placeProbe = (probe, location, inTank) => {
      // Probes in the mixer hang in it; anywhere else they are lifted out
      probe.body.setAttribute('transform', location === 0 ? '' : 'translate(0,-120)');
      probe.body.setAttribute('opacity', inTank ? 1 : 0.6);
    };
    placeProbe(this.phProbe, world.phProbe.location, world.phProbe.location === 0);
    placeProbe(this.tdsProbe, world.tdsProbe.location, world.tdsProbe.location === 0);

    // Mixer board LEDs
    for (const [id, led] of Object.entries(this.boardLeds)) {
      const on = !!board.leds[id];
      led.glow.setAttribute('opacity', on ? 0.9 : 0);
      led.body.setAttribute('fill', on ? led.color : '#3a3a3a');
    }
    this.psuLabel.textContent = state.params['fault.pumpSupply'] === 1 ? 'OFF' : 'J1 · on';
  }

  // The tube from a bottle to the pump it is connected to
  routeBottleTube(pump, bottle, color) {
    const name = `bottle${pump}`;
    if (!this.pipes[name]) {
      this.pipe(name, 'M0,0', 6);
    }
    const pipe = this.pipes[name];
    const tube = pipe.previousSibling;
    if (bottle < 0) {
      pipe.setAttribute('d', '');
      tube.setAttribute('d', '');
      return;
    }
    const from = this.bottles[bottle];
    const to = this.peristaltic[pump];
    const d = `M${from.x},170 C${from.x},196 ${to.x - 30},${to.y - 30} ${to.x - 22},${to.y - 14}`;
    pipe.setAttribute('d', d);
    tube.setAttribute('d', d);
    const flowing = this.pipes[`dose${pump}`].classList.contains('flow');
    pipe.setAttribute('stroke', color);
    pipe.classList.toggle('flow', flowing);
  }

  setScreen(dataUrl) {
    this.screenImage.setAttribute('href', dataUrl);
  }
}

// The mixer board rendered from the EAGLE design (eagle/untitled.brd) with the external modules it connects
// to: T-Display, two L298N motor drivers, two DR21A01 relay boards, the PH-4502C pH board and the Keyestudio
// TDS meter. Nets show their live logic levels and the LEDs light up like on the real board.
import { svgElement as el } from './display.js';

const COLORS = {
  mask: '#175e35',
  copper: '#2c8c55',
  bottomCopper: '#0f4227',
  pad: '#d6b75e',
  hole: '#0b0d0c',
  silk: '#f1f1ec',
  high: '#ffb340',
  power: '#ff6b5b',
};

// Live level of a net from the simulator state: GPIO nets from the pin levels, the 74HC595 outputs from the
// shift register. Returns 1 (high), 0 (low), 'power' or null (no logic level).
const GPIO_NETS = {
  RELAY1: 27, RELAY2: 26, SR_SER: 2, SR_SCK: 15, SR_RCK: 33, SR_SCL: 25, ENC_A: 17, ENC_B: 22, ENC_SW: 21,
  ABORT_SW: 13, LED_ERROR: 12, FLOAT_RES: 37, FLOAT_MIXER: 38,
};
const REGISTER_NETS = { PH_ON: 0, PPUMP1_EN: 1, PPUMP2_EN: 2, PPUMP4_EN: 3, PPUMP_B: 4, PPUMP_A: 5, PPUMP3_EN: 6, TDS_ON: 7 };
const ANALOG_NETS = { PH3V: 36, TDS3V: 32, TEMP3V: 39 };
const POWER_NETS = ['12V', 'VCC', '3V3'];

function netLevel(net, state) {
  if (!state) return null;
  if (net in GPIO_NETS) return state.board.pins[GPIO_NETS[net]]?.level ?? null;
  if (net in REGISTER_NETS) return (state.board.shiftRegister >> REGISTER_NETS[net]) & 1;
  if (POWER_NETS.includes(net)) return 'power';
  return null;
}

function parseRotation(rot) {
  rot = rot || 'R0';
  return { mirror: rot.includes('M'), angle: parseFloat(rot.replace(/[^0-9.]/g, '')) || 0 };
}

function number(element, name, fallback = 0) {
  const value = element.getAttribute(name);
  return value === null ? fallback : parseFloat(value);
}

export class BoardView {
  constructor(container, hoverLabel, frontPanel) {
    this.container = container;
    this.hoverLabel = hoverLabel;
    this.frontPanel = frontPanel;
    this.state = null;
    this.netOverlays = new Map();  // net -> elements to highlight
    this.leds = {};
    this.svg = el('svg', { viewBox: '-72 -112 246 176', role: 'img', 'aria-label': 'Mixer board and modules' }, container);
    this.load();
    for (const id of ['showModules', 'showTDisplay', 'showBottom', 'showNets']) {
      document.getElementById(id).addEventListener('change', () => this.applyToggles());
    }
  }

  async load() {
    try {
      const response = await fetch('/eagle/untitled.brd');
      const text = await response.text();
      this.render(new DOMParser().parseFromString(text, 'application/xml'));
    } catch (error) {
      el('text', { x: 0, y: 0, class: 'svg-label' }, this.svg).textContent = 'Could not load eagle/untitled.brd: ' + error;
    }
  }

  // Board coordinates (mm, y up) to SVG (y down)
  point(x, y) {
    return [x, -y];
  }

  render(doc) {
    const svg = this.svg;
    const defs = el('defs', {}, svg);
    defs.innerHTML = `
      <filter id="boardGlow" x="-200%" y="-200%" width="500%" height="500%"><feGaussianBlur stdDeviation="1.2"/></filter>
      <linearGradient id="moduleMetal" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#e0e3e6"/><stop offset="1" stop-color="#8a9096"/></linearGradient>`;

    // Packages by library and name
    const packages = new Map();
    for (const library of doc.querySelectorAll('libraries > library')) {
      for (const pkg of library.querySelectorAll('packages > package')) packages.set(library.getAttribute('name') + ':' + pkg.getAttribute('name'), pkg);
    }
    // Net of each element pad
    const padNets = new Map();
    for (const signal of doc.querySelectorAll('signals > signal')) {
      for (const ref of signal.querySelectorAll('contactref')) padNets.set(ref.getAttribute('element') + '.' + ref.getAttribute('pad'), signal.getAttribute('name'));
    }

    this.layers = {
      bottom: el('g', { opacity: 0.0 }, svg),
      base: el('g', {}, svg),
      pour: el('g', {}, svg),
      clearance: el('g', {}, svg),
      copper: el('g', {}, svg),
      highlight: el('g', {}, svg),
      pads: el('g', {}, svg),
      silk: el('g', {}, svg),
      live: el('g', {}, svg),
      cables: el('g', {}, svg),
      modules: el('g', {}, svg),
      tdisplay: el('g', {}, svg),
    };
    // The bottom layer is drawn below the board, seen through it when enabled
    svg.insertBefore(this.layers.base, this.layers.bottom);

    // Board outline
    const outline = [];
    for (const wire of doc.querySelectorAll('plain > wire[layer="20"]')) {
      outline.push([number(wire, 'x1'), number(wire, 'y1')]);
    }
    const xs = outline.map((p) => p[0]), ys = outline.map((p) => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    this.boardBox = { minX, maxX, minY, maxY };
    el('rect', { x: minX, y: -maxY, width: maxX - minX, height: maxY - minY, rx: 1.5, fill: COLORS.mask, stroke: '#0c3a20', 'stroke-width': 0.4 }, this.layers.base);

    // Signals: pours, clearances, tracks, vias
    for (const signal of doc.querySelectorAll('signals > signal')) {
      const net = signal.getAttribute('name');
      for (const polygon of signal.querySelectorAll('polygon')) {
        const points = [...polygon.querySelectorAll('vertex')].map((v) => this.point(number(v, 'x'), number(v, 'y')).join(',')).join(' ');
        const layer = polygon.getAttribute('layer');
        el('polygon', { points, fill: layer === '1' ? COLORS.copper : COLORS.bottomCopper, opacity: layer === '1' ? 0.55 : 1 }, layer === '1' ? this.layers.pour : this.layers.bottom);
      }
      for (const wire of signal.querySelectorAll('wire')) {
        const layer = wire.getAttribute('layer');
        const d = this.wirePath(wire);
        const width = number(wire, 'width', 0.25);
        if (layer === '1') {
          if (net !== 'GND') el('path', { d, stroke: COLORS.mask, 'stroke-width': width + 0.8, fill: 'none', 'stroke-linecap': 'round' }, this.layers.clearance);
          const track = el('path', { d, stroke: COLORS.copper, 'stroke-width': width, fill: 'none', 'stroke-linecap': 'round', 'data-net': net }, this.layers.copper);
          this.addHighlight(net, d, width, track);
        } else if (layer === '16') {
          el('path', { d, stroke: '#2a7a4b', 'stroke-width': width, fill: 'none', 'stroke-linecap': 'round', 'data-net': net }, this.layers.bottom);
        }
      }
      for (const via of signal.querySelectorAll('via')) {
        const [x, y] = this.point(number(via, 'x'), number(via, 'y'));
        const drill = number(via, 'drill', 0.6);
        const diameter = number(via, 'diameter', 0) || drill * 1.9;
        if (net !== 'GND') el('circle', { cx: x, cy: y, r: diameter / 2 + 0.4, fill: COLORS.mask }, this.layers.clearance);
        el('circle', { cx: x, cy: y, r: diameter / 2, fill: COLORS.copper, stroke: '#3aa36a', 'stroke-width': 0.1, 'data-net': net }, this.layers.copper);
        el('circle', { cx: x, cy: y, r: drill / 2, fill: COLORS.hole }, this.layers.pads);
      }
    }

    // Elements
    this.elements = new Map();
    for (const element of doc.querySelectorAll('elements > element')) {
      const name = element.getAttribute('name');
      const pkg = packages.get(element.getAttribute('library') + ':' + element.getAttribute('package'));
      const ex = number(element, 'x'), ey = number(element, 'y');
      const { mirror, angle } = parseRotation(element.getAttribute('rot'));
      const transform = (px, py) => {
        if (mirror) px = -px;
        const a = (angle * Math.PI) / 180;
        return [ex + px * Math.cos(a) - py * Math.sin(a), ey + px * Math.sin(a) + py * Math.cos(a)];
      };
      const info = { name, x: ex, y: ey, angle, mirror, pads: new Map() };
      this.elements.set(name, info);
      if (pkg) this.renderPackage(pkg, name, transform, angle, mirror, padNets, info);
      // Smashed name labels
      for (const attribute of element.querySelectorAll('attribute[name="NAME"]')) {
        if (attribute.getAttribute('display') === 'off' || attribute.getAttribute('layer') !== '25') continue;
        this.text(name, number(attribute, 'x'), number(attribute, 'y'), number(attribute, 'size', 1.27), attribute.getAttribute('rot'), attribute.getAttribute('align'), this.layers.silk, 0.75);
      }
    }

    // Board texts on the top silkscreen
    for (const text of doc.querySelectorAll('plain > text')) {
      const layer = text.getAttribute('layer');
      if (layer !== '25' && layer !== '21') continue;
      this.text(text.textContent, number(text, 'x'), number(text, 'y'), number(text, 'size', 1.27), text.getAttribute('rot'), text.getAttribute('align'), this.layers.silk, 1);
    }
    for (const wire of doc.querySelectorAll('plain > wire[layer="21"]')) {
      el('path', { d: this.wirePath(wire), stroke: COLORS.silk, 'stroke-width': number(wire, 'width', 0.15), fill: 'none' }, this.layers.silk);
    }

    this.renderLive();
    this.renderModules();
    this.bindHover();
    this.applyToggles();
    if (this.state) this.update(this.state);
  }

  wirePath(wire) {
    const [x1, y1] = this.point(number(wire, 'x1'), number(wire, 'y1'));
    const [x2, y2] = this.point(number(wire, 'x2'), number(wire, 'y2'));
    const curve = number(wire, 'curve', 0);
    if (!curve) return `M${x1},${y1} L${x2},${y2}`;
    const chord = Math.hypot(x2 - x1, y2 - y1);
    const radius = Math.abs(chord / (2 * Math.sin((curve * Math.PI) / 360)));
    return `M${x1},${y1} A${radius},${radius} 0 ${Math.abs(curve) > 180 ? 1 : 0} ${curve > 0 ? 0 : 1} ${x2},${y2}`;
  }

  addHighlight(net, d, width, source) {
    if (!(net in GPIO_NETS || net in REGISTER_NETS || POWER_NETS.includes(net))) return;
    const overlay = el('path', { d, stroke: COLORS.high, 'stroke-width': width + 0.15, fill: 'none', 'stroke-linecap': 'round', opacity: 0, 'pointer-events': 'none' }, this.layers.highlight);
    if (!this.netOverlays.has(net)) this.netOverlays.set(net, []);
    this.netOverlays.get(net).push(overlay);
  }

  renderPackage(pkg, name, transform, angle, mirror, padNets, info) {
    for (const child of pkg.children) {
      const tag = child.tagName;
      if (tag === 'pad' || tag === 'smd') {
        const [bx, by] = transform(number(child, 'x'), number(child, 'y'));
        const [x, y] = this.point(bx, by);
        const padName = child.getAttribute('name');
        const net = padNets.get(name + '.' + padName) || '';
        const localRotation = parseRotation(child.getAttribute('rot')).angle;
        const rotation = -(angle + localRotation);
        info.pads.set(padName, { x: bx, y: by, net });
        const attributes = { 'data-net': net, 'data-pad': `${name}.${padName}`, fill: COLORS.pad };
        let shape;
        if (tag === 'smd') {
          const dx = number(child, 'dx'), dy = number(child, 'dy');
          const roundness = number(child, 'roundness', 0) / 100;
          if (net !== 'GND') el('rect', { x: x - dx / 2 - 0.35, y: y - dy / 2 - 0.35, width: dx + 0.7, height: dy + 0.7, fill: COLORS.mask, transform: `rotate(${rotation} ${x} ${y})` }, this.layers.clearance);
          shape = el('rect', { ...attributes, x: x - dx / 2, y: y - dy / 2, width: dx, height: dy, rx: (Math.min(dx, dy) / 2) * roundness, transform: `rotate(${rotation} ${x} ${y})` }, this.layers.pads);
        } else {
          const drill = number(child, 'drill', 1);
          const diameter = number(child, 'diameter', 0) || drill * 1.65;
          const kind = child.getAttribute('shape') || 'round';
          const width = kind === 'long' ? diameter * 2 : diameter;
          if (net !== 'GND') el('rect', { x: x - width / 2 - 0.4, y: y - diameter / 2 - 0.4, width: width + 0.8, height: diameter + 0.8, rx: kind === 'square' ? 0.2 : diameter / 2 + 0.4, fill: COLORS.mask, transform: `rotate(${rotation} ${x} ${y})` }, this.layers.clearance);
          shape = el('rect', { ...attributes, x: x - width / 2, y: y - diameter / 2, width, height: diameter, rx: kind === 'square' ? 0.1 : diameter / 2, transform: `rotate(${rotation} ${x} ${y})` }, this.layers.pads);
          el('circle', { cx: x, cy: y, r: drill / 2, fill: COLORS.hole, 'pointer-events': 'none' }, this.layers.pads);
        }
        if (net && (net in GPIO_NETS || net in REGISTER_NETS || POWER_NETS.includes(net))) {
          const overlay = shape.cloneNode();
          overlay.setAttribute('fill', COLORS.high);
          overlay.setAttribute('opacity', 0);
          overlay.setAttribute('pointer-events', 'none');
          overlay.removeAttribute('data-net');
          this.layers.highlight.appendChild(overlay);
          if (!this.netOverlays.has(net)) this.netOverlays.set(net, []);
          this.netOverlays.get(net).push(overlay);
        }
      } else if (child.getAttribute('layer') === '21') {
        if (tag === 'wire') {
          const [x1, y1] = transform(number(child, 'x1'), number(child, 'y1'));
          const [x2, y2] = transform(number(child, 'x2'), number(child, 'y2'));
          const copy = child.cloneNode();
          copy.setAttribute('x1', x1); copy.setAttribute('y1', y1); copy.setAttribute('x2', x2); copy.setAttribute('y2', y2);
          if (mirror && copy.getAttribute('curve')) copy.setAttribute('curve', -number(child, 'curve'));
          el('path', { d: this.wirePath(copy), stroke: COLORS.silk, 'stroke-width': number(child, 'width', 0.15), fill: 'none', 'stroke-linecap': 'round' }, this.layers.silk);
        } else if (tag === 'circle') {
          const [cx, cy] = this.point(...transform(number(child, 'x'), number(child, 'y')));
          const width = number(child, 'width', 0);
          el('circle', { cx, cy, r: number(child, 'radius'), fill: width ? 'none' : COLORS.silk, stroke: COLORS.silk, 'stroke-width': width }, this.layers.silk);
        } else if (tag === 'rectangle') {
          const corners = [
            transform(number(child, 'x1'), number(child, 'y1')), transform(number(child, 'x2'), number(child, 'y1')),
            transform(number(child, 'x2'), number(child, 'y2')), transform(number(child, 'x1'), number(child, 'y2')),
          ].map((p) => this.point(...p).join(','));
          el('polygon', { points: corners.join(' '), fill: COLORS.silk }, this.layers.silk);
        }
      }
    }
  }

  // EAGLE vector text, approximated with a sans-serif font. Text is turned to stay readable like in EAGLE.
  text(content, x, y, size, rot, align, parent, opacity) {
    let { angle } = parseRotation(rot);
    align = align || 'bottom-left';
    let [vertical, horizontal] = align === 'center' ? ['center', 'center'] : align.includes('-') ? align.split('-') : ['bottom', align];
    if (angle > 90 && angle <= 270) {
      angle = (angle + 180) % 360;
      vertical = { top: 'bottom', bottom: 'top', center: 'center' }[vertical];
      horizontal = { left: 'right', right: 'left', center: 'center' }[horizontal];
    }
    const [sx, sy] = this.point(x, y);
    const lines = content.split('\n');
    const lineHeight = size * 1.5;
    const g = el('g', { transform: `translate(${sx},${sy}) rotate(${-angle})`, opacity }, parent);
    const anchor = { left: 'start', center: 'middle', right: 'end' }[horizontal];
    const totalHeight = lineHeight * (lines.length - 1);
    const firstBaseline = vertical === 'top' ? size : vertical === 'center' ? size / 2 - totalHeight / 2 : -totalHeight;
    lines.forEach((line, i) => {
      const t = el('text', { x: 0, y: firstBaseline + i * lineHeight, 'text-anchor': anchor, style: `font: 600 ${size * 1.3}px ui-sans-serif, Helvetica, Arial; fill: ${COLORS.silk}` }, g);
      t.textContent = line;
    });
  }

  padPosition(element, pad) {
    const info = this.elements.get(element);
    const p = info && info.pads.get(String(pad));
    return p ? this.point(p.x, p.y) : [0, 0];
  }

  renderLive() {
    // LEDs
    for (const [id, color] of [['D4', '#ffb340'], ['D5', '#ffb340'], ['D6', '#ffb340'], ['D7', '#ffb340'], ['D8', '#ffb340'], ['D9', '#ffb340'], ['LED1', '#ff3b30'], ['LED2', '#4cd964'], ['LED3', '#4cd964']]) {
      const info = this.elements.get(id);
      if (!info) continue;
      const [x, y] = this.point(info.x, info.y);
      const glow = el('circle', { cx: x, cy: y, r: 2.2, fill: color, filter: 'url(#boardGlow)', opacity: 0, 'pointer-events': 'none' }, this.layers.live);
      const body = el('rect', { x: x - 0.8, y: y - 0.45, width: 1.6, height: 0.9, fill: '#e8e8e0', opacity: 0.9, transform: `rotate(${-info.angle} ${x} ${y})` }, this.layers.live);
      this.leds[id] = { glow, body, color };
    }
    // Encoder knob on SW1 and the cap of the abort switch S1
    const sw1 = this.elements.get('SW1');
    if (sw1) {
      const [x, y] = this.point(sw1.x, sw1.y);
      el('rect', { x: x - 6.2, y: y - 6.7, width: 12.4, height: 13.4, rx: 0.6, fill: '#a9aeb3', stroke: '#6f757a', 'stroke-width': 0.2 }, this.layers.live);
      this.knob = el('g', {}, this.layers.live);
      el('circle', { cx: x, cy: y, r: 7.5, fill: '#25282b', stroke: '#0e0f10', 'stroke-width': 0.4 }, this.knob);
      el('line', { x1: x, y1: y - 2.5, x2: x, y2: y - 6.5, stroke: '#f0f0f0', 'stroke-width': 0.8, 'stroke-linecap': 'round' }, this.knob);
      this.knobCenter = [x, y];
    }
    const s1 = this.elements.get('S1');
    if (s1) {
      const [x, y] = this.point(s1.x, s1.y);
      el('rect', { x: x - 3, y: y - 3, width: 6, height: 6, rx: 0.4, fill: '#cfd2d5', stroke: '#777', 'stroke-width': 0.15 }, this.layers.live);
      this.abortCap = el('circle', { cx: x, cy: y, r: 1.8, fill: '#c0352b' }, this.layers.live);
    }
  }

  // EXTERNAL MODULES

  cable(from, to, color, label) {
    const [x1, y1] = from, [x2, y2] = to;
    const midY = (y1 + y2) / 2;
    const path = el('path', { d: `M${x1},${y1} C${x1},${midY} ${x2},${midY} ${x2},${y2}`, stroke: color, 'stroke-width': 0.55, fill: 'none', opacity: 0.85, 'stroke-linecap': 'round' }, this.layers.cables);
    if (label) el('title', {}, path).textContent = label;
    return path;
  }

  header(parent, x, y, count, pitch = 2.54, vertical = false) {
    const pins = [];
    for (let i = 0; i < count; ++i) {
      const px = vertical ? x : x + i * pitch, py = vertical ? y + i * pitch : y;
      el('rect', { x: px - 1.2, y: py - 1.2, width: 2.4, height: 2.4, fill: '#151515' }, parent);
      el('rect', { x: px - 0.3, y: py - 0.3, width: 0.6, height: 0.6, fill: '#d6b75e' }, parent);
      pins.push([px, py]);
    }
    return pins;
  }

  renderModules() {
    const m = this.layers.modules;
    const label = (x, y, text, size = 2.2, color = '#f4f4f0', anchor = 'start') => {
      const t = el('text', { x, y, 'text-anchor': anchor, style: `font: 600 ${size}px ui-sans-serif, Helvetica, Arial; fill: ${color}` }, m);
      t.textContent = text;
      return t;
    };
    const dupont = ['#e74c3c', '#1c1c1c', '#f1c40f', '#2ecc71', '#3498db', '#9b59b6', '#e67e22', '#ecf0f1'];
    this.modules = {};

    // L298N motor drivers (43 x 43 mm), below the board
    const l298n = (x, y, name, element, pumps) => {
      el('rect', { x, y, width: 43, height: 43, rx: 1, fill: '#b3261e', stroke: '#6e130e', 'stroke-width': 0.3 }, m);
      el('rect', { x: x + 10, y: y + 8, width: 23, height: 25, rx: 0.5, fill: '#2b2b2b' }, m);
      for (let k = 0; k < 9; ++k) el('rect', { x: x + 11 + k * 2.5, y: y + 8, width: 1.2, height: 25, fill: '#444' }, m);
      el('rect', { x: x + 1, y: y + 16, width: 7.5, height: 10, rx: 0.5, fill: '#1e66c4' }, m);
      el('rect', { x: x + 34.5, y: y + 16, width: 7.5, height: 10, rx: 0.5, fill: '#1e66c4' }, m);
      el('rect', { x: x + 6, y: y + 36, width: 11, height: 6.5, rx: 0.5, fill: '#1e66c4' }, m);
      for (let k = 0; k < 3; ++k) el('circle', { cx: x + 8 + k * 3.5, cy: y + 39, r: 1, fill: '#bbb' }, m);
      el('circle', { cx: x + 38, cy: y + 38, r: 3, fill: '#1d1d1d', stroke: '#888', 'stroke-width': 0.3 }, m);
      const pins = this.header(m, x + 21.6, y + 2.5, 6);
      label(x + 21.6, y + 6.8, 'ENA IN1 IN2 IN3 IN4 ENB', 1.15, '#ffd9d6');
      label(x + 1, y + 47, name, 2.4);
      const outA = label(x + 4.7, y + 30, 'OUT1/2', 1.5, '#fff', 'middle');
      const outB = label(x + 38.3, y + 30, 'OUT3/4', 1.5, '#fff', 'middle');
      const led = el('circle', { cx: x + 2.5, cy: y + 33.5, r: 0.8, fill: '#ff3b30' }, m);
      ['ENA', 'IN1', 'IN2', 'IN3', 'IN4', 'ENB'].forEach((signal, i) =>
        this.cable(pins[i], this.padPosition(element, i + 1), dupont[i], `${name} ${signal} ↔ ${element} pin ${i + 1}`));
      this.modules[name] = { outA, outB, pumps };
    };
    l298n(0, 12, 'L298N #1 (nutrients)', 'JP3', ['P1', 'P2']);
    l298n(57, 12, 'L298N #2 (acid)', 'JP4', ['P3', 'P4']);

    // DR21A01 relay boards beside the board
    const relay = (x, y, name, element, gpio) => {
      el('rect', { x, y, width: 44, height: 26, rx: 1, fill: '#1a4fa3', stroke: '#0b2c63', 'stroke-width': 0.3 }, m);
      el('rect', { x: x + 4, y: y + 4, width: 19, height: 15.5, rx: 0.5, fill: '#2f6fd6', stroke: '#173f86', 'stroke-width': 0.3 }, m);
      label(x + 13.5, y + 12.5, 'SRD-05VDC', 1.3, '#e6eefc', 'middle');
      el('rect', { x: x + 27, y: y + 3, width: 15, height: 9, rx: 0.4, fill: '#1e66c4' }, m);
      for (let k = 0; k < 3; ++k) el('circle', { cx: x + 30 + k * 4.5, cy: y + 7.5, r: 1.2, fill: '#ccc' }, m);
      label(x + 34.5, y + 15, 'COM NO NC', 1.1, '#cfe0ff', 'middle');
      const led = el('circle', { cx: x + 4, cy: y + 22.5, r: 1, fill: '#333' }, m);
      const glow = el('circle', { cx: x + 4, cy: y + 22.5, r: 2.5, fill: '#ff3b30', filter: 'url(#boardGlow)', opacity: 0 }, m);
      const pins = this.header(m, x + 28, y + 22.5, 3);
      label(x + 30.5, y + 20.2, 'IN DC+ DC-', 1.1, '#cfe0ff', 'middle');
      label(x, y - 1.5, `${name} · DR21A01 · GPIO${gpio}`, 2.2);
      // JP1/JP2: pin 7 RELAYx, pin 8 VCC, pin 9 GND
      [7, 8, 9].forEach((pin, i) => this.cable(pins[i], this.padPosition(element, pin), [dupont[2], dupont[0], dupont[1]][i], `${name} ↔ ${element} pin ${pin}`));
      return { led, glow };
    };
    this.modules.relay1 = relay(112, -46, 'Relay 1 → output pump', 'JP1', 27);
    this.modules.relay2 = relay(-68, -46, 'Relay 2 → input pump', 'JP2', 26);

    // PH-4502C pH board (42 x 32 mm)
    const ph = (x, y) => {
      el('rect', { x, y, width: 42, height: 32, rx: 1, fill: '#1f5fa8', stroke: '#0d2240', 'stroke-width': 0.3 }, m);
      el('circle', { cx: x + 32, cy: y + 12, r: 6.5, fill: 'url(#moduleMetal)', stroke: '#666', 'stroke-width': 0.3 }, m);
      el('circle', { cx: x + 32, cy: y + 12, r: 2.5, fill: '#f0f0f0' }, m);
      label(x + 32, y + 22.5, 'BNC probe', 1.4, '#dfe9f7', 'middle');
      for (const px of [8, 18]) {
        el('rect', { x: x + px - 3.5, y: y + 6, width: 7, height: 7, fill: '#2a62c9', stroke: '#9cc', 'stroke-width': 0.2 }, m);
        el('circle', { cx: x + px, cy: y + 9.5, r: 2, fill: '#e8c66a' }, m);
      }
      el('rect', { x: x + 6, y: y + 17, width: 8, height: 6, fill: '#111' }, m);
      const led = el('circle', { cx: x + 3, cy: y + 26, r: 1, fill: '#333' }, m);
      const glow = el('circle', { cx: x + 3, cy: y + 26, r: 2.5, fill: '#ff3b30', filter: 'url(#boardGlow)', opacity: 0 }, m);
      const pins = this.header(m, x + 8, y + 29.5, 6);
      label(x + 14.4, y + 27.4, 'To Do Po G G V+', 1.15, '#dfe9f7', 'middle');
      label(x, y - 1.5, 'PH-4502C pH board', 2.2);
      ['TEMP', 'DIGI', 'PH', 'GND', 'GND', 'VCC'].forEach((signal, i) => this.cable(pins[i], this.padPosition('JP6', i + 1), dupont[(i + 3) % 8], `pH ${signal} ↔ JP6 pin ${i + 1}`));
      return { led, glow };
    };
    this.modules.ph = ph(-68, -108);

    // Keyestudio TDS meter V1.0
    const tds = (x, y) => {
      el('rect', { x, y, width: 42, height: 32, rx: 1, fill: '#1b3f73', stroke: '#0d2240', 'stroke-width': 0.3 }, m);
      el('rect', { x: x + 26, y: y + 6, width: 11, height: 8, rx: 0.5, fill: '#f2f2f2' }, m);
      label(x + 31.5, y + 18, 'probe', 1.4, '#dfe9f7', 'middle');
      el('rect', { x: x + 8, y: y + 8, width: 10, height: 7, fill: '#111' }, m);
      const led = el('circle', { cx: x + 3, cy: y + 26, r: 1, fill: '#333' }, m);
      const glow = el('circle', { cx: x + 3, cy: y + 26, r: 2.5, fill: '#ff3b30', filter: 'url(#boardGlow)', opacity: 0 }, m);
      const pins = this.header(m, x + 16, y + 29.5, 3);
      label(x + 18.5, y + 27.4, '−  +  A', 1.2, '#dfe9f7', 'middle');
      label(x, y - 1.5, 'Keyestudio TDS Meter V1.0', 2.2);
      ['GND (switched)', 'VCC', 'TDS'].forEach((signal, i) => this.cable(pins[i], this.padPosition('JP5', i + 1), dupont[i], `TDS ${signal} ↔ JP5 pin ${i + 1}`));
      return { led, glow };
    };
    this.modules.tds = tds(128, -108);

    // T-Display mounted on JP8/JP9 (USB-C at the 5V end of JP8)
    const t = this.layers.tdisplay;
    const [p1x] = this.padPosition('JP8', 1);
    const [, topY] = this.padPosition('JP9', 1);
    const [, bottomY] = this.padPosition('JP8', 1);
    const x0 = p1x - 9, y0 = topY - 1.3, width = 51.5, height = bottomY - topY + 2.6;
    el('rect', { x: x0, y: y0, width, height, rx: 1.2, fill: '#121417', stroke: '#2a2d31', 'stroke-width': 0.3, opacity: 0.97 }, t);
    el('rect', { x: x0 - 2, y: y0 + height / 2 - 4.5, width: 7.5, height: 9, rx: 1.6, fill: 'url(#moduleMetal)' }, t);
    el('rect', { x: x0 + 11, y: y0 + 3.2, width: 33, height: height - 6.4, rx: 0.6, fill: '#050607' }, t);
    this.tdisplayScreen = el('image', { x: x0 + 13.5, y: y0 + 4.5, width: 28, height: 15.75, preserveAspectRatio: 'none', style: 'image-rendering:pixelated' }, t);
    for (let k = 0; k < 2; ++k) {
      el('rect', { x: x0 + 6, y: y0 + 3.5 + k * (height - 9), width: 3.5, height: 3.5, fill: '#c9cdd1' }, t);
    }
    for (const [element, count] of [['JP8', 12], ['JP9', 12]]) {
      for (let i = 1; i <= count; ++i) {
        const [px, py] = this.padPosition(element, i);
        el('circle', { cx: px, cy: py, r: 0.8, fill: '#c9a54a' }, t);
      }
    }
    const tt = el('text', { x: x0 + width - 1, y: y0 + height - 1.2, 'text-anchor': 'end', style: 'font: 600 1.6px ui-sans-serif, Helvetica; fill: #9aa0a6' }, t);
    tt.textContent = 'LILYGO T-Display ESP32';
  }

  bindHover() {
    const descriptions = {
      ...Object.fromEntries(Object.entries(GPIO_NETS).map(([net, pin]) => [net, `GPIO${pin}`])),
      ...Object.fromEntries(Object.entries(REGISTER_NETS).map(([net, bit]) => [net, `74HC595 Q${'ABCDEFGH'[bit]}`])),
      ...Object.fromEntries(Object.entries(ANALOG_NETS).map(([net, pin]) => [net, `GPIO${pin} (ADC)`])),
    };
    this.svg.addEventListener('mousemove', (event) => {
      const target = event.target.closest('[data-net], [data-pad]');
      if (!target) {
        this.hoverLabel.textContent = '';
        return;
      }
      const net = target.getAttribute('data-net') || '';
      const pad = target.getAttribute('data-pad');
      const level = netLevel(net, this.state);
      const levelText = level === 'power' ? 'supply' : level === 1 ? 'HIGH' : level === 0 ? 'LOW' : '';
      this.hoverLabel.textContent = [pad ? `${pad.replace('.', ' pin ')}` : null, net ? `net ${net}` : 'no net', descriptions[net], levelText].filter(Boolean).join(' · ');
    });
  }

  applyToggles() {
    if (!this.layers) return;
    const checked = (id) => document.getElementById(id).checked;
    this.layers.modules.style.display = checked('showModules') ? '' : 'none';
    this.layers.cables.style.display = checked('showModules') ? '' : 'none';
    this.layers.tdisplay.style.display = checked('showTDisplay') ? '' : 'none';
    this.layers.bottom.setAttribute('opacity', checked('showBottom') ? 0.9 : 0);
    this.layers.pour.setAttribute('opacity', checked('showBottom') ? 0.25 : 1);
    this.layers.highlight.style.display = checked('showNets') ? '' : 'none';
  }

  setScreen(url) {
    if (this.tdisplayScreen) this.tdisplayScreen.setAttribute('href', url);
  }

  update(state) {
    this.state = state;
    if (!this.layers) return;
    for (const [net, overlays] of this.netOverlays) {
      const level = netLevel(net, state);
      const opacity = level === 'power' ? 0.35 : level === 1 ? 0.95 : 0;
      const color = level === 'power' ? COLORS.power : COLORS.high;
      for (const overlay of overlays) {
        overlay.setAttribute('opacity', opacity);
        overlay.setAttribute(overlay.tagName === 'path' ? 'stroke' : 'fill', color);
      }
    }
    for (const [id, led] of Object.entries(this.leds)) {
      const on = !!state.board.leds[id];
      led.glow.setAttribute('opacity', on ? 1 : 0);
      led.body.setAttribute('fill', on ? led.color : '#e8e8e0');
    }
    if (this.knob) this.knob.setAttribute('transform', `rotate(${this.frontPanel.detents * 15} ${this.knobCenter[0]} ${this.knobCenter[1]})`);
    if (this.abortCap) this.abortCap.setAttribute('r', state.board.buttons.abort ? 1.4 : 1.8);
    if (this.modules) {
      const showDriver = (module, driver) => {
        const text = (out, base) => base + (out > 0 ? ' ▶' : out < 0 ? ' ◀' : '');
        module.outA.textContent = text(driver.outA, `OUT1/2 ${module.pumps[0]}`);
        module.outB.textContent = text(driver.outB, `OUT3/4 ${module.pumps[1]}`);
        module.outA.style.fill = driver.outA ? '#ffe08a' : '#fff';
        module.outB.style.fill = driver.outB ? '#ffe08a' : '#fff';
      };
      showDriver(this.modules['L298N #1 (nutrients)'], state.board.l298n[0]);
      showDriver(this.modules['L298N #2 (acid)'], state.board.l298n[1]);
      const ledOn = (module, on) => {
        module.led.setAttribute('fill', on ? '#ff3b30' : '#333');
        module.glow.setAttribute('opacity', on ? 1 : 0);
      };
      ledOn(this.modules.relay1, state.board.relays[0]);
      ledOn(this.modules.relay2, state.board.relays[1]);
      ledOn(this.modules.ph, state.board.phBoard.powered);
      ledOn(this.modules.tds, state.board.tdsBoard.powered);
    }
  }
}

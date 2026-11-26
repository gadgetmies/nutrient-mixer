// Front panel: the T-Display (live ST7789 contents and its IO0/IO35 buttons), the EC12E rotary encoder, the
// abort switch S1 and the LEDs of the mixer board.
const SVG = 'http://www.w3.org/2000/svg';

export function svgElement(tag, attributes = {}, parent = null) {
  const element = document.createElementNS(SVG, tag);
  for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, value);
  if (parent) parent.appendChild(element);
  return element;
}

export class FrontPanel {
  constructor(container, send) {
    this.send = send;
    this.detents = 0;
    this.wheelAccumulator = 0;
    this.keysDown = new Set();

    // The display contents, upright as mounted (the firmware draws with rotation 3)
    this.screen = document.createElement('canvas');
    this.screen.width = 240;
    this.screen.height = 135;
    this.screenContext = this.screen.getContext('2d');
    this.image = this.screenContext.createImageData(240, 135);
    this.backlight = false;
    this.screenVersion = 0;

    const body = document.createElement('div');
    body.className = 'front-panel-body';
    container.appendChild(body);
    const svg = svgElement('svg', { viewBox: '0 0 720 340', role: 'img', 'aria-label': 'Front panel' }, body);
    this.svg = svg;
    this.build(svg);
  }

  build(svg) {
    const defs = svgElement('defs', {}, svg);
    defs.innerHTML = `
      <radialGradient id="knobGradient" cx="40%" cy="35%" r="70%"><stop offset="0" stop-color="#5a5f63"/><stop offset="1" stop-color="#1b1d1f"/></radialGradient>
      <radialGradient id="capGradient" cx="40%" cy="35%" r="70%"><stop offset="0" stop-color="#e04a3f"/><stop offset="1" stop-color="#7d1a14"/></radialGradient>
      <linearGradient id="metal" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#d9dcdf"/><stop offset="1" stop-color="#8d9196"/></linearGradient>
      <filter id="glow" x="-100%" y="-100%" width="300%" height="300%"><feGaussianBlur stdDeviation="4"/></filter>
      <pattern id="pcbTexture" width="8" height="8" patternUnits="userSpaceOnUse"><rect width="8" height="8" fill="#14532f"/><circle cx="4" cy="4" r="0.6" fill="#1b6339"/></pattern>`;

    // Mixer board around the T-Display
    svgElement('rect', { x: 0, y: 0, width: 720, height: 340, rx: 14, fill: 'url(#pcbTexture)' }, svg);
    svgElement('text', { x: 590, y: 26, class: 'svg-tiny', fill: '#cfe8d5', style: 'fill:#cfe8d5' }, svg).textContent = 'mixer board';

    // T-DISPLAY
    const td = svgElement('g', { transform: 'translate(16,20)' }, svg);
    svgElement('rect', { x: 0, y: 0, width: 560, height: 300, rx: 10, fill: '#101214', stroke: '#2a2d31' }, td);
    for (let i = 0; i < 12; ++i) {
      for (const y of [12, 288]) {
        svgElement('circle', { cx: 130 + i * 34, cy: y, r: 7, fill: '#c9a54a' }, td);
        svgElement('circle', { cx: 130 + i * 34, cy: y, r: 3.2, fill: '#0b0b0b' }, td);
      }
    }
    const topLabels = ['G', 'G', '21', '22', '17', '2', '15', '13', '12', 'G', 'G', '3V'];
    const bottomLabels = ['5V', 'G', '27', '26', '25', '33', '32', '39', '38', '37', '36', '3V'];
    topLabels.forEach((t, i) => (svgElement('text', { x: 130 + i * 34, y: 31, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#9aa0a6' }, td).textContent = t));
    bottomLabels.forEach((t, i) => (svgElement('text', { x: 130 + i * 34, y: 276, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#9aa0a6' }, td).textContent = t));
    // USB-C
    svgElement('rect', { x: -14, y: 128, width: 34, height: 44, rx: 9, fill: 'url(#metal)', stroke: '#555' }, td);
    svgElement('rect', { x: -8, y: 140, width: 18, height: 20, rx: 5, fill: '#2a2c2f' }, td);
    svgElement('text', { x: 46, y: 154, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#9aa0a6', transform: 'rotate(-90 46 150)' }, td).textContent = 'LILYGO T-Display';

    this.buttonIo35 = this.tactile(td, 52, 62, 'IO35', 'io35');
    this.buttonIo0 = this.tactile(td, 52, 238, 'IO0', 'io0');
    // Reset (EN) button on the side
    const reset = svgElement('g', { class: 'panel-button', transform: 'translate(20,268)' }, td);
    svgElement('rect', { x: -6, y: -6, width: 22, height: 14, rx: 3, fill: '#c8c8c8' }, reset);
    svgElement('text', { x: 5, y: 22, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#9aa0a6' }, reset).textContent = 'RST';
    svgElement('title', {}, reset).textContent = 'Reset (EN): restart the ESP32';
    reset.addEventListener('click', () => this.send({ type: 'device', action: 'reset' }));

    // Screen: glass, then the 240 x 135 active area
    svgElement('rect', { x: 92, y: 40, width: 452, height: 220, rx: 6, fill: '#050607', stroke: '#26292c' }, td);
    // Active area at 1.5 times the panel resolution
    const foreign = svgElement('foreignObject', { x: 138, y: 49, width: 360, height: 202.5 }, td);
    this.screen.style.cssText = 'width:100%;height:100%;image-rendering:pixelated;display:block;background:#000';
    const holder = document.createElement('div');
    holder.style.cssText = 'width:100%;height:100%;';
    holder.appendChild(this.screen);
    foreign.appendChild(holder);
    this.screenOff = svgElement('rect', { x: 138, y: 49, width: 360, height: 202.5, fill: '#000', opacity: 0 }, td);

    // ROTARY ENCODER (SW1, EC12E with push switch)
    const encoder = svgElement('g', { transform: 'translate(650,112)' }, svg);
    svgElement('rect', { x: -40, y: -40, width: 80, height: 80, rx: 4, fill: '#9ea3a8', stroke: '#6b7075' }, encoder);
    this.knob = svgElement('g', { class: 'knob' }, encoder);
    svgElement('circle', { r: 52, fill: 'url(#knobGradient)', stroke: '#0e0f10', 'stroke-width': 2 }, this.knob);
    for (let i = 0; i < 36; ++i) {
      const a = (i * Math.PI * 2) / 36;
      svgElement('line', { x1: Math.cos(a) * 46, y1: Math.sin(a) * 46, x2: Math.cos(a) * 52, y2: Math.sin(a) * 52, stroke: '#0b0c0d', 'stroke-width': 2 }, this.knob);
    }
    this.knobIndicator = svgElement('line', { x1: 0, y1: -24, x2: 0, y2: -44, stroke: '#f5f5f5', 'stroke-width': 4, 'stroke-linecap': 'round' }, this.knob);
    this.knobCap = svgElement('circle', { r: 22, fill: '#2b2e31', stroke: '#45494d', class: 'panel-button' }, encoder);
    svgElement('title', {}, this.knobCap).textContent = 'Push (ENC_SW, GPIO21)';
    svgElement('title', {}, this.knob).textContent = 'Drag or scroll to turn (ENC_A GPIO17, ENC_B GPIO22)';
    svgElement('text', { x: 0, y: 70, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#cfe8d5' }, encoder).textContent = 'SW1 EC12E · push';
    this.arrow(svg, 598, 186, -1, '↺');
    this.arrow(svg, 702, 186, 1, '↻');

    // ABORT SWITCH (S1)
    const abort = svgElement('g', { class: 'panel-button', transform: 'translate(650,236)' }, svg);
    svgElement('rect', { x: -24, y: -24, width: 48, height: 48, rx: 4, fill: '#cfd2d5', stroke: '#777' }, abort);
    this.abortCap = svgElement('circle', { r: 15, fill: 'url(#capGradient)', class: 'cap' }, abort);
    svgElement('text', { x: 0, y: 40, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#ffd6d2;font-weight:600' }, abort).textContent = 'ABORT · S1';
    svgElement('title', {}, abort).textContent = 'Abort switch (GPIO13): stops all pumps';
    this.bindButton(abort, 'abort');

    // LEDS
    this.leds = {};
    const ledDefinitions = [
      ['LED1', 612, '#ff3b30', 'ERR', 'LED1: float switch triggered (LED_ERROR, GPIO12)'],
      ['LED3', 650, '#4cd964', 'pH', 'LED3: pH board powered (PH_ON)'],
      ['LED2', 688, '#4cd964', 'TDS', 'LED2: TDS board powered (TDS_ON)'],
    ];
    for (const [id, x, color, label, title] of ledDefinitions) {
      const g = svgElement('g', { transform: `translate(${x},302)` }, svg);
      const glow = svgElement('circle', { r: 11, fill: color, filter: 'url(#glow)', opacity: 0 }, g);
      const body = svgElement('rect', { x: -6, y: -4, width: 12, height: 8, rx: 1.5, fill: '#3a3a3a', stroke: '#222' }, g);
      svgElement('text', { x: 0, y: 22, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#cfe8d5' }, g).textContent = label;
      svgElement('title', {}, g).textContent = title;
      this.leds[id] = { glow, body, color };
    }

    this.bindKnob();
    this.bindButton(this.knobCap, 'encoder');
    svg.addEventListener('wheel', (event) => this.onWheel(event), { passive: false });
  }

  tactile(parent, x, y, label, id) {
    const g = svgElement('g', { class: 'panel-button', transform: `translate(${x},${y})` }, parent);
    svgElement('rect', { x: -17, y: -17, width: 34, height: 34, rx: 3, fill: '#c9cdd1', stroke: '#7a7f84' }, g);
    const cap = svgElement('circle', { r: 10, fill: '#2a2c2f', class: 'cap' }, g);
    svgElement('text', { x: 0, y: id === 'io35' ? -24 : 32, 'text-anchor': 'middle', class: 'svg-tiny', style: 'fill:#9aa0a6' }, g).textContent = label;
    svgElement('title', {}, g).textContent = `T-Display button ${label} (GPIO${label.slice(2)})`;
    this.bindButton(g, id);
    return cap;
  }

  arrow(svg, x, y, detents, symbol) {
    const g = svgElement('g', { class: 'panel-button', transform: `translate(${x},${y})` }, svg);
    svgElement('circle', { r: 13, fill: 'rgba(0,0,0,0.35)', stroke: '#cfe8d5', 'stroke-opacity': 0.4 }, g);
    svgElement('text', { y: 5, 'text-anchor': 'middle', style: 'fill:#e8f3ea;font-size:15px' }, g).textContent = symbol;
    svgElement('title', {}, g).textContent = detents > 0 ? 'Turn clockwise (menu down)' : 'Turn counter-clockwise (menu up)';
    g.addEventListener('click', () => this.turn(detents));
  }

  bindButton(element, id) {
    let down = false;
    const press = (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (down) return;
      down = true;
      element.setPointerCapture?.(event.pointerId);
      this.send({ type: 'button', button: id, action: 'press' });
    };
    const release = () => {
      if (!down) return;
      down = false;
      this.send({ type: 'button', button: id, action: 'release' });
    };
    element.addEventListener('pointerdown', press);
    element.addEventListener('pointerup', release);
    element.addEventListener('pointercancel', release);
    element.addEventListener('lostpointercapture', release);
  }

  bindKnob() {
    let lastAngle = null;
    let accumulated = 0;
    const angleOf = (event) => {
      const box = this.knob.getBoundingClientRect();
      return Math.atan2(event.clientY - (box.top + box.height / 2), event.clientX - (box.left + box.width / 2));
    };
    this.knob.addEventListener('pointerdown', (event) => {
      lastAngle = angleOf(event);
      accumulated = 0;
      this.knob.setPointerCapture(event.pointerId);
      event.preventDefault();
    });
    this.knob.addEventListener('pointermove', (event) => {
      if (lastAngle === null) return;
      const angle = angleOf(event);
      let delta = angle - lastAngle;
      if (delta > Math.PI) delta -= 2 * Math.PI;
      if (delta < -Math.PI) delta += 2 * Math.PI;
      lastAngle = angle;
      accumulated += delta;
      const step = Math.PI / 12;  // 24 detents per turn
      while (Math.abs(accumulated) >= step) {
        const direction = Math.sign(accumulated);
        accumulated -= direction * step;
        this.turn(direction);
      }
    });
    const end = () => (lastAngle = null);
    this.knob.addEventListener('pointerup', end);
    this.knob.addEventListener('pointercancel', end);
  }

  onWheel(event) {
    event.preventDefault();
    const delta = event.deltaMode === 1 ? event.deltaY * 40 : event.deltaY;
    this.wheelAccumulator += delta;
    const threshold = 50;
    while (Math.abs(this.wheelAccumulator) >= threshold) {
      const direction = Math.sign(this.wheelAccumulator);
      this.wheelAccumulator -= direction * threshold;
      this.turn(direction);
    }
  }

  // Positive = clockwise (the firmware moves the menu pointer down)
  turn(detents) {
    this.detents += detents;
    this.knob.setAttribute('transform', `rotate(${this.detents * 15})`);
    this.send({ type: 'encoder', detents });
  }

  handleKey(event) {
    const map = { ArrowDown: 1, ArrowRight: 1, ArrowUp: -1, ArrowLeft: -1 };
    if (event.key in map) {
      this.turn(map[event.key]);
      return true;
    }
    const buttons = { Enter: 'encoder', ' ': 'encoder', Escape: 'abort', '0': 'io0' };
    if (event.key in buttons) {
      if (!event.repeat && !this.keysDown.has(event.key)) {
        this.keysDown.add(event.key);
        this.send({ type: 'button', button: buttons[event.key], action: 'press' });
      }
      return true;
    }
    return false;
  }

  handleKeyUp(event) {
    const buttons = { Enter: 'encoder', ' ': 'encoder', Escape: 'abort', '0': 'io0' };
    if (!(event.key in buttons) || !this.keysDown.has(event.key)) return false;
    this.keysDown.delete(event.key);
    this.send({ type: 'button', button: buttons[event.key], action: 'release' });
    return true;
  }

  // bytes: [1, backlight, then 135 x 240 RGB565 little endian pixels of the panel memory (portrait)]
  setPanel(bytes) {
    const data = this.image.data;
    for (let y = 0; y < 135; ++y) {
      for (let x = 0; x < 240; ++x) {
        const index = 2 + 2 * ((239 - x) * 135 + y);
        const c = bytes[index] | (bytes[index + 1] << 8);
        const o = 4 * (y * 240 + x);
        data[o] = ((c >> 11) & 31) * 255 / 31;
        data[o + 1] = ((c >> 5) & 63) * 255 / 63;
        data[o + 2] = (c & 31) * 255 / 31;
        data[o + 3] = 255;
      }
    }
    this.screenContext.putImageData(this.image, 0, 0);
    this.screenVersion++;
  }

  update(state) {
    const leds = state.board.leds;
    this.backlight = leds.backlight;
    this.screenOff.setAttribute('opacity', this.backlight ? 0 : 0.93);
    for (const [id, led] of Object.entries(this.leds)) {
      const on = !!leds[id];
      led.glow.setAttribute('opacity', on ? 0.9 : 0);
      led.body.setAttribute('fill', on ? led.color : '#3a3a3a');
    }
    const buttons = state.board.buttons;
    this.buttonIo0.setAttribute('fill', buttons.io0 ? '#666' : '#2a2c2f');
    this.buttonIo35.setAttribute('fill', buttons.io35 ? '#666' : '#2a2c2f');
    this.abortCap.setAttribute('r', buttons.abort ? 13 : 15);
    this.knobCap.setAttribute('fill', buttons.encoder ? '#55595e' : '#2b2e31');
  }
}

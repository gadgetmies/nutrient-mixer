// Time series of the true solution in the reservoirs, the firmware's own readings and the pump activity.
// One y axis per chart, a legend, a crosshair tooltip and a table of the latest values.

const SAMPLE_INTERVAL = 0.5;  // Seconds of world time between samples
const MAX_SAMPLES = 40000;

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function niceStep(range, count) {
  const raw = range / count;
  const magnitude = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * magnitude >= raw) return m * magnitude;
  return 10 * magnitude;
}

function formatTime(seconds) {
  seconds = Math.max(0, Math.round(seconds));
  const h = Math.floor(seconds / 3600), m = Math.floor(seconds / 60) % 60, s = seconds % 60;
  return (h ? `${h}:${String(m).padStart(2, '0')}` : `${m}`) + ':' + String(s).padStart(2, '0');
}

const firmwareNumber = (text) => {
  const value = parseFloat(text);
  return isFinite(value) ? value : null;
};

const CHARTS = [
  {
    id: 'tds', title: 'TDS', unit: 'ppm', digits: 0,
    series: [
      { name: 'Mixer reservoir', color: '--series-1', value: (s) => s.world.mixer.tds },
      { name: 'Plant reservoir', color: '--series-2', value: (s) => s.world.plant.tds },
      { name: 'Firmware reading', color: '--series-3', value: (s) => firmwareNumber(s.firmware.tds), step: true },
      { name: 'Target', color: '--text-secondary', value: (s) => s.firmware.targetTds, dashed: true },
    ],
  },
  {
    id: 'ph', title: 'pH', unit: '', digits: 2,
    series: [
      { name: 'Mixer reservoir', color: '--series-1', value: (s) => s.world.mixer.ph },
      { name: 'Plant reservoir', color: '--series-2', value: (s) => s.world.plant.ph },
      { name: 'Firmware reading', color: '--series-3', value: (s) => firmwareNumber(s.firmware.ph), step: true },
      { name: 'Target', color: '--text-secondary', value: (s) => s.firmware.targetPh, dashed: true },
    ],
  },
  {
    id: 'volume', title: 'Volume', unit: 'L', digits: 1,
    series: [
      { name: 'Mixer reservoir', color: '--series-1', value: (s) => s.world.mixer.volume },
      { name: 'Plant reservoir', color: '--series-2', value: (s) => s.world.plant.volume },
    ],
  },
];

const PUMP_ROWS = [
  { name: 'Output (submersible)', color: '--series-1', value: (s) => s.world.outputPump.on },
  { name: 'Input (self-priming)', color: '--series-2', value: (s) => s.world.inputPump.on },
  { name: 'pH Down', color: '--series-3', value: (s) => s.world.peristaltic.some((p) => p.on && p.bottle === 0) },
  { name: 'Nutrient A', color: '--series-3', value: (s) => s.world.peristaltic.some((p) => p.on && p.bottle === 1) },
  { name: 'Nutrient B', color: '--series-3', value: (s) => s.world.peristaltic.some((p) => p.on && p.bottle === 2) },
  { name: 'Mixer float', color: '--status-critical', value: (s) => s.world.mixer.floatClosed },
  { name: 'Plant float', color: '--status-critical', value: (s) => s.world.plant.floatClosed },
];

export class Charts {
  constructor(container) {
    this.container = container;
    this.times = [];
    this.values = { tds: [], ph: [], volume: [], pumps: [] };
    this.range = 1800;
    this.lastDraw = 0;
    this.hover = null;
    this.views = [];
    for (const chart of CHARTS) this.views.push(this.createView(chart.id, chart.title, chart.unit, chart.series));
    this.views.push(this.createView('pumps', 'Pumps and float switches', '', PUMP_ROWS, true));
    for (const button of document.querySelectorAll('#chartRange button')) {
      button.onclick = () => {
        this.range = Number(button.dataset.range);
        for (const b of document.querySelectorAll('#chartRange button')) b.classList.toggle('active', b === button);
        this.draw();
      };
    }
    document.getElementById('clearCharts').onclick = () => {
      this.times = [];
      for (const key of Object.keys(this.values)) this.values[key] = [];
      this.draw();
    };
    window.addEventListener('resize', () => this.draw());
  }

  createView(id, title, unit, series, timeline = false) {
    const wrapper = document.createElement('div');
    wrapper.className = 'chart';
    const legend = series
      .map((s) => `<span><span class="sw ${s.dashed ? 'dashed' : ''}" style="background:var(${s.color})"></span>${s.name}</span>`)
      .join('');
    wrapper.innerHTML = `<div class="chart-head"><h3>${title}${unit ? ` (${unit})` : ''}</h3>${timeline ? '' : `<div class="legend">${legend}</div>`}</div>`;
    const canvas = document.createElement('canvas');
    if (timeline) canvas.style.height = `${series.length * 22 + 26}px`;
    wrapper.appendChild(canvas);
    const tooltip = document.createElement('div');
    tooltip.className = 'chart-tooltip';
    wrapper.appendChild(tooltip);
    this.container.appendChild(wrapper);
    const view = { id, series, canvas, tooltip, timeline, unit };
    canvas.addEventListener('mousemove', (event) => {
      const box = canvas.getBoundingClientRect();
      this.hover = { x: event.clientX - box.left, view };
      this.draw();
    });
    canvas.addEventListener('mouseleave', () => {
      this.hover = null;
      this.draw();
    });
    return view;
  }

  add(state) {
    const t = state.world.seconds;
    const last = this.times[this.times.length - 1];
    if (last !== undefined && t < last) {
      // The world was reset
      this.times = [];
      for (const key of Object.keys(this.values)) this.values[key] = [];
    } else if (last !== undefined && t - last < SAMPLE_INTERVAL) {
      return;
    }
    this.times.push(t);
    for (const chart of CHARTS) this.values[chart.id].push(chart.series.map((s) => s.value(state)));
    this.values.pumps.push(PUMP_ROWS.map((row) => (row.value(state) ? 1 : 0)));
    if (this.times.length > MAX_SAMPLES) {
      this.times.splice(0, 1000);
      for (const key of Object.keys(this.values)) this.values[key].splice(0, 1000);
    }
    const now = performance.now();
    if (now - this.lastDraw > 250) this.draw();
  }

  visibleRange() {
    const end = this.times.length ? this.times[this.times.length - 1] : 0;
    const start = this.range ? Math.max(this.times[0] ?? 0, end - this.range) : this.times[0] ?? 0;
    let first = 0;
    while (first < this.times.length && this.times[first] < start) first++;
    return { start, end: Math.max(end, start + 1), first };
  }

  draw() {
    this.lastDraw = performance.now();
    if (!document.getElementById('tab-charts').classList.contains('active')) return;
    const range = this.visibleRange();
    for (const view of this.views) this.drawView(view, range);
  }

  drawView(view, range) {
    const canvas = view.canvas;
    const ratio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width) return;
    canvas.width = width * ratio;
    canvas.height = height * ratio;
    const g = canvas.getContext('2d');
    g.scale(ratio, ratio);
    const text = cssVar('--text-secondary'), muted = cssVar('--text-muted'), grid = cssVar('--border');
    g.font = '11px -apple-system, Segoe UI, sans-serif';
    // The same plot area on every chart so that their time axes line up
    const left = 140, right = 110, top = 8, bottom = 20;
    const plotWidth = width - left - right, plotHeight = height - top - bottom;
    const x = (t) => left + ((t - range.start) / (range.end - range.start)) * plotWidth;
    const data = this.values[view.id];
    const indices = [];
    // Thin out to about one point per pixel
    const stride = Math.max(1, Math.floor((this.times.length - range.first) / plotWidth));
    for (let i = range.first; i < this.times.length; i += stride) indices.push(i);
    if (this.times.length && indices[indices.length - 1] !== this.times.length - 1) indices.push(this.times.length - 1);

    // X axis
    g.strokeStyle = grid;
    g.fillStyle = muted;
    g.lineWidth = 1;
    const timeStep = niceStep(range.end - range.start, Math.max(2, Math.floor(plotWidth / 90)));
    g.textAlign = 'center';
    for (let t = Math.ceil(range.start / timeStep) * timeStep; t <= range.end; t += timeStep) {
      g.fillText(formatTime(t), x(t), height - 5);
    }

    if (view.timeline) {
      const rowHeight = plotHeight / view.series.length;
      view.series.forEach((row, r) => {
        const y = top + r * rowHeight;
        g.fillStyle = text;
        g.textAlign = 'right';
        g.fillText(row.name, left - 8, y + rowHeight / 2 + 4);
        g.fillStyle = cssVar('--surface-2');
        g.fillRect(left, y + 3, plotWidth, rowHeight - 6);
        g.fillStyle = cssVar(row.color);
        let startT = null;
        for (let k = 0; k < indices.length; ++k) {
          const i = indices[k];
          const on = data[i][r];
          if (on && startT === null) startT = this.times[i];
          if ((!on || k === indices.length - 1) && startT !== null) {
            const endT = this.times[i];
            g.fillRect(x(startT), y + 4, Math.max(2, x(endT) - x(startT)), rowHeight - 8);
            startT = null;
          }
        }
      });
      this.drawHover(view, range, x, left, plotWidth, top, plotHeight, (i) =>
        view.series.map((row, r) => [row, data[i][r] ? 'on' : 'off']));
      return;
    }

    // Y range from the visible values
    let min = Infinity, max = -Infinity;
    for (const i of indices) {
      for (const v of data[i]) {
        if (v != null && isFinite(v)) {
          min = Math.min(min, v);
          max = Math.max(max, v);
        }
      }
    }
    if (!isFinite(min)) { min = 0; max = 1; }
    const pad = Math.max((max - min) * 0.08, view.id === 'ph' ? 0.1 : 1);
    min -= pad;
    max += pad;
    if (view.id !== 'ph' && min < 0) min = 0;
    const y = (v) => top + (1 - (v - min) / (max - min)) * plotHeight;
    const valueStep = niceStep(max - min, Math.max(2, Math.floor(plotHeight / 34)));
    g.textAlign = 'right';
    for (let v = Math.ceil(min / valueStep) * valueStep; v <= max; v += valueStep) {
      g.strokeStyle = grid;
      g.globalAlpha = 0.6;
      g.beginPath();
      g.moveTo(left, Math.round(y(v)) + 0.5);
      g.lineTo(left + plotWidth, Math.round(y(v)) + 0.5);
      g.stroke();
      g.globalAlpha = 1;
      g.fillStyle = muted;
      g.fillText(v.toFixed(valueStep < 1 ? (valueStep < 0.1 ? 2 : 1) : 0), left - 6, y(v) + 4);
    }

    // Series (2 px lines; the target dashed)
    view.series.forEach((series, s) => {
      g.strokeStyle = cssVar(series.color);
      g.lineWidth = 2;
      g.lineJoin = 'round';
      g.setLineDash(series.dashed ? [5, 4] : []);
      g.beginPath();
      let drawing = false, previousY = null;
      for (const i of indices) {
        const v = data[i][s];
        if (v == null || !isFinite(v)) {
          drawing = false;
          continue;
        }
        const px = x(this.times[i]), py = y(v);
        if (!drawing) g.moveTo(px, py);
        else if (series.step) g.lineTo(px, previousY);
        g.lineTo(px, py);
        drawing = true;
        previousY = py;
      }
      g.stroke();
      g.setLineDash([]);
    });

    // Direct labels at the line ends
    const last = indices[indices.length - 1];
    if (last !== undefined) {
      const placed = [];
      view.series.forEach((series, s) => {
        const v = data[last][s];
        if (v == null || !isFinite(v)) return;
        let ly = y(v) + 4;
        for (const p of placed) if (Math.abs(p - ly) < 12) ly = p + 12;
        placed.push(ly);
        g.fillStyle = cssVar(series.color);
        g.fillRect(left + plotWidth + 6, ly - 5, 8, 3);
        g.fillStyle = text;
        g.textAlign = 'left';
        g.fillText(`${v.toFixed(CHARTS.find((c) => c.id === view.id).digits)} ${series.name.split(' ')[0]}`, left + plotWidth + 18, ly);
      });
    }
    this.drawHover(view, range, x, left, plotWidth, top, plotHeight, (i) =>
      view.series.map((series, s) => {
        const v = data[i][s];
        return [series, v == null || !isFinite(v) ? '–' : v.toFixed(CHARTS.find((c) => c.id === view.id).digits) + (view.unit ? ' ' + view.unit : '')];
      }));
  }

  drawHover(view, range, x, left, plotWidth, top, plotHeight, rows) {
    if (!this.hover || !this.times.length || this.hover.x < left || this.hover.x > left + plotWidth) {
      if (!this.hover || this.hover.view === view) view.tooltip.style.display = 'none';
      if (!this.hover) return;
    }
    // Crosshair on every chart at the same time; tooltip on the hovered one
    const t = range.start + ((this.hover.x - left) / plotWidth) * (range.end - range.start);
    let best = range.first, bestDistance = Infinity;
    for (let i = range.first; i < this.times.length; ++i) {
      const d = Math.abs(this.times[i] - t);
      if (d < bestDistance) { bestDistance = d; best = i; }
      if (this.times[i] > t) break;
    }
    const g = view.canvas.getContext('2d');
    const px = x(this.times[best]);
    if (px < left || px > left + plotWidth) return;
    g.strokeStyle = cssVar('--text-muted');
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(Math.round(px) + 0.5, top);
    g.lineTo(Math.round(px) + 0.5, top + plotHeight);
    g.stroke();
    if (this.hover.view !== view) return;
    view.tooltip.innerHTML = `<div class="t">${formatTime(this.times[best])}</div>` + rows(best)
      .map(([series, value]) => `<div class="r"><span class="sw" style="background:var(${series.color})"></span><span class="n">${series.name}</span><span class="val">${value}</span></div>`)
      .join('');
    view.tooltip.style.display = 'block';
    const width = view.tooltip.offsetWidth;
    view.tooltip.style.left = `${px + 12 + width > view.canvas.clientWidth ? px - width - 12 : px + 12}px`;
    view.tooltip.style.top = `${30}px`;
  }
}

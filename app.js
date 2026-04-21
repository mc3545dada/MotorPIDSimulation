class PIDController {
  constructor(kp, ki, kd, outLimit, ioutLimit) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.outLimit = outLimit;
    this.ioutLimit = ioutLimit;
    this.clear();
  }

  clear() {
    this.err = [0, 0, 0];
    this.pOut = 0;
    this.iOut = 0;
    this.dOut = 0;
    this.out = 0;
  }

  update(current, target) {
    this.err[2] = this.err[1];
    this.err[1] = this.err[0];
    this.err[0] = target - current;
    this.pOut = this.kp * this.err[0];
    this.iOut += this.ki * this.err[0];
    this.dOut = this.kd * (this.err[0] - this.err[1]);
    this.iOut = clampSymmetric(this.iOut, this.ioutLimit);
    this.out = clampSymmetric(this.pOut + this.iOut + this.dOut, this.outLimit);
    return this.out;
  }
}

const defaults = {
  mode: "cascade",
  targetAngle: 90,
  targetSpeed: 120,
  angleKp: 15,
  angleKi: 0,
  angleKd: 0,
  angleOutLimit: 720,
  angleIOutLimit: 1000,
  speedKp: 60,
  speedKi: 0.9,
  speedKd: 0,
  speedOutLimit: 16384,
  speedIOutLimit: 10000,
  simTime: 3,
  dtMs: 1,
  maxCurrentA: 3,
  torqueConstant: 0.741,
  speedTorqueGradient: 156,
  mechTauMs: 3,
  initialAngle: 0,
  initialSpeed: 0,
  loadTorque: 0.15,
  disturbTime: 1.2,
  disturbTorque: 0,
  speedNoise: 0,
  wrapAngle: true,
  showReference: true,
};

const presets = {
  yaw: {
    mode: "cascade",
    targetAngle: 90,
    targetSpeed: 120,
    angleKp: 15,
    angleKi: 0,
    angleKd: 0,
    angleOutLimit: 720,
    angleIOutLimit: 1000,
    speedKp: 60,
    speedKi: 0.9,
    speedKd: 0,
    speedOutLimit: 16384,
    speedIOutLimit: 10000,
  },
  pitch: {
    mode: "cascade",
    targetAngle: 45,
    targetSpeed: 100,
    angleKp: 16,
    angleKi: 0,
    angleKd: 0,
    angleOutLimit: 210,
    angleIOutLimit: 1000,
    speedKp: 80,
    speedKi: 0.5,
    speedKd: 0,
    speedOutLimit: 16384,
    speedIOutLimit: 10000,
  },
  speed: {
    mode: "speed",
    targetAngle: 0,
    targetSpeed: 150,
    angleKp: 15,
    angleKi: 0,
    angleKd: 0,
    angleOutLimit: 720,
    angleIOutLimit: 1000,
    speedKp: 60,
    speedKi: 0.9,
    speedKd: 0,
    speedOutLimit: 16384,
    speedIOutLimit: 10000,
  },
};

const ids = Object.keys(defaults);
const elements = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));

const metricsEl = document.getElementById("metrics");
const canvases = {
  main: document.getElementById("mainChart"),
  output: document.getElementById("outputChart"),
  error: document.getElementById("errorChart"),
};
const chartState = new Map();
const tooltipEl = createChartTooltip();

document.getElementById("runButton").addEventListener("click", runSimulation);
const runButtonTop = document.getElementById("runButtonTop");
if (runButtonTop) runButtonTop.addEventListener("click", runSimulation);
const runButtonTopResult = document.getElementById("runButtonTopResult");
if (runButtonTopResult) runButtonTopResult.addEventListener("click", runSimulation);
document.getElementById("resetButton").addEventListener("click", () => {
  applyValues(defaults);
  runSimulation();
});

document.querySelectorAll("[data-preset]").forEach((button) => {
  button.addEventListener("click", () => {
    applyValues({ ...readValues(), ...presets[button.dataset.preset] });
    runSimulation();
  });
});
Object.values(canvases).forEach(bindChartInteraction);

applyValues(defaults);
runSimulation();

function applyValues(values) {
  for (const id of ids) {
    const element = elements[id];
    const value = values[id];
    if (element.type === "checkbox") {
      element.checked = Boolean(value);
    } else {
      element.value = value;
    }
  }
}

function readValues() {
  const result = {};
  for (const id of ids) {
    const element = elements[id];
    result[id] = element.type === "checkbox" ? element.checked : Number(element.value);
    if (element.tagName === "SELECT") {
      result[id] = element.value;
    }
  }
  return result;
}

function runSimulation() {
  const config = readValues();
  const result = simulate(config);
  renderMetrics(config, result);
  renderCharts(config, result);
  hideChartTooltip();
}

function simulate(config) {
  const dt = config.dtMs / 1000;
  const steps = Math.max(1, Math.floor(config.simTime / dt));
  const anglePid = new PIDController(
    config.angleKp,
    config.angleKi,
    config.angleKd,
    config.angleOutLimit,
    config.angleIOutLimit
  );
  const speedPid = new PIDController(
    config.speedKp,
    config.speedKi,
    config.speedKd,
    config.speedOutLimit,
    config.speedIOutLimit
  );

  let angleDeg = config.initialAngle;
  let speedRpm = config.initialSpeed;

  const time = [];
  const reference = [];
  const mainValue = [];
  const speedValue = [];
  const angleValue = [];
  const output = [];
  const currentA = [];
  const error = [];

  for (let i = 0; i < steps; i += 1) {
    const t = i * dt;
    const targetAngle = config.targetAngle;
    const targetSpeed = config.targetSpeed;
    const disturbActive = t >= config.disturbTime;
    const appliedLoad = config.loadTorque + (disturbActive ? config.disturbTorque : 0);

    const measuredSpeed = speedRpm + pseudoNoise(i) * config.speedNoise;
    const angleErrorRaw = targetAngle - angleDeg;
    const angleError = config.wrapAngle ? wrapTo180(angleErrorRaw) : angleErrorRaw;

    let speedTarget = targetSpeed;
    if (config.mode === "cascade") {
      speedTarget = anglePid.update(0, angleError);
    }

    const ctrlOutput = speedPid.update(measuredSpeed, speedTarget);
    const cmdCurrent = (ctrlOutput / Math.max(config.speedOutLimit, 1)) * config.maxCurrentA;
    const limitedCurrent = clampSymmetric(cmdCurrent, config.maxCurrentA);
    const torque = limitedCurrent * config.torqueConstant;

    // Use command-direction load to avoid discontinuous sign flips near zero speed.
    // This keeps the teaching model stable in speed-only mode while preserving load effect.
    const loadDirection = Math.sign(speedTarget || config.targetSpeed || 1);
    const equivalentTargetSpeed = (torque - appliedLoad * loadDirection) * config.speedTorqueGradient;
    const tau = Math.max(config.mechTauMs / 1000, dt);
    speedRpm += (equivalentTargetSpeed - speedRpm) * (dt / tau);
    angleDeg += speedRpm * 6 * dt;

    time.push(t);
    reference.push(config.mode === "cascade" ? targetAngle : targetSpeed);
    mainValue.push(config.mode === "cascade" ? angleDeg : speedRpm);
    speedValue.push(speedRpm);
    angleValue.push(angleDeg);
    output.push(ctrlOutput);
    currentA.push(limitedCurrent);
    error.push(config.mode === "cascade" ? angleError : targetSpeed - measuredSpeed);
  }

  return {
    time,
    reference,
    mainValue,
    speedValue,
    angleValue,
    output,
    currentA,
    error,
    metrics: calculateMetrics(config, time, reference, mainValue),
  };
}

function calculateMetrics(config, time, reference, actual) {
  const finalRef = reference[reference.length - 1];
  const finalActual = actual[actual.length - 1];
  const steadyError = finalRef - finalActual;
  const amplitude = Math.max(Math.abs(finalRef), 1e-6);

  let peak = actual[0];
  for (const value of actual) {
    if (finalRef >= 0) {
      peak = Math.max(peak, value);
    } else {
      peak = Math.min(peak, value);
    }
  }

  const overshoot = finalRef === 0
    ? 0
    : Math.max(0, ((peak - finalRef) / Math.abs(finalRef)) * 100);

  const tolerance = 0.02 * amplitude;
  let settlingTime = null;
  for (let i = 0; i < actual.length; i += 1) {
    const tail = actual.slice(i);
    if (tail.every((value) => Math.abs(value - finalRef) <= tolerance)) {
      settlingTime = time[i];
      break;
    }
  }

  let riseTime = null;
  const riseThreshold = finalRef * 0.9;
  for (let i = 0; i < actual.length; i += 1) {
    if ((finalRef >= 0 && actual[i] >= riseThreshold) || (finalRef < 0 && actual[i] <= riseThreshold)) {
      riseTime = time[i];
      break;
    }
  }

  return {
    overshoot,
    steadyError,
    settlingTime,
    riseTime,
    finalActual,
  };
}

function renderMetrics(config, result) {
  const unit = config.mode === "cascade" ? "deg" : "rpm";
  const cards = [
    {
      label: "最终输出",
      value: `${format(result.metrics.finalActual, 2)} ${unit}`,
      help: config.mode === "cascade" ? "末端角度" : "末端转速",
    },
    {
      label: "稳态误差",
      value: `${format(result.metrics.steadyError, 2)} ${unit}`,
      help: "目标减实际",
    },
    {
      label: "超调",
      value: `${format(result.metrics.overshoot, 1)} %`,
      help: "越大越容易振荡",
    },
    {
      label: "调节时间",
      value: result.metrics.settlingTime == null ? "--" : `${format(result.metrics.settlingTime, 3)} s`,
      help: "进入并保持 2% 误差带",
    },
  ];

  metricsEl.innerHTML = cards
    .map(
      (card) => `
        <article class="metric-card">
          <div class="metric-label">${card.label}</div>
          <div class="metric-value">${card.value}</div>
          <div class="metric-help">${card.help}</div>
        </article>
      `
    )
    .join("");
}

function renderCharts(config, result) {
  drawLineChart(canvases.main, {
    labels: result.time,
    series: [
      config.showReference && {
        label: config.mode === "cascade" ? "目标角度" : "目标速度",
        color: "#b4492d",
        values: result.reference,
      },
      {
        label: config.mode === "cascade" ? "实际角度" : "实际速度",
        color: "#145b73",
        values: result.mainValue,
      },
    ].filter(Boolean),
  });

  drawLineChart(canvases.output, {
    labels: result.time,
    series: [
      { label: "PID 输出", color: "#7c3aed", values: result.output },
      { label: "等效电流(A)", color: "#1f7a4f", values: result.currentA },
    ],
  });

  drawLineChart(canvases.error, {
    labels: result.time,
    series: [{ label: "误差", color: "#a55d14", values: result.error }],
  });
}

function drawLineChart(canvas, data, hoverIndex = null) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const padding = { top: 20, right: 18, bottom: 28, left: 44 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fffaf2";
  ctx.fillRect(0, 0, width, height);

  const allValues = data.series.flatMap((series) => series.values);
  let minY = Math.min(...allValues);
  let maxY = Math.max(...allValues);
  if (minY === maxY) {
    minY -= 1;
    maxY += 1;
  }
  const yPad = (maxY - minY) * 0.1;
  minY -= yPad;
  maxY += yPad;

  const minX = data.labels[0] ?? 0;
  const maxX = data.labels[data.labels.length - 1] ?? 1;
  const safeXSpan = Math.max(maxX - minX, 1e-9);
  chartState.set(canvas.id, { data, minX, maxX, minY, maxY, padding, plotWidth, plotHeight, safeXSpan });

  ctx.strokeStyle = "rgba(107, 114, 128, 0.22)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = padding.top + (plotHeight / 4) * i;
    ctx.beginPath();
    ctx.moveTo(padding.left, y);
    ctx.lineTo(width - padding.right, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "#94a3b8";
  ctx.beginPath();
  ctx.moveTo(padding.left, padding.top);
  ctx.lineTo(padding.left, height - padding.bottom);
  ctx.lineTo(width - padding.right, height - padding.bottom);
  ctx.stroke();

  ctx.fillStyle = "#475569";
  ctx.font = '12px "Segoe UI"';
  for (let i = 0; i <= 4; i += 1) {
    const value = maxY - ((maxY - minY) / 4) * i;
    const y = padding.top + (plotHeight / 4) * i + 4;
    ctx.fillText(format(value, 1), 6, y);
  }

  for (let i = 0; i <= 4; i += 1) {
    const value = minX + (safeXSpan / 4) * i;
    const x = padding.left + (plotWidth / 4) * i - 8;
    ctx.fillText(format(value, 2), x, height - 8);
  }

  for (const series of data.series) {
    ctx.strokeStyle = series.color;
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    series.values.forEach((value, index) => {
      const x = padding.left + ((data.labels[index] - minX) / safeXSpan) * plotWidth;
      const y = padding.top + ((maxY - value) / (maxY - minY)) * plotHeight;
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  }

  if (hoverIndex != null && hoverIndex >= 0 && hoverIndex < data.labels.length) {
    const hoverX = padding.left + ((data.labels[hoverIndex] - minX) / safeXSpan) * plotWidth;
    ctx.strokeStyle = "rgba(10, 80, 110, 0.45)";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(hoverX, padding.top);
    ctx.lineTo(hoverX, height - padding.bottom);
    ctx.stroke();

    for (const series of data.series) {
      const v = series.values[hoverIndex];
      const y = padding.top + ((maxY - v) / (maxY - minY)) * plotHeight;
      ctx.beginPath();
      ctx.fillStyle = series.color;
      ctx.arc(hoverX, y, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#f8f6f1";
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  let legendX = padding.left;
  const legendY = 12;
  for (const series of data.series) {
    ctx.fillStyle = series.color;
    ctx.fillRect(legendX, legendY - 8, 10, 10);
    ctx.fillStyle = "#334155";
    ctx.fillText(series.label, legendX + 14, legendY);
    legendX += ctx.measureText(series.label).width + 42;
  }
}

function bindChartInteraction(canvas) {
  if (!canvas) return;
  canvas.addEventListener("mousemove", (event) => {
    const state = chartState.get(canvas.id);
    if (!state || !state.data.labels.length) return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const x = (event.clientX - rect.left) * scaleX;
    const minPlotX = state.padding.left;
    const maxPlotX = canvas.width - state.padding.right;
    if (x < minPlotX || x > maxPlotX) {
      drawLineChart(canvas, state.data);
      hideChartTooltip();
      return;
    }

    const ratio = (x - minPlotX) / (maxPlotX - minPlotX);
    const index = Math.max(0, Math.min(state.data.labels.length - 1, Math.round(ratio * (state.data.labels.length - 1))));
    drawLineChart(canvas, state.data, index);
    showChartTooltip(state.data, index, event.clientX, event.clientY);
  });

  canvas.addEventListener("mouseleave", () => {
    const state = chartState.get(canvas.id);
    if (state) drawLineChart(canvas, state.data);
    hideChartTooltip();
  });
}

function createChartTooltip() {
  const el = document.createElement("div");
  el.className = "chart-tooltip";
  el.style.display = "none";
  document.body.appendChild(el);
  return el;
}

function showChartTooltip(data, index, clientX, clientY) {
  const lines = [`t = ${format(data.labels[index], 3)} s`];
  for (const series of data.series) {
    lines.push(`${series.label}: ${format(series.values[index], 3)}`);
  }
  tooltipEl.innerHTML = lines.map((line) => `<div>${line}</div>`).join("");
  tooltipEl.style.display = "block";
  tooltipEl.style.left = `${clientX + 14}px`;
  tooltipEl.style.top = `${clientY + 14}px`;

  const rect = tooltipEl.getBoundingClientRect();
  if (rect.right > window.innerWidth - 8) {
    tooltipEl.style.left = `${clientX - rect.width - 14}px`;
  }
  if (rect.bottom > window.innerHeight - 8) {
    tooltipEl.style.top = `${clientY - rect.height - 14}px`;
  }
}

function hideChartTooltip() {
  tooltipEl.style.display = "none";
}

function clampSymmetric(value, limit) {
  if (limit <= 0) {
    return 0;
  }
  return Math.max(-limit, Math.min(limit, value));
}

function wrapTo180(value) {
  let result = value;
  while (result > 180) result -= 360;
  while (result <= -180) result += 360;
  return result;
}

function pseudoNoise(index) {
  const x = Math.sin(index * 12.9898) * 43758.5453;
  return (x - Math.floor(x)) * 2 - 1;
}

function format(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

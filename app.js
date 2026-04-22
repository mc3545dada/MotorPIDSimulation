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
  simMode: "realtime",
  mode: "cascade",
  realtimeSpeed: 1,
  realtimeWindowSec: 8,
  extStepHigh: 90,
  extStepLow: 0,
  extStepPeriodSec: 1.5,
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
  speedFeedforwardEnable: false,
  speedFeedforwardGain: 0,
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
  loadTorque: 0,
  disturbTime: 1.2,
  disturbTorque: 0,
  speedNoise: 0,
  wrapAngle: true,
  showMainTargetAngle: true,
  showMainActualAngle: true,
  showMainTargetSpeed: true,
  showMainActualSpeed: true,
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
    angleKp: 0,
    angleKi: 0,
    angleKd: 0,
    angleOutLimit: 720,
    angleIOutLimit: 1000,
    speedKp: 0,
    speedKi: 0,
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
let realtimeRuntime = null;
let lastRenderSnapshot = null;

const runButtonTopResult = document.getElementById("runButtonTopResult");
if (runButtonTopResult) runButtonTopResult.addEventListener("click", runSimulation);
const updateRtButton = document.getElementById("updateRtButton");
if (updateRtButton) updateRtButton.addEventListener("click", updateRealtimeTargets);
const extStepButton = document.getElementById("extStepButton");
if (extStepButton) extStepButton.addEventListener("click", toggleExternalStep);
document.getElementById("resetButton").addEventListener("click", () => {
  stopRealtimeSimulation();
  applyValues(defaults);
  updateModeUI();
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
const simModeEl = document.getElementById("simMode");
if (simModeEl) {
  simModeEl.addEventListener("change", updateModeUI);
}
bindMainSeriesToggleRepaint();
updateModeUI();
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
  hideChartTooltip();
  chartState.clear();
  const config = readValues();
  if (config.simMode === "realtime") {
    startRealtimeSimulation(config);
    return;
  }

  stopRealtimeSimulation();
  const result = simulate(config);
  lastRenderSnapshot = { config: { ...config }, result };
  renderMetrics(config, result);
  renderCharts(config, result);
}

function updateModeUI() {
  const config = readValues();
  const realtime = config.simMode === "realtime";
  document.querySelectorAll(".realtime-only").forEach((el) => {
    el.style.display = realtime ? "block" : "none";
  });
  setRunButtonsText(realtime ? "启动实时仿真" : "运行仿真");
  setStepButtonText(realtimeRuntime?.stepGen?.enabled ?? false);
  if (!realtime) {
    stopRealtimeSimulation();
  }
}

function setRunButtonsText(text) {
  const ids = ["runButtonTopResult"];
  ids.forEach((id) => {
    const btn = document.getElementById(id);
    if (btn) btn.textContent = text;
  });
}

function buildRealtimeRuntime(config) {
  const dt = config.dtMs / 1000;
  return {
    config: { ...config },
    dt,
    stepIndex: 0,
    fraction: 0,
    timer: null,
    lastTickMs: performance.now(),
    stepGen: {
      enabled: false,
      highPhase: true,
      anchorTime: 0,
    },
    anglePid: new PIDController(
      config.angleKp,
      config.angleKi,
      config.angleKd,
      config.angleOutLimit,
      config.angleIOutLimit
    ),
    speedPid: new PIDController(
      config.speedKp,
      config.speedKi,
      config.speedKd,
      config.speedOutLimit,
      config.speedIOutLimit
    ),
    angleDeg: config.initialAngle,
    speedRpm: config.initialSpeed,
    time: [],
    targetAngleSeries: [],
    reference: [],
    targetSpeedSeries: [],
    mainValue: [],
    speedValue: [],
    angleValue: [],
    output: [],
    currentA: [],
    error: [],
  };
}

function startRealtimeSimulation(config) {
  stopRealtimeSimulation();
  realtimeRuntime = buildRealtimeRuntime(config);
  setStepButtonText(realtimeRuntime.stepGen.enabled);
  runRealtimeChunk(1);
  renderRealtimeRuntime();
  const tickMs = 33;
  realtimeRuntime.lastTickMs = performance.now();
  realtimeRuntime.timer = setInterval(() => {
    const rt = realtimeRuntime;
    if (!rt) return;
    const now = performance.now();
    const elapsedSec = Math.max(0, (now - rt.lastTickMs) / 1000);
    rt.lastTickMs = now;
    const speedFactor = Math.max(0.2, Number(rt.config.realtimeSpeed) || 1);
    rt.fraction += (elapsedSec * speedFactor) / rt.dt;
    const steps = Math.max(1, Math.floor(rt.fraction));
    rt.fraction -= steps;
    runRealtimeChunk(steps);
    renderRealtimeRuntime();
  }, tickMs);
}

function stopRealtimeSimulation() {
  if (!realtimeRuntime) return;
  if (realtimeRuntime.timer) clearInterval(realtimeRuntime.timer);
  realtimeRuntime.timer = null;
  realtimeRuntime = null;
  setStepButtonText(false);
}

function runRealtimeChunk(steps) {
  const rt = realtimeRuntime;
  if (!rt) return;
  for (let n = 0; n < steps; n += 1) {
    runRealtimeStep(rt);
  }
}

function runRealtimeStep(rt) {
  const c = rt.config;
  const t = rt.stepIndex * rt.dt;
  const period = Math.max(0.05, Number(c.extStepPeriodSec) || 1.5);
  if (rt.stepGen.enabled) {
    while (t - rt.stepGen.anchorTime >= period) {
      rt.stepGen.anchorTime += period;
      rt.stepGen.highPhase = !rt.stepGen.highPhase;
    }
  }

  const stepTarget = rt.stepGen.highPhase ? c.extStepHigh : c.extStepLow;
  let targetAngle = c.targetAngle;
  let targetSpeed = c.targetSpeed;
  if (rt.stepGen.enabled) {
    if (c.mode === "cascade") {
      targetAngle = stepTarget;
    } else {
      targetSpeed = stepTarget;
    }
  }
  const disturbActive = t >= c.disturbTime;
  const appliedLoad = c.loadTorque + (disturbActive ? c.disturbTorque : 0);

  const measuredSpeedRaw = rt.speedRpm + pseudoNoise(rt.stepIndex) * c.speedNoise;
  const measuredSpeed = Math.round(measuredSpeedRaw);
  const angleErrorRaw = targetAngle - rt.angleDeg;
  const angleError = c.wrapAngle ? wrapTo180(angleErrorRaw) : angleErrorRaw;

  let speedTarget = targetSpeed;
  if (c.mode === "cascade") {
    speedTarget = rt.anglePid.update(0, angleError);
  }

  const pidOut = rt.speedPid.update(measuredSpeedRaw, speedTarget);
  const ffOut = c.speedFeedforwardEnable ? c.speedFeedforwardGain * speedTarget : 0;
  const ctrlOutput = clampSymmetric(pidOut + ffOut, c.speedOutLimit);
  const cmdCurrent = (ctrlOutput / Math.max(c.speedOutLimit, 1)) * c.maxCurrentA;
  const limitedCurrent = clampSymmetric(cmdCurrent, c.maxCurrentA);
  const torque = limitedCurrent * c.torqueConstant;

  const loadDirection = getLoadOpposeDirection(rt.speedRpm, torque);
  const equivalentTargetSpeed = (torque - appliedLoad * loadDirection) * c.speedTorqueGradient;
  const tau = Math.max(c.mechTauMs / 1000, rt.dt);
  rt.speedRpm += (equivalentTargetSpeed - rt.speedRpm) * (rt.dt / tau);
  rt.angleDeg += rt.speedRpm * 6 * rt.dt;

  rt.time.push(t);
  rt.targetAngleSeries.push(targetAngle);
  rt.reference.push(c.mode === "cascade" ? targetAngle : targetSpeed);
  rt.targetSpeedSeries.push(speedTarget);
  rt.mainValue.push(c.mode === "cascade" ? rt.angleDeg : measuredSpeed);
  rt.speedValue.push(measuredSpeed);
  rt.angleValue.push(rt.angleDeg);
  rt.output.push(ctrlOutput);
  rt.currentA.push(limitedCurrent);
  rt.error.push(c.mode === "cascade" ? angleError : targetSpeed - measuredSpeedRaw);
  rt.stepIndex += 1;
  trimRealtimeSeries(rt);
}

function renderRealtimeRuntime() {
  const rt = realtimeRuntime;
  if (!rt || rt.time.length === 0) return;
  const result = {
    time: rt.time,
    targetAngleSeries: rt.targetAngleSeries,
    reference: rt.reference,
    targetSpeedSeries: rt.targetSpeedSeries,
    mainValue: rt.mainValue,
    speedValue: rt.speedValue,
    angleValue: rt.angleValue,
    output: rt.output,
    currentA: rt.currentA,
    error: rt.error,
    metrics: calculateMetrics(rt.config, rt.time, rt.reference, rt.mainValue),
  };
  const viewConfig = { ...rt.config, ...readMainSeriesToggleValues() };
  lastRenderSnapshot = { config: { ...viewConfig }, result };
  renderMetrics(rt.config, result);
  renderCharts(viewConfig, result);
}

function updateRealtimeTargets() {
  if (!realtimeRuntime) return;
  const latest = readValues();
  const rt = realtimeRuntime;
  Object.assign(rt.config, latest);
  rt.anglePid.kp = latest.angleKp;
  rt.anglePid.ki = latest.angleKi;
  rt.anglePid.kd = latest.angleKd;
  rt.anglePid.outLimit = latest.angleOutLimit;
  rt.anglePid.ioutLimit = latest.angleIOutLimit;
  rt.speedPid.kp = latest.speedKp;
  rt.speedPid.ki = latest.speedKi;
  rt.speedPid.kd = latest.speedKd;
  rt.speedPid.outLimit = latest.speedOutLimit;
  rt.speedPid.ioutLimit = latest.speedIOutLimit;
  rt.config.extStepPeriodSec = Math.max(0.05, Number(latest.extStepPeriodSec) || 1.5);
  rt.config.extStepHigh = Number(latest.extStepHigh);
  rt.config.extStepLow = Number(latest.extStepLow);
}

function toggleExternalStep() {
  if (!realtimeRuntime) return;
  updateRealtimeTargets();
  const rt = realtimeRuntime;
  rt.stepGen.enabled = !rt.stepGen.enabled;
  rt.stepGen.highPhase = true;
  rt.stepGen.anchorTime = rt.stepIndex * rt.dt;
  setStepButtonText(rt.stepGen.enabled);
}

function setStepButtonText(enabled) {
  if (!extStepButton) return;
  extStepButton.textContent = enabled ? "关闭外部阶跃" : "启用外部阶跃";
}

function trimRealtimeSeries(rt) {
  const windowSec = Math.max(1, Number(rt.config.realtimeWindowSec) || 8);
  const maxPoints = Math.max(64, Math.ceil((windowSec / rt.dt) * 1.2));
  if (rt.time.length <= maxPoints) return;
  const removeCount = rt.time.length - maxPoints;
  rt.time.splice(0, removeCount);
  rt.targetAngleSeries.splice(0, removeCount);
  rt.reference.splice(0, removeCount);
  rt.targetSpeedSeries.splice(0, removeCount);
  rt.mainValue.splice(0, removeCount);
  rt.speedValue.splice(0, removeCount);
  rt.angleValue.splice(0, removeCount);
  rt.output.splice(0, removeCount);
  rt.currentA.splice(0, removeCount);
  rt.error.splice(0, removeCount);
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
  const targetAngleSeries = [];
  const reference = [];
  const targetSpeedSeries = [];
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

    const measuredSpeedRaw = speedRpm + pseudoNoise(i) * config.speedNoise;
    const measuredSpeed = Math.round(measuredSpeedRaw);
    const angleErrorRaw = targetAngle - angleDeg;
    const angleError = config.wrapAngle ? wrapTo180(angleErrorRaw) : angleErrorRaw;

    let speedTarget = targetSpeed;
    if (config.mode === "cascade") {
      speedTarget = anglePid.update(0, angleError);
    }

    const pidOut = speedPid.update(measuredSpeedRaw, speedTarget);
    const ffOut = config.speedFeedforwardEnable ? config.speedFeedforwardGain * speedTarget : 0;
    const ctrlOutput = clampSymmetric(pidOut + ffOut, config.speedOutLimit);
    const cmdCurrent = (ctrlOutput / Math.max(config.speedOutLimit, 1)) * config.maxCurrentA;
    const limitedCurrent = clampSymmetric(cmdCurrent, config.maxCurrentA);
    const torque = limitedCurrent * config.torqueConstant;

    // Use command-direction load to avoid discontinuous sign flips near zero speed.
    // This keeps the teaching model stable in speed-only mode while preserving load effect.
    const loadDirection = getLoadOpposeDirection(speedRpm, torque);
    const equivalentTargetSpeed = (torque - appliedLoad * loadDirection) * config.speedTorqueGradient;
    const tau = Math.max(config.mechTauMs / 1000, dt);
    speedRpm += (equivalentTargetSpeed - speedRpm) * (dt / tau);
    angleDeg += speedRpm * 6 * dt;

    time.push(t);
    targetAngleSeries.push(targetAngle);
    reference.push(config.mode === "cascade" ? targetAngle : targetSpeed);
    mainValue.push(config.mode === "cascade" ? angleDeg : measuredSpeed);
    speedValue.push(measuredSpeed);
    targetSpeedSeries.push(speedTarget);
    angleValue.push(angleDeg);
    output.push(ctrlOutput);
    currentA.push(limitedCurrent);
    error.push(config.mode === "cascade" ? angleError : targetSpeed - measuredSpeedRaw);
  }

  return {
    time,
    targetAngleSeries,
    reference,
    targetSpeedSeries,
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
  const wrapAngleView = config.mode === "cascade" && config.wrapAngle;
  const refSeries = wrapAngleView ? reference.map((v) => wrapTo180(v)) : reference;
  const actualSeries = wrapAngleView ? actual.map((v) => wrapTo180(v)) : actual;

  const finalRef = refSeries[refSeries.length - 1];
  const finalActual = actualSeries[actualSeries.length - 1];
  const steadyError = wrapAngleView
    ? wrapTo180(finalRef - finalActual)
    : (finalRef - finalActual);
  const amplitude = Math.max(Math.abs(finalRef), 1e-6);

  let peak = actualSeries[0];
  for (const value of actualSeries) {
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
  for (let i = 0; i < actualSeries.length; i += 1) {
    const tail = actualSeries.slice(i);
    if (tail.every((value) => Math.abs(value - finalRef) <= tolerance)) {
      settlingTime = time[i];
      break;
    }
  }

  let riseTime = null;
  const riseThreshold = finalRef * 0.9;
  for (let i = 0; i < actualSeries.length; i += 1) {
    if ((finalRef >= 0 && actualSeries[i] >= riseThreshold) || (finalRef < 0 && actualSeries[i] <= riseThreshold)) {
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
  const xWindowSec = config.simMode === "realtime"
    ? Math.max(1, Number(config.realtimeWindowSec) || 8)
    : null;
  const wrapAngleView = config.mode === "cascade" && config.wrapAngle;
  const targetAngleSeries = wrapAngleView
    ? result.targetAngleSeries.map((v) => wrapTo180(v))
    : result.targetAngleSeries;
  const actualAngleSeries = wrapAngleView
    ? result.angleValue.map((v) => wrapTo180(v))
    : result.angleValue;
  const mainSeries = [];
  if (config.showMainTargetAngle) {
    mainSeries.push({ label: "目标角度", color: "#b4492d", values: targetAngleSeries });
  }
  if (config.showMainActualAngle) {
    mainSeries.push({ label: "当前角度", color: "#145b73", values: actualAngleSeries });
  }
  if (config.showMainTargetSpeed) {
    mainSeries.push({ label: "目标速度", color: "#7c3aed", values: result.targetSpeedSeries });
  }
  if (config.showMainActualSpeed) {
    mainSeries.push({ label: "当前速度", color: "#1f7a4f", values: result.speedValue });
  }

  drawLineChart(canvases.main, {
    labels: result.time,
    series: mainSeries,
  }, null, { xWindowSec });

  drawLineChart(canvases.output, {
    labels: result.time,
    series: [
      { label: "PID 输出", color: "#7c3aed", values: result.output },
      { label: "等效电流(A)", color: "#1f7a4f", values: result.currentA },
    ],
  }, null, { xWindowSec });

  drawLineChart(canvases.error, {
    labels: result.time,
    series: [{ label: "误差", color: "#a55d14", values: result.error }],
  }, null, { xWindowSec });
}
function drawLineChart(canvas, data, hoverIndex = null, options = {}) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const padding = { top: 20, right: 18, bottom: 28, left: 44 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fffaf2";
  ctx.fillRect(0, 0, width, height);

  let minX = data.labels[0] ?? 0;
  let maxX = data.labels[data.labels.length - 1] ?? 1;
  if (options.xWindowSec != null) {
    const win = Math.max(1e-6, options.xWindowSec);
    const lastX = data.labels[data.labels.length - 1] ?? 0;
    if (lastX <= win) {
      minX = 0;
      maxX = win;
    } else {
      maxX = lastX;
      minX = lastX - win;
    }
  }

  const visibleValues = [];
  for (const series of data.series) {
    for (let i = 0; i < data.labels.length; i += 1) {
      const t = data.labels[i];
      const v = series.values[i];
      if (t >= minX && t <= maxX && Number.isFinite(v)) {
        visibleValues.push(v);
      }
    }
  }
  const allFiniteValues = data.series
    .flatMap((series) => series.values)
    .filter((v) => Number.isFinite(v));
  const valuesForScale = visibleValues.length > 0 ? visibleValues : allFiniteValues;

  let minY;
  let maxY;
  if (valuesForScale.length === 0) {
    minY = -1;
    maxY = 1;
  } else {
    minY = Math.min(...valuesForScale);
    maxY = Math.max(...valuesForScale);
  }

  if (!Number.isFinite(minY) || !Number.isFinite(maxY) || minY >= maxY) {
    minY = -1;
    maxY = 1;
  }

  if (minY === maxY) {
    minY -= 1;
    maxY += 1;
  }
  const yPad = (maxY - minY) * 0.1;
  minY -= yPad;
  maxY += yPad;

  const safeXSpan = Math.max(maxX - minX, 1e-9);
  chartState.set(canvas.id, { data, minX, maxX, minY, maxY, padding, plotWidth, plotHeight, safeXSpan, options });

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

  ctx.save();
  ctx.beginPath();
  ctx.rect(padding.left, padding.top, plotWidth, plotHeight);
  ctx.clip();
  for (const series of data.series) {
    ctx.strokeStyle = series.color;
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    series.values.forEach((value, index) => {
      if (!Number.isFinite(value)) return;
      const safeValue = Math.max(minY, Math.min(maxY, value));
      const x = padding.left + ((data.labels[index] - minX) / safeXSpan) * plotWidth;
      const y = padding.top + ((maxY - safeValue) / (maxY - minY)) * plotHeight;
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  }
  ctx.restore();

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
      if (!Number.isFinite(v)) continue;
      const safeValue = Math.max(minY, Math.min(maxY, v));
      const y = padding.top + ((maxY - safeValue) / (maxY - minY)) * plotHeight;
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
      drawLineChart(canvas, state.data, null, state.options || {});
      hideChartTooltip();
      return;
    }

    const ratio = (x - minPlotX) / (maxPlotX - minPlotX);
    const xValue = state.minX + ratio * state.safeXSpan;
    const index = findNearestLabelIndex(state.data.labels, xValue);
    drawLineChart(canvas, state.data, index, state.options || {});
    showChartTooltip(state.data, index, event.clientX, event.clientY);
  });

  canvas.addEventListener("mouseleave", () => {
    const state = chartState.get(canvas.id);
    if (state) drawLineChart(canvas, state.data, null, state.options || {});
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

function getLoadOpposeDirection(speedRpm, driveTorque) {
  const speedEps = 0.5;
  const torqueEps = 1e-5;
  if (Math.abs(speedRpm) > speedEps) {
    return Math.sign(speedRpm);
  }
  if (Math.abs(driveTorque) > torqueEps) {
    return Math.sign(driveTorque);
  }
  return 0;
}

function format(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function findNearestLabelIndex(labels, target) {
  if (!labels.length) return 0;
  let lo = 0;
  let hi = labels.length - 1;
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (labels[mid] < target) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  if (lo === 0) return 0;
  const prev = lo - 1;
  return Math.abs(labels[lo] - target) < Math.abs(labels[prev] - target) ? lo : prev;
}

function bindMainSeriesToggleRepaint() {
  [
    "showMainTargetAngle",
    "showMainActualAngle",
    "showMainTargetSpeed",
    "showMainActualSpeed",
  ].forEach((id) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("change", () => {
      if (!lastRenderSnapshot) return;
      const viewConfig = {
        ...lastRenderSnapshot.config,
        ...readMainSeriesToggleValues(),
      };
      lastRenderSnapshot.config = viewConfig;
      renderCharts(viewConfig, lastRenderSnapshot.result);
    });
  });
}

function readMainSeriesToggleValues() {
  const read = (id) => Boolean(document.getElementById(id)?.checked);
  return {
    showMainTargetAngle: read("showMainTargetAngle"),
    showMainActualAngle: read("showMainActualAngle"),
    showMainTargetSpeed: read("showMainTargetSpeed"),
    showMainActualSpeed: read("showMainActualSpeed"),
  };
}


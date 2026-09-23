/* app.js — page wiring for the static demo published with this repository.
 *
 * The three models live in engine.js, which is a line-by-line mirror of the
 * repository's Python files. This file only connects them to the DOM: tab
 * switching, the exploration animation, the latency and link-budget panels,
 * and the verification table that compares the browser results against the
 * committed numbers in data/reference.json.
 *
 * Nothing on this page is a measurement. Every figure is either a model
 * output or a data-sheet-level assumption, and the verification table exists
 * so that the two implementations of each model can be checked against each
 * other in the browser itself.
 */
(function () {
  "use strict";

  const E = window.FS2S;
  if (!E) {
    return;
  }

  const TOL = 1e-9;
  const BYTES_PER_CELL = 2; /* payload of one grid-cell index, used only for the order-of-magnitude uplink readout */
  const CONTROL_HZ = 5; /* assumed control-loop rate, used only for the same readout */
  const STEP_SECONDS = 1 / CONTROL_HZ;
  const ANIM_MS = 34;

  const COL = {
    unexplored: "#0e1729",
    obstacle: "#24314f",
    server: "#38d39f",
    local: "#f2b544",
    robot: "#4da3ff",
    axis: "rgba(255,255,255,0.18)",
    grid: "rgba(255,255,255,0.06)",
    text: "rgba(255,255,255,0.60)",
    faint: "rgba(255,255,255,0.35)",
    warn: "#ff7a7a",
    raw: "#8b93a7",
    down: "#4da3ff",
    sparse: "#38d39f"
  };

  const REF = { data: null, ok: false };
  const PARITY = { checked: 0, failed: 0 };

  function q(id) {
    return document.getElementById(id);
  }

  function pct(x, digits) {
    return (100 * x).toFixed(digits === undefined ? 1 : digits) + " %";
  }

  function sleep(ms) {
    return new Promise(function (resolve) {
      setTimeout(resolve, ms || 0);
    });
  }

  /* ------------------------------------------------------------- canvas -- */

  function prep(canvas, height) {
    const dpr = window.devicePixelRatio || 1;
    const host = canvas.parentElement || canvas;
    const w = Math.max(260, Math.floor(host.clientWidth || canvas.clientWidth || 640));
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(height * dpr);
    canvas.style.width = w + "px";
    canvas.style.height = height + "px";
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, height);
    return { ctx: ctx, w: w, h: height };
  }

  /* Generic line plot: axes, optional bands, series with markers, notes. */
  function plot(ctx, w, h, cfg) {
    const pad = cfg.pad || { l: 56, r: 18, t: 16, b: 34 };
    const iw = w - pad.l - pad.r;
    const ih = h - pad.t - pad.b;
    const X = function (x) {
      return pad.l + ((x - cfg.xmin) / (cfg.xmax - cfg.xmin)) * iw;
    };
    const Y = function (y) {
      return h - pad.b - ((y - cfg.ymin) / (cfg.ymax - cfg.ymin)) * ih;
    };

    ctx.save();
    ctx.font = "11px ui-monospace, Menlo, Consolas, monospace";

    (cfg.yTicks || []).forEach(function (t) {
      const y = Y(t.value);
      ctx.strokeStyle = COL.grid;
      ctx.beginPath();
      ctx.moveTo(pad.l, y);
      ctx.lineTo(w - pad.r, y);
      ctx.stroke();
      ctx.fillStyle = COL.text;
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      ctx.fillText(t.label, pad.l - 8, y);
    });

    (cfg.xTicks || []).forEach(function (t) {
      const x = X(t.value);
      ctx.strokeStyle = COL.grid;
      ctx.beginPath();
      ctx.moveTo(x, pad.t);
      ctx.lineTo(x, h - pad.b);
      ctx.stroke();
      ctx.fillStyle = COL.text;
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      ctx.fillText(t.label, x, h - pad.b + 6);
    });

    ctx.strokeStyle = COL.axis;
    ctx.strokeRect(pad.l + 0.5, pad.t + 0.5, iw, ih);

    (cfg.bands || []).forEach(function (b) {
      ctx.fillStyle = b.color;
      ctx.beginPath();
      b.upper.forEach(function (p, i) {
        const x = X(p[0]);
        const y = Y(p[1]);
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      for (let i = b.lower.length - 1; i >= 0; i--) {
        ctx.lineTo(X(b.lower[i][0]), Y(b.lower[i][1]));
      }
      ctx.closePath();
      ctx.fill();
    });

    (cfg.series || []).forEach(function (s) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = s.width || 2;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath();
      s.points.forEach(function (p, i) {
        const x = X(p[0]);
        const y = Y(p[1]);
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
      ctx.setLineDash([]);
      (s.markers || []).forEach(function (p) {
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(X(p[0]), Y(p[1]), 4, 0, Math.PI * 2);
        ctx.fill();
      });
    });

    if (cfg.xLabel) {
      ctx.fillStyle = COL.faint;
      ctx.textAlign = "right";
      ctx.textBaseline = "bottom";
      ctx.fillText(cfg.xLabel, w - pad.r, h - 2);
    }
    if (cfg.yLabel) {
      ctx.fillStyle = COL.faint;
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(cfg.yLabel, 4, 2);
    }

    (cfg.notes || []).forEach(function (n) {
      ctx.fillStyle = n.color || COL.text;
      ctx.font = "12px system-ui, -apple-system, sans-serif";
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(n.text, X(n.x) + 6, Y(n.y) - 14);
      ctx.font = "11px ui-monospace, Menlo, Consolas, monospace";
    });

    ctx.restore();
  }

  function yTicksLinear(min, max, count, format) {
    const out = [];
    for (let i = 0; i <= count; i++) {
      const value = min + ((max - min) * i) / count;
      out.push({ value: value, label: format(value) });
    }
    return out;
  }

  /* --------------------------------------------------------------- tabs -- */

  function initTabs() {
    const buttons = Array.prototype.slice.call(document.querySelectorAll("nav.tabs button[data-panel]"));
    const panels = Array.prototype.slice.call(document.querySelectorAll("section.panel"));

    function show(name) {
      panels.forEach(function (p) {
        p.classList.toggle("active", p.id === "panel-" + name);
      });
      buttons.forEach(function (b) {
        b.setAttribute("aria-selected", b.getAttribute("data-panel") === name ? "true" : "false");
      });
      redraw();
    }

    buttons.forEach(function (b) {
      b.addEventListener("click", function () {
        const name = b.getAttribute("data-panel");
        if (window.history && window.history.replaceState) {
          window.history.replaceState(null, "", "#" + name);
        }
        show(name);
      });
    });

    const initial = (window.location.hash || "").replace("#", "");
    show(buttons.some(function (b) { return b.getAttribute("data-panel") === initial; }) ? initial : "overview");
  }

  /* -------------------------------------------------------- exploration -- */

  const X = {
    grid: null,
    sim: null,
    step: 0,
    strategy: "diff",
    budget: 8,
    running: false,
    raf: null,
    last: 0,
    ref: null
  };

  function newSim() {
    const meta = X.ref.meta;
    const budget =
      X.strategy === "full" ? E.constants.ROWS * E.constants.COLS : X.budget;
    return new E.Exploration(X.strategy, budget, meta.seed + 1, X.grid);
  }

  function drawGrid() {
    const canvas = q("gridCanvas");
    if (!canvas || !X.grid) {
      return;
    }
    const host = canvas.parentElement || canvas;
    const size = Math.max(260, Math.floor(host.clientWidth || 420));
    const r = prep(canvas, size);
    const ctx = r.ctx;
    const rows = E.constants.ROWS;
    const cols = E.constants.COLS;
    const cell = size / cols;

    ctx.fillStyle = COL.unexplored;
    ctx.fillRect(0, 0, size, size);

    const server = X.sim ? X.sim.serverMap() : new Set();
    const local = X.sim ? X.sim.localView() : new Set();

    for (let i = 0; i < X.grid.length; i++) {
      const rr = Math.floor(i / cols);
      const cc = i % cols;
      let color = null;
      if (X.grid[i] === 1) {
        color = COL.obstacle;
      }
      if (local.has(i)) {
        color = COL.local;
      }
      if (server.has(i)) {
        color = COL.server;
      }
      if (color) {
        ctx.fillStyle = color;
        ctx.fillRect(cc * cell, rr * cell, Math.ceil(cell), Math.ceil(cell));
      }
    }

    ctx.strokeStyle = COL.grid;
    ctx.lineWidth = 1;
    for (let i = 0; i <= cols; i += 5) {
      ctx.beginPath();
      ctx.moveTo(i * cell, 0);
      ctx.lineTo(i * cell, size);
      ctx.moveTo(0, i * cell);
      ctx.lineTo(size, i * cell);
      ctx.stroke();
    }

    if (X.sim) {
      X.sim.positions.forEach(function (pos, index) {
        const rr = Math.floor(pos / cols);
        const cc = pos % cols;
        ctx.fillStyle = COL.robot;
        ctx.beginPath();
        ctx.arc((cc + 0.5) * cell, (rr + 0.5) * cell, Math.max(3, cell * 0.42), 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#0b111c";
        ctx.font = "bold " + Math.max(8, Math.round(cell * 0.7)) + "px ui-monospace, monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(index + 1), (cc + 0.5) * cell, (rr + 0.5) * cell);
      });
    }
  }

  function drawCoverage() {
    const canvas = q("coverageCanvas");
    if (!canvas || !X.sim) {
      return;
    }
    const r = prep(canvas, 300);
    const ctx = r.ctx;
    const steps = E.constants.STEPS;
    const server = X.sim.coverage.map(function (v, i) { return [i + 1, v]; });
    const local = X.sim.localCoverage.map(function (v, i) { return [i + 1, v]; });

    plot(ctx, r.w, r.h, {
      xmin: 0,
      xmax: steps,
      ymin: 0,
      ymax: 1,
      xTicks: [0, 50, 100, 150, 200].map(function (v) { return { value: v, label: String(v) }; }),
      yTicks: yTicksLinear(0, 1, 5, function (v) { return (100 * v).toFixed(0) + "%"; }),
      xLabel: "simulation step",
      yLabel: "coverage",
      series: [
        { points: local, color: COL.local, width: 2 },
        { points: server, color: COL.server, width: 2 }
      ],
      notes: [
        { x: steps * 0.06, y: 0.93, text: "on-board", color: COL.local },
        { x: steps * 0.06, y: 0.93 - 0.08, text: "server", color: COL.server }
      ]
    });
  }

  function drawSweep() {
    const canvas = q("sweepCanvas");
    if (!canvas) {
      return;
    }
    const r = prep(canvas, 260);
    const ctx = r.ctx;
    const sweep = (REF.data && REF.data.exploration.sweep) || [];
    if (!sweep.length) {
      return;
    }
    const pad = { l: 56, r: 18, t: 16, b: 34 };
    const iw = r.w - pad.l - pad.r;
    const ih = r.h - pad.t - pad.b;
    const xs = sweep.map(function (e) { return e.budget_cells_per_step; });
    const xmin = Math.min.apply(null, xs);
    const xmax = Math.max.apply(null, xs);
    const bar = iw / (xmax - xmin + 1);

    ctx.save();
    ctx.font = "11px ui-monospace, Menlo, Consolas, monospace";
    yTicksLinear(0, 1, 5, function (v) { return (100 * v).toFixed(0) + "%"; }).forEach(function (t) {
      const y = r.h - pad.b - t.value * ih;
      ctx.strokeStyle = COL.grid;
      ctx.beginPath();
      ctx.moveTo(pad.l, y);
      ctx.lineTo(r.w - pad.r, y);
      ctx.stroke();
      ctx.fillStyle = COL.text;
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      ctx.fillText(t.label, pad.l - 8, y);
    });
    ctx.strokeStyle = COL.axis;
    ctx.strokeRect(pad.l + 0.5, pad.t + 0.5, iw, ih);

    sweep.forEach(function (e) {
      const h = e.final_coverage * ih;
      const x = pad.l + (e.budget_cells_per_step - xmin) * bar + bar * 0.15;
      ctx.fillStyle = e.budget_cells_per_step === X.budget ? COL.local : "rgba(77,163,255,0.65)";
      ctx.fillRect(x, r.h - pad.b - h, bar * 0.7, h);
    });

    [1, 5, 10, 15, 20].forEach(function (v) {
      if (v < xmin || v > xmax) {
        return;
      }
      ctx.fillStyle = COL.text;
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      ctx.fillText(String(v), pad.l + (v - xmin) * bar + bar * 0.5, r.h - pad.b + 6);
    });

    ctx.fillStyle = COL.faint;
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText("cells per robot per step", r.w - pad.r, r.h - 2);
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("final server coverage after 200 steps", 4, 2);
    ctx.restore();
  }

  function fillRefTable() {
    const body = q("refTable") && q("refTable").querySelector("tbody");
    if (!body || !REF.data) {
      return;
    }
    const runs = REF.data.exploration.runs;
    const names = Object.keys(runs).sort(function (a, b) {
      return runs[a].budget_cells_per_step - runs[b].budget_cells_per_step;
    });
    body.innerHTML = "";
    names.forEach(function (name) {
      const run = runs[name];
      const tr = document.createElement("tr");
      const label = run.strategy === "full" ? "整張地圖上傳" : "差異更新，預算 " + run.budget_cells_per_step + " 格";
      tr.innerHTML =
        "<td>" + label + "</td>" +
        '<td class="num">' + run.budget_cells_per_step + "</td>" +
        '<td class="num">' + pct(run.final_coverage) + "</td>" +
        '<td class="num">' + run.transmitted_cells.toLocaleString("en-US") + "</td>";
      body.appendChild(tr);
    });
  }

  function updateStats() {
    if (!X.sim) {
      return;
    }
    const steps = E.constants.STEPS;
    const server = X.sim.coverage.length ? X.sim.coverage[X.sim.coverage.length - 1] : 0;
    const local = X.sim.localCoverage.length ? X.sim.localCoverage[X.sim.localCoverage.length - 1] : 0;
    const budget = X.strategy === "full" ? E.constants.ROWS * E.constants.COLS : X.budget;
    const kbps = (budget * BYTES_PER_CELL * 8 * CONTROL_HZ) / 1000;

    q("statStep").textContent = X.step + " / " + steps;
    q("statTime").textContent = (X.step * STEP_SECONDS).toFixed(1) + " s";
    q("statServer").textContent = pct(server);
    q("statLocal").textContent = pct(local);
    q("statGap").textContent = (100 * (local - server)).toFixed(1) + " pp";
    q("statTx").textContent = X.sim.transmittedCells.toLocaleString("en-US");
    q("statBacklog").textContent = X.sim.backlogCells.toLocaleString("en-US");
    q("statKbps").textContent = kbps.toFixed(2).replace(/\.00$/, "") + " kbps";
  }

  function advance(count) {
    const steps = E.constants.STEPS;
    if (!X.sim) {
      return;
    }
    for (let i = 0; i < count && X.step < steps; i++) {
      X.sim.step();
      X.step += 1;
    }
    drawGrid();
    drawCoverage();
    updateStats();
    if (X.step >= steps) {
      setRunning(false);
      q("runBtn").textContent = "執行";
    }
  }

  function setRunning(value) {
    X.running = value;
    if (value) {
      X.last = 0;
      X.raf = window.requestAnimationFrame(tick);
    } else if (X.raf) {
      window.cancelAnimationFrame(X.raf);
      X.raf = null;
    }
  }

  function tick(ts) {
    if (!X.running) {
      return;
    }
    if (!X.last || ts - X.last >= ANIM_MS) {
      X.last = ts;
      advance(1);
    }
    if (X.running) {
      X.raf = window.requestAnimationFrame(tick);
    }
  }

  function resetSim() {
    setRunning(false);
    X.step = 0;
    X.sim = newSim();
    q("runBtn").textContent = "執行 / 暫停";
    drawGrid();
    drawCoverage();
    updateStats();
  }

  function initExploration() {
    const meta = X.ref.meta;
    X.grid = E.buildGrid(new E.Rng(meta.seed));
    X.sim = newSim();
    X.step = 0;

    q("strategy").addEventListener("change", function () {
      X.strategy = this.value;
      q("budgetField").style.display = this.value === "full" ? "none" : "";
      resetSim();
      drawSweep();
    });

    q("budget").addEventListener("input", function () {
      X.budget = Number(this.value);
      q("budgetValue").textContent = this.value;
      resetSim();
      drawSweep();
    });

    q("runBtn").addEventListener("click", function () {
      if (X.step >= E.constants.STEPS) {
        resetSim();
      }
      const next = !X.running;
      setRunning(next);
      this.textContent = next ? "暫停" : "繼續";
    });

    q("resetBtn").addEventListener("click", resetSim);

    q("fastBtn").addEventListener("click", function () {
      setRunning(false);
      advance(E.constants.STEPS);
    });

    q("refNote").textContent =
      "模擬時間以每步 " + STEP_SECONDS.toFixed(1) + " s（" + CONTROL_HZ +
      " Hz 控制迴路）換算；uplink 負載以每格 " + BYTES_PER_CELL +
      " bytes 的索引表示估算，兩者都是為了給出量級，不是量測值。已提交的參考結果由 tools/coverage_model.py 以同一組種子產生。";

    fillRefTable();
    drawGrid();
    drawCoverage();
    drawSweep();
    updateStats();
  }

  /* ------------------------------------------------------------- latency -- */

  function drawLatency() {
    const canvas = q("latencyCanvas");
    if (!canvas) {
      return;
    }
    const r = prep(canvas, 340);
    const ctx = r.ctx;
    const rtt = Number(q("rtt").value);
    const speed = Number(q("speed").value);
    const series = [];
    const speeds = [0.5, 1.0, 1.5, 2.0];
    const palette = ["#8b93a7", "#4da3ff", "#38d39f", "#f2b544"];
    const maxErr = 2.0 * 0.5;

    speeds.forEach(function (v, i) {
      const points = [];
      for (let tau = 0; tau <= 500; tau += 10) {
        points.push([tau, E.deadbandError(v, tau)]);
      }
      series.push({
        points: points,
        color: v === speed ? palette[i] : "rgba(255,255,255,0.22)",
        width: v === speed ? 2.4 : 1.4,
        markers: v === speed ? [[rtt, E.deadbandError(v, rtt)]] : []
      });
    });

    const band = { upper: [], lower: [] };
    for (let tau = 0; tau <= 500; tau += 10) {
      band.upper.push([tau, E.deadbandError(1.0, tau + E.JITTER_SIGMA_MS)]);
      band.lower.push([tau, E.deadbandError(1.0, Math.max(0, tau - E.JITTER_SIGMA_MS))]);
    }

    const budget = [];
    for (let tau = 0; tau <= 500; tau += 10) {
      budget.push([tau, E.ACCEPTABLE_ERROR_M]);
    }

    plot(ctx, r.w, r.h, {
      xmin: 0,
      xmax: 500,
      ymin: 0,
      ymax: maxErr,
      xTicks: [0, 100, 200, 300, 400, 500].map(function (v) { return { value: v, label: String(v) }; }),
      yTicks: yTicksLinear(0, maxErr, 4, function (v) { return v.toFixed(2); }),
      xLabel: "round-trip latency tau (ms)",
      yLabel: "dead-band error v*tau (m)",
      bands: [{ color: "rgba(56,211,159,0.10)", upper: band.upper, lower: band.lower }],
      series: series.concat([
        { points: budget, color: COL.warn, width: 1.4, dash: [6, 5] }
      ]),
      notes: [
        { x: 300, y: 0.1, text: "0.1 m tolerance", color: COL.warn },
        { x: 26, y: 0.62, text: "jitter band (sigma = " + E.JITTER_SIGMA_MS + " ms, v = 1.0)", color: COL.sparse }
      ]
    });

    q("latErr").textContent = E.deadbandError(speed, rtt).toFixed(3) + " m";
    q("latMax").textContent = E.maxRttWithin(speed).toFixed(0) + " ms";
    q("latJit").textContent = E.JITTER_SIGMA_MS + " ms";

    const body = q("latencyTable").querySelector("tbody");
    body.innerHTML = "";
    speeds.forEach(function (v) {
      const tr = document.createElement("tr");
      tr.innerHTML =
        "<td>" + v.toFixed(1) + " m/s</td>" +
        '<td class="num">' + E.deadbandError(v, rtt).toFixed(3) + " m</td>" +
        '<td class="num">' + E.maxRttWithin(v).toFixed(1) + " ms</td>";
      body.appendChild(tr);
    });
  }

  /* ---------------------------------------------------------------- link -- */

  function drawLink() {
    const canvas = q("linkCanvas");
    if (!canvas || !REF.data) {
      return;
    }
    const r = prep(canvas, 330);
    const ctx = r.ctx;
    const bandwidth = Number(q("bandwidth").value);
    const curves = REF.data.link_budget.curves;
    const rates = REF.data.link_budget.rates_hz;
    const names = ["raw", "downsampled", "sparse"];
    const palette = { raw: COL.raw, downsampled: COL.down, sparse: COL.sparse };
    const yMax = Math.max(100, Math.ceil(bandwidth / 10) * 10);

    const series = names.map(function (name) {
      return {
        points: rates.map(function (rate, i) { return [rate, curves[name].required_mbps[i]]; }),
        color: palette[name],
        width: 2,
        markers: [[E.achievableRateHz(curves[name].base_mbps_at_10hz, bandwidth), bandwidth]]
      };
    });

    series.push({
      points: [[1, bandwidth], [20, bandwidth]],
      color: COL.warn,
      width: 1.4,
      dash: [6, 5]
    });

    plot(ctx, r.w, r.h, {
      xmin: 1,
      xmax: 20,
      ymin: 0,
      ymax: yMax,
      xTicks: [1, 5, 10, 15, 20].map(function (v) { return { value: v, label: v + " Hz" }; }),
      yTicks: yTicksLinear(0, yMax, 5, function (v) { return v.toFixed(0); }),
      xLabel: "update rate (Hz)",
      yLabel: "required uplink (Mbps)",
      series: series,
      notes: [
        { x: 11.4, y: curves.raw.required_mbps[0], text: "raw point cloud", color: COL.raw },
        { x: 11.4, y: curves.downsampled.required_mbps[0], text: "downsampled", color: COL.down },
        { x: 11.4, y: curves.sparse.required_mbps[0], text: "sparse diff", color: COL.sparse }
      ]
    });

    q("bwValue").textContent = bandwidth + " Mbps";

    const body = q("linkTable").querySelector("tbody");
    body.innerHTML = "";
    names.forEach(function (name) {
      const curve = curves[name];
      const label = { raw: "原始點雲", downsampled: "降採樣 / 特徵", sparse: "佔據網格差異" }[name];
      const achievable = E.achievableRateHz(curve.base_mbps_at_10hz, bandwidth);
      const tr = document.createElement("tr");
      tr.innerHTML =
        "<td>" + label + "</td>" +
        '<td class="num">' + curve.base_mbps_at_10hz.toFixed(1) + " Mbps</td>" +
        '<td class="num">' + (achievable > 0 ? achievable + " Hz" : "無法維持") + "</td>";
      body.appendChild(tr);
    });
  }

  /* ------------------------------------------------------------ verify --- */

  function row(name, ok, samples, detail) {
    PARITY.checked += 1;
    if (!ok) {
      PARITY.failed += 1;
    }
    const body = q("verifyTable").querySelector("tbody");
    const tr = document.createElement("tr");
    tr.innerHTML =
      "<td>" + name + "</td>" +
      '<td class="' + (ok ? "pass" : "fail") + '">' + (ok ? "PASS" : "FAIL") + "</td>" +
      '<td class="num">' + samples + "</td>";
    if (!ok && detail) {
      tr.title = detail;
    }
    body.appendChild(tr);
    return ok;
  }

  function maxDelta(a, b) {
    if (a.length !== b.length) {
      return Infinity;
    }
    let worst = 0;
    for (let i = 0; i < a.length; i++) {
      worst = Math.max(worst, Math.abs(a[i] - b[i]));
    }
    return worst;
  }

  async function runVerification() {
    const body = q("verifyTable").querySelector("tbody");
    body.innerHTML = "";
    const exp = REF.data.exploration;
    const meta = exp.meta;

    const constantsOk =
      E.constants.ROWS === meta.rows &&
      E.constants.COLS === meta.cols &&
      E.constants.STEPS === meta.steps &&
      E.constants.SEED === meta.seed &&
      E.constants.SENSOR_RADIUS === meta.sensor_radius &&
      E.constants.N_ROBOTS === meta.robots &&
      Math.abs(E.constants.OBSTACLE_PROB - meta.obstacle_prob) <= TOL &&
      Math.abs(E.constants.RANDOM_STEP_PROB - meta.random_step_prob) <= TOL;
    row("模型常數（" + meta.rows + "×" + meta.cols + "／" + meta.steps + " 步／種子 " + meta.seed + "）", constantsOk, 8);
    await sleep(0);

    let gridString = "";
    for (let i = 0; i < X.grid.length; i++) {
      gridString += X.grid[i];
    }
    row("障礙物網格（" + X.grid.length + " 格逐格比對）", gridString === exp.grid, X.grid.length);

    const free = E.freeCells(X.grid);
    const startsOk =
      free.length === exp.reachable_cells &&
      exp.start_indices.every(function (idx, i) {
        const cell = free[Math.floor(free.length / E.constants.N_ROBOTS) * i];
        return cell[0] * E.constants.COLS + cell[1] === idx;
      });
    row("可達格數與起始位置", startsOk, 3);
    await sleep(0);

    const names = Object.keys(exp.runs);
    for (let i = 0; i < names.length; i++) {
      const name = names[i];
      const run = exp.runs[name];
      const derived = E.runExperiment(X.grid, run.strategy, run.budget_cells_per_step, meta.seed + 1);
      const d =
        Math.max(
          maxDelta(derived.coverage, run.coverage),
          maxDelta(derived.localCoverage, run.local_coverage)
        );
      const ok = d <= TOL && derived.transmittedCells === run.transmitted_cells && derived.backlogCells === run.backlog_cells;
      row(
        "執行 " + name + "（覆蓋率序列、" + run.coverage.length + " 步）",
        ok,
        run.coverage.length * 2 + 1,
        ok ? "" : "max |delta| = " + d
      );
      await sleep(0);
    }

    let sweepOk = true;
    let sweepDetail = "";
    for (let i = 0; i < exp.sweep.length; i++) {
      const entry = exp.sweep[i];
      const derived = E.runExperiment(X.grid, "diff", entry.budget_cells_per_step, meta.seed + 1);
      if (
        Math.abs(derived.finalCoverage - entry.final_coverage) > TOL ||
        derived.transmittedCells !== entry.transmitted_cells
      ) {
        sweepOk = false;
        sweepDetail = "budget " + entry.budget_cells_per_step + " 不一致";
      }
      if (i % 4 === 3) {
        await sleep(0);
      }
    }
    row("每步預算掃描（" + exp.sweep.length + " 組預算）", sweepOk, exp.sweep.length, sweepDetail);
    await sleep(0);

    const lat = REF.data.latency;
    let latOk = true;
    let latSamples = 0;
    Object.keys(lat.error_m).forEach(function (key) {
      const speed = Number(key);
      const derived = lat.rtt_ms.map(function (tau) { return E.deadbandError(speed, tau); });
      if (maxDelta(derived, lat.error_m[key]) > TOL) {
        latOk = false;
      }
      latSamples += derived.length;
    });
    row("延遲模型 v·tau（" + Object.keys(lat.error_m).length + " 種速度 × " + lat.rtt_ms.length + " 個延遲）", latOk, latSamples);

    const rttMaxOk = lat.speeds_mps.every(function (speed) {
      return Math.abs(E.maxRttWithin(speed) - lat.max_rtt_within_acceptable_ms[speed.toFixed(1)]) <= TOL;
    });
    row("0.1 m 容許下的延遲上限", rttMaxOk, lat.speeds_mps.length);
    await sleep(0);

    const link = REF.data.link_budget;
    let linkOk = true;
    let linkSamples = 0;
    Object.keys(link.curves).forEach(function (name) {
      const curve = link.curves[name];
      const derived = link.rates_hz.map(function (rate) {
        return E.requiredMbps(curve.base_mbps_at_10hz, rate);
      });
      if (maxDelta(derived, curve.required_mbps) > TOL) {
        linkOk = false;
      }
      linkSamples += derived.length;
    });
    row("鏈路需求曲線（" + Object.keys(link.curves).length + " 種資料流 × " + link.rates_hz.length + " 個更新率）", linkOk, linkSamples);

    let rateOk = true;
    let rateSamples = 0;
    link.achievable_rate_hz.forEach(function (entry) {
      Object.keys(link.curves).forEach(function (name) {
        const derived = E.achievableRateHz(link.curves[name].base_mbps_at_10hz, entry.bandwidth_mbps, 30);
        if (derived !== entry[name]) {
          rateOk = false;
        }
        rateSamples += 1;
      });
    });
    row("給定頻寬下的可達更新率", rateOk, rateSamples);

    const tag = q("tag-parity");
    if (tag) {
      if (PARITY.failed === 0) {
        tag.textContent = "JS ↔ Python 對照：" + PARITY.checked + " 項全部一致";
        tag.classList.remove("warn");
      } else {
        tag.textContent = "JS ↔ Python 對照：" + PARITY.failed + " / " + PARITY.checked + " 項不一致";
        tag.classList.add("warn");
      }
    }
  }

  /* ------------------------------------------------------------- resize -- */

  let resizeTimer = null;
  function redraw() {
    drawGrid();
    drawCoverage();
    drawSweep();
    drawLatency();
    drawLink();
  }

  function initResize() {
    window.addEventListener("resize", function () {
      if (resizeTimer) {
        window.clearTimeout(resizeTimer);
      }
      resizeTimer = window.setTimeout(redraw, 160);
    });
  }

  /* --------------------------------------------------------------- boot -- */

  function registerWorker() {
    if (!("serviceWorker" in navigator) || window.location.protocol === "file:") {
      return;
    }
    navigator.serviceWorker.register("sw.js").catch(function () {
      /* the page works without the offline cache; nothing to report */
    });
  }

  function panic(message) {
    const tag = q("tag-parity");
    if (tag) {
      tag.textContent = message;
      tag.classList.add("warn");
    }
    const body = q("verifyTable") && q("verifyTable").querySelector("tbody");
    if (body) {
      body.innerHTML = '<tr><td colspan="3">' + message + "</td></tr>";
    }
  }

  function boot() {
    initTabs();

    fetch("data/reference.json", { cache: "no-cache" })
      .then(function (res) {
        if (!res.ok) {
          throw new Error("HTTP " + res.status);
        }
        return res.json();
      })
      .then(function (data) {
        REF.data = data;
        REF.ok = true;
        X.ref = data.exploration;
        initExploration();
        initResize();
        q("rtt").addEventListener("input", drawLatency);
        q("speed").addEventListener("input", drawLatency);
        q("bandwidth").addEventListener("input", drawLink);
        drawLatency();
        drawLink();
        return runVerification();
      })
      .catch(function (err) {
        panic("無法載入 data/reference.json（" + err.message + "）：驗證表與滑桿的參考標記不可用。");
      });

    registerWorker();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", boot);
  } else {
    boot();
  }
})();

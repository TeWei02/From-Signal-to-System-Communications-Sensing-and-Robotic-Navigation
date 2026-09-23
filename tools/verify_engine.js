/* verify_engine.js — cross-language parity harness.
 *
 * Run with the JavaScriptCore shell (no Node.js required):
 *
 *     jsc tools/verify_engine.js
 *
 * It loads the published browser engine (docs/engine.js), re-derives the whole
 * reference dataset from the shared seed, and compares it with the numbers the
 * Python models wrote to docs/data/reference.json. Any disagreement is printed
 * as a FAIL line; the last line is either RESULT: PASS or RESULT: FAIL <n>.
 */
"use strict";

load("docs/engine.js");
const ref = JSON.parse(readFile("docs/data/reference.json"));
const E = globalThis.FS2S;
const failures = [];
const TOL = 1e-9;

function check(condition, message) {
  if (!condition) failures.push(message);
}

function seriesEqual(a, b, message) {
  if (a.length !== b.length) {
    failures.push(message + ": length " + a.length + " != " + b.length);
    return;
  }
  let worst = 0;
  for (let i = 0; i < a.length; i++) worst = Math.max(worst, Math.abs(a[i] - b[i]));
  check(worst <= TOL, message + ": max |delta| " + worst.toExponential(3));
}

/* ---- 1. constants ------------------------------------------------------ */
const exp = ref.exploration;
check(E.constants.ROWS === exp.meta.rows, "ROWS mismatch");
check(E.constants.COLS === exp.meta.cols, "COLS mismatch");
check(E.constants.STEPS === exp.meta.steps, "STEPS mismatch");
check(E.constants.SEED === exp.meta.seed, "SEED mismatch");
check(E.constants.SENSOR_RADIUS === exp.meta.sensor_radius, "SENSOR_RADIUS mismatch");
check(E.constants.N_ROBOTS === exp.meta.robots, "N_ROBOTS mismatch");
check(Math.abs(E.constants.OBSTACLE_PROB - exp.meta.obstacle_prob) < TOL, "OBSTACLE_PROB mismatch");
check(Math.abs(E.constants.RANDOM_STEP_PROB - exp.meta.random_step_prob) < TOL, "RANDOM_STEP_PROB mismatch");

/* ---- 2. grid and start positions --------------------------------------- */
const grid = E.buildGrid(new E.Rng(exp.meta.seed));
let gridString = "";
for (let i = 0; i < grid.length; i++) gridString += grid[i];
check(gridString === exp.grid, "generated grid differs from the Python grid");

const free = E.freeCells(grid);
check(free.length === exp.reachable_cells,
  "reachable cells " + free.length + " != " + exp.reachable_cells);
const step = Math.floor(free.length / E.constants.N_ROBOTS);
for (let i = 0; i < E.constants.N_ROBOTS; i++) {
  const cell = free[i * step];
  const index = cell[0] * E.constants.COLS + cell[1];
  check(index === exp.start_indices[i], "start index " + i + " mismatch");
}

/* ---- 3. the exploration runs ------------------------------------------ */
for (const name of Object.keys(exp.runs)) {
  const run = exp.runs[name];
  const budget = run.budget_cells_per_step;
  const derived = E.runExperiment(grid, run.strategy, budget, exp.meta.seed + 1);
  seriesEqual(derived.coverage, run.coverage, name + ".coverage");
  seriesEqual(derived.localCoverage, run.local_coverage, name + ".local_coverage");
  check(derived.transmittedCells === run.transmitted_cells,
    name + ".transmitted_cells " + derived.transmittedCells + " != " + run.transmitted_cells);
}

/* ---- 4. the budget sweep --------------------------------------------- */
exp.sweep.forEach((entry) => {
  const derived = E.runExperiment(grid, "diff", entry.budget_cells_per_step, exp.meta.seed + 1);
  check(Math.abs(derived.finalCoverage - entry.final_coverage) <= TOL,
    "sweep " + entry.budget_cells_per_step + ": coverage differs");
  check(derived.transmittedCells === entry.transmitted_cells,
    "sweep " + entry.budget_cells_per_step + ": transmitted cells differ");
});

/* ---- 5. latency model ------------------------------------------------- */
const lat = ref.latency;
lat.rtt_ms.forEach((tau, i) => {
  lat.speeds_mps.forEach((v) => {
    const key = v.toFixed(1);
    check(Math.abs(E.deadbandError(v, tau) - lat.error_m[key][i]) <= TOL,
      "latency error mismatch at v=" + v + " tau=" + tau);
  });
});
lat.speeds_mps.forEach((v) => {
  const key = v.toFixed(1);
  check(Math.abs(E.maxRttWithin(v, lat.acceptable_error_m) - lat.max_rtt_within_acceptable_ms[key]) <= TOL,
    "max RTT mismatch at v=" + v);
});
check(Math.abs(E.JITTER_SIGMA_MS - lat.jitter.sigma_ms) < TOL, "jitter sigma mismatch");

/* ---- 6. link budget -------------------------------------------------- */
const link = ref.link_budget;
Object.keys(link.curves).forEach((name) => {
  const curve = link.curves[name];
  link.rates_hz.forEach((rate, i) => {
    check(Math.abs(E.requiredMbps(curve.base_mbps_at_10hz, rate) - curve.required_mbps[i]) <= TOL,
      "link budget mismatch for " + name + " at " + rate + " Hz");
  });
});
link.achievable_rate_hz.forEach((row) => {
  Object.keys(link.curves).forEach((name) => {
    const achieved = E.achievableRateHz(link.curves[name].base_mbps_at_10hz, row.bandwidth_mbps);
    check(achieved === row[name], "achievable rate mismatch for " + name + " at " + row.bandwidth_mbps + " Mbps");
  });
});

/* ---- report ---------------------------------------------------------- */
if (failures.length === 0) {
  print("checked " + Object.keys(exp.runs).length + " exploration runs x " + exp.runs.full.coverage.length +
        " steps, " + exp.sweep.length + " sweep points, " + lat.rtt_ms.length * lat.speeds_mps.length +
        " latency samples and " + link.rates_hz.length * Object.keys(link.curves).length + " link-budget samples");
  print("browser engine and Python models agree to " + TOL.toExponential(0));
  print("RESULT: PASS");
} else {
  failures.forEach((f) => print("FAIL " + f));
  print("RESULT: FAIL " + failures.length);
}

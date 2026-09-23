/* engine.js — browser-side mirror of the repository's Python models.
 *
 * Three models are ported here, one to one:
 *
 *   1. tools/coverage_model.py        grid-world exploration under a per-step
 *                                     transmission budget (the case study)
 *   2. analysis/plot_latency_vs_nav_error.py       dead-band latency model
 *   3. analysis/plot_bandwidth_vs_update_rate.py   LiDAR link budget
 *
 * The port is exact, not approximate: the random stream is a Mulberry32
 * generator, whose 32-bit integer recurrence is identical in Python and
 * JavaScript, and the experiment consumes the stream call for call in the same
 * order. tools/verify_engine.js re-runs this file under JavaScriptCore and
 * compares the result with the Python output, and the published page repeats
 * that comparison in the browser against docs/data/reference.json.
 */
(function (root) {
  "use strict";

  const ENGINE_VERSION = "1.0.0";

  /* ---- environment definition (must match tools/coverage_model.py) ------- */
  const ROWS = 40;
  const COLS = 40;
  const OBSTACLE_PROB = 0.15;
  const N_ROBOTS = 2;
  const STEPS = 200;
  const SEED = 20260922;
  const SENSOR_RADIUS = 3;
  const RANDOM_STEP_PROB = 0.25;
  const NEIGHBOUR_ORDER = [[-1, 0], [1, 0], [0, -1], [0, 1]];
  const TWO32 = 4294967296;

  /* ---- Mulberry32 ------------------------------------------------------- */
  function mulberry32(seed) {
    let a = seed | 0;
    return function () {
      a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / TWO32;
    };
  }

  class Rng {
    constructor(seed) {
      this.stream = mulberry32(seed & 0xffffffff);
    }
    nextFloat() {
      return this.stream();
    }
    /* Uniform integer in [0, n), same mapping as the Python `index()`. Note
       that n === 1 still consumes one draw, as it does in Python. */
    index(n) {
      if (n <= 0) return 0;
      return Math.min(Math.floor(this.nextFloat() * n), n - 1);
    }
  }

  /* ---- grid world ------------------------------------------------------- */
  function buildGrid(rng) {
    const grid = new Uint8Array(ROWS * COLS);
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) {
        let occupied = rng.nextFloat() < OBSTACLE_PROB ? 1 : 0;
        if (r === 0 || r === ROWS - 1 || c === 0 || c === COLS - 1) occupied = 0;
        grid[r * COLS + c] = occupied;
      }
    }
    return grid;
  }

  function freeCells(grid) {
    const out = [];
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) if (grid[r * COLS + c] === 0) out.push([r, c]);
    }
    return out;
  }

  function sensingWindow(grid, r0, c0, out) {
    const cells = out || [];
    cells.length = 0;
    const rLo = Math.max(0, r0 - SENSOR_RADIUS);
    const rHi = Math.min(ROWS, r0 + SENSOR_RADIUS + 1);
    const cLo = Math.max(0, c0 - SENSOR_RADIUS);
    const cHi = Math.min(COLS, c0 + SENSOR_RADIUS + 1);
    for (let r = rLo; r < rHi; r++) {
      for (let c = cLo; c < cHi; c++) {
        if (grid[r * COLS + c] === 0) cells.push(r * COLS + c);
      }
    }
    return cells;
  }

  function neighbours(grid, r, c) {
    const out = [];
    for (let k = 0; k < NEIGHBOUR_ORDER.length; k++) {
      const rr = r + NEIGHBOUR_ORDER[k][0];
      const cc = c + NEIGHBOUR_ORDER[k][1];
      if (rr < 0 || rr >= ROWS || cc < 0 || cc >= COLS) continue;
      if (grid[rr * COLS + cc] !== 0) continue;
      out.push(rr * COLS + cc);
    }
    return out;
  }

  /* ---- exploration experiment ------------------------------------------ */
  /* Stepwise API: the animation and the reference run use exactly the same
     code path, so what the page shows is what it verifies. */
  class Exploration {
    constructor(strategy, budgetCells, seed, grid) {
      if (strategy !== "full" && strategy !== "diff") {
        throw new Error("unknown strategy: " + strategy);
      }
      this.strategy = strategy;
      this.budgetCells = budgetCells;
      this.grid = grid || buildGrid(new Rng(SEED));
      this.rng = new Rng(seed);
      this.rng.nextFloat(); // separation draw, as in the Python model

      const free = freeCells(this.grid);
      this.free = free;
      this.reachable = free.length;
      const step = Math.floor(this.reachable / N_ROBOTS);
      this.starts = [];
      for (let i = 0; i < N_ROBOTS; i++) this.starts.push(free[i * step]);

      /* positions are cell indices (r * COLS + c) from here on */
      this.positions = this.starts.map((cell) => cell[0] * COLS + cell[1]);
      this.localMaps = this.positions.map(() => new Set());
      this.received = this.positions.map(() => new Set());
      for (let i = 0; i < N_ROBOTS; i++) {
        const pos = this.positions[i];
        for (const cell of sensingWindow(this.grid, (pos / COLS) | 0, pos % COLS)) {
          this.localMaps[i].add(cell);
        }
      }

      this.coverage = [];
      this.localCoverage = [];
      this.transmittedCells = 0;
      this.backlogCells = 0;
      this.stepIndex = 0;
      this._window = [];
    }

    /* One simulation step: every robot moves, senses and transmits. */
    step() {
      for (let i = 0; i < N_ROBOTS; i++) {
        const pos = this.positions[i];
        const candidates = neighbours(this.grid, (pos / COLS) | 0, pos % COLS);
        if (candidates.length > 0) {
          let choice;
          if (this.rng.nextFloat() < RANDOM_STEP_PROB || candidates.length === 1) {
            choice = candidates[this.rng.index(candidates.length)];
          } else {
            let best = -1;
            let bestSet = [];
            for (const cand of candidates) {
              const window = sensingWindow(this.grid, (cand / COLS) | 0, cand % COLS, this._window.slice());
              let unseen = 0;
              for (const cell of window) if (!this.localMaps[i].has(cell)) unseen++;
              if (unseen > best) {
                best = unseen;
                bestSet = [cand];
              } else if (unseen === best) {
                bestSet.push(cand);
              }
            }
            choice = bestSet[this.rng.index(bestSet.length)];
          }
          this.positions[i] = choice;
        }
        const [r, c] = [(this.positions[i] / COLS) | 0, this.positions[i] % COLS];
        for (const cell of sensingWindow(this.grid, r, c)) this.localMaps[i].add(cell);

        if (this.strategy === "full") {
          this.received[i] = new Set(this.localMaps[i]);
          this.transmittedCells += ROWS * COLS;
        } else {
          const pending = [];
          for (const cell of this.localMaps[i]) {
            if (!this.received[i].has(cell)) pending.push(cell);
          }
          pending.sort((a, b) => a - b);
          this.backlogCells += Math.max(0, pending.length - this.budgetCells);
          const limit = Math.min(pending.length, this.budgetCells);
          for (let k = 0; k < limit; k++) this.received[i].add(pending[k]);
          this.transmittedCells += limit;
        }
      }

      const serverMap = union(this.received);
      const localView = union(this.localMaps);
      this.coverage.push(serverMap.size / this.reachable);
      this.localCoverage.push(localView.size / this.reachable);
      this.stepIndex += 1;
      return this;
    }

    run(steps) {
      for (let s = 0; s < steps; s++) this.step();
      return this;
    }

    serverMap() {
      return union(this.received);
    }

    localView() {
      return union(this.localMaps);
    }
  }

  function union(sets) {
    const out = new Set();
    for (const set of sets) for (const cell of set) out.add(cell);
    return out;
  }

  function runExperiment(grid, strategy, budgetCells, seed) {
    const run = new Exploration(strategy, budgetCells, seed, grid).run(STEPS);
    return {
      strategy: strategy,
      budgetCellsPerStep: strategy === "full" ? ROWS * COLS : budgetCells,
      coverage: run.coverage,
      localCoverage: run.localCoverage,
      transmittedCells: run.transmittedCells,
      backlogCells: run.backlogCells,
      finalCoverage: run.coverage[run.coverage.length - 1],
      finalLocalCoverage: run.localCoverage[run.localCoverage.length - 1]
    };
  }

  /* ---- latency model (analysis/plot_latency_vs_nav_error.py) ------------ */
  const ACCEPTABLE_ERROR_M = 0.1;
  const JITTER_SIGMA_MS = 20;

  function deadbandError(speedMps, rttMs) {
    return (speedMps * rttMs) / 1000;
  }

  function maxRttWithin(speedMps, acceptableM) {
    return (1000 * (acceptableM === undefined ? ACCEPTABLE_ERROR_M : acceptableM)) / speedMps;
  }

  /* ---- link budget (analysis/plot_bandwidth_vs_update_rate.py) ---------- */
  function requiredMbps(baseMbpsAt10Hz, rateHz) {
    return baseMbpsAt10Hz * (rateHz / 10);
  }

  function achievableRateHz(baseMbpsAt10Hz, bandwidthMbps, maxRate) {
    const cap = maxRate === undefined ? 30 : maxRate;
    let best = 0;
    for (let rate = 1; rate <= cap; rate++) {
      if (requiredMbps(baseMbpsAt10Hz, rate) <= bandwidthMbps) best = rate;
      else break;
    }
    return best;
  }

  root.FS2S = {
    ENGINE_VERSION: ENGINE_VERSION,
    constants: {
      ROWS: ROWS,
      COLS: COLS,
      OBSTACLE_PROB: OBSTACLE_PROB,
      N_ROBOTS: N_ROBOTS,
      STEPS: STEPS,
      SEED: SEED,
      SENSOR_RADIUS: SENSOR_RADIUS,
      RANDOM_STEP_PROB: RANDOM_STEP_PROB
    },
    Mulberry32: function (seed) {
      const rng = new Rng(seed);
      return function () {
        return rng.nextFloat();
      };
    },
    Rng: Rng,
    buildGrid: buildGrid,
    freeCells: freeCells,
    sensingWindow: sensingWindow,
    neighbours: neighbours,
    Exploration: Exploration,
    runExperiment: runExperiment,
    deadbandError: deadbandError,
    maxRttWithin: maxRttWithin,
    requiredMbps: requiredMbps,
    achievableRateHz: achievableRateHz,
    ACCEPTABLE_ERROR_M: ACCEPTABLE_ERROR_M,
    JITTER_SIGMA_MS: JITTER_SIGMA_MS
  };
})(typeof globalThis !== "undefined" ? globalThis : this);

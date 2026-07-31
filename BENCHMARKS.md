# Benchmarks

The public benchmark surface is intentionally small:

- feasibility: cumulative exact-solution success over runtime
- anytime: cumulative success plus median current-best makespan over runtime
- multi-core: feasibility runs comparing ARC and ParallelARC worker counts

All workload executables are built from `apps/` and placed in `build/apps/`.
`benchmarks/` contains the runners, tests, and output documentation.

## Runner Requirements

The benchmark runners require Python 3.10 or newer. Trial execution, manifests,
and CSV outputs use only the Python standard library. Install matplotlib if you
want the runners to also produce PNG/SVG plots; when matplotlib is missing, the
runners skip plots and still write the machine-readable outputs.

Multi-core runs exercise ParallelARC's POSIX process-based worker path, so they
are intended for Linux-style environments with `fork`, pipes, and signals.

## Build

```bash
cmake -S . -B build -DCOMOTION_BUILD_APPS=ON
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
```

## Public Runners

When run from the source tree, the runners default to `build/apps` for
executables and `benchmarks/results/<experiment>_<timestamp>` for outputs. When
run from an installed package, they default to the install prefix's `bin/`
directory for executables and `./comotion-results/<experiment>_<timestamp>` for
outputs. Use `--build-dir` and `--output-root` to override either location.
Installed runners execute workload apps from the install prefix's
`share/comotion/` directory so installed Panda and Planar3 resource paths resolve.

Installed runner example:

```bash
python3 /path/to/comotion-install/share/comotion/benchmarks/scripts/run_feasibility.py \
  --cases default \
  --num-seeds 5 \
  --time-limit 60
```

### Feasibility

```bash
python3 benchmarks/scripts/run_feasibility.py \
  --cases default \
  --algorithms arc,prioritized,drrt,stcbs \
  --num-seeds 5 \
  --time-limit 60
```

This writes cumulative success plots under `<output-root>/plots/`.

### Anytime

```bash
python3 benchmarks/scripts/run_anytime.py \
  --cases default \
  --algorithms arc,ao_arc,composite_aorrtc \
  --num-seeds 5 \
  --time-limit 120
```

This writes one stacked panel per benchmark case. The top panel is cumulative
success over runtime; the bottom panel is median current-best makespan over
runtime.

### Multi-Core

```bash
python3 benchmarks/scripts/run_multicore.py \
  --cases mobile_parallel_n8,planar_cross_n8 \
  --worker-counts 2,4 \
  --num-seeds 5 \
  --time-limit 60
```

This compares ARC against ParallelARC with the requested worker-process counts
and writes cumulative success plots.

By default, ParallelARC plans different robots in parallel during initial
solution construction but does not duplicate a robot's initial query. Add
`--parallel-arc-initial-solution-or` to any runner, or to a direct
`--algorithm parallel_arc` executable run, to let spare initial-planning workers
race duplicate attempts and keep the first successful path.

Paper-style P-ARC presets are available through `--variant-set`. Use
`--dry-run` first to inspect the generated command matrix without launching the
experiments:

```bash
python3 benchmarks/scripts/run_multicore.py \
  --cases paper_2d \
  --variant-set paper-full \
  --num-seeds 20 \
  --time-limit 30 \
  --keep-metrics-json \
  --dry-run

python3 benchmarks/scripts/run_multicore.py \
  --cases paper_panda \
  --variant-set paper-full \
  --num-seeds 10 \
  --task-indices 0,1,2,3,4 \
  --time-limit 100 \
  --keep-metrics-json \
  --dry-run
```

The `paper-arc` variant set includes ARC, P-ARC worker counts, OR-ARC-16, and
OR-P-ARC 2x8/4x4/8x2. The `paper-full` variant set additionally includes the
implemented composite-RRT baselines: RRT-C, cooperative C-RRT-C-16,
OR-RRT-C-16, OR-C-RRT-C 2x8/4x4/8x2, and OR-PP-ST-RRT-16. The
OR-PP-ST-RRT baseline runs `prioritized` under outer OR parallelism with
`--strrt-shuffle-priority-order`, so each OR worker samples its priority order
from that worker's distinct OR planning seed. Use
`--or-pp-strrt-worker-counts` to override the default 16-worker paper setting,
or `--variant-set paper-or-pp-strrt` to run only that baseline.
The conflict-detection horizon ablation is available as
`paper-horizon-ablation`. Parallel conflict detection always distributes the
pair frontier uniformly among workers in round-robin order.

The optimistic-conflict batching ablation runs full P-ARC planning trials on
the 8-robot Panda Cage benchmark and preserves per-trial metrics JSON so
conflict-detection time can be summarized from whole planning runs:

```bash
python3 benchmarks/scripts/run_parallel_arc_optimistic_ablation.py \
  --num-seeds 10 \
  --task-indices 0,1,2,3,4 \
  --time-limit 100 \
  --jobs 1
```

Without `--allow-nonpaper-matrix`, this runner enforces the exact Table III
matrix: five tasks, ten seeds per task, both batch modes, and a 100-second
timeout (100 trials total). It refuses to summarize an incomplete matrix.
`optimistic_conflict_ablation_summary.csv` reports arithmetic means for both
successful trials and all trials; failed trials count as 100-second runtimes,
matching the paper. It also reports average batch size and average first-batch
size. Use `--allow-nonpaper-matrix` only for explicitly partial diagnostics.
The historical `parallel_arc_optimistic_ablation_20260720_163149` bundle
covers tasks 1–4 only and should be treated as a partial diagnostic, not as the
Table III source.

## Case Catalog

The runners expose these benchmark cases:

```text
mobile_parallel_n4
mobile_parallel_n8
mobile_parallel_n16
mobile_parallel_n32
mobile_parallel_n64
mobile_parallel_n128
mobile_circle_n4
mobile_circle_n8
mobile_circle_n16
planar_cross_n4
planar_cross_n8
planar_cross_n16
planar_cross_n32
planar_cross_n64
planar_cross_n128
planar_adaptive_n8
panda_cage_n2
panda_cage_n4
panda_cage_n8
panda_cage_n16
panda_flat_n4
```

`--cases default` expands to a small smoke/evaluation set. Named paper groups
are also accepted: `paper`, `paper_2d`, `paper_mobile_cross`,
`paper_mobile_circle`, `paper_planar_cross`, `paper_panda`, and
`paper_conflict_ablation`. Panda cases are task-based and use `--task-indices`
to select built-in tasks.

## Output Contract

Each runner writes:

```text
manifest.json
results.csv
solution_events.csv
plots/*.png
plots/*.svg
```

`results.csv` contains one row per trial and is appended as trials finish, so
completed trial rows survive an interrupted run. `solution_events.csv` contains
the solution-improvement events needed to regenerate anytime plots.

By default, rerunning with the same `--output-root` or `--results-csv` keeps
existing `results.csv` rows and skips matching completed trials. A completed
trial is identified by case, task index, seed, method, and time limit. Pass
`--overwrite-results` to discard existing CSV rows and rerun the requested
matrix. Use `--results-csv <path>` to write the main CSV to an explicit path;
if `--output-root` is omitted, metrics, manifest, plots, and the default
`solution_events.csv` path are rooted beside that CSV.

`results.csv` columns:

```text
case,case_title,task_index,seed,method,time_limit_seconds,success,first_solution_time_seconds,planning_time_seconds,makespan_timesteps,sum_of_cost_timesteps,metrics_json
```

`solution_events.csv` columns:

```text
case,task_index,seed,method,elapsed_seconds,makespan_timesteps
```

Direct `--metrics-json` output from each workload executable uses the compact
schema consumed by the runners, plus full planner stats and benchmark context:

```json
{
  "success": true,
  "planning_time_seconds": 1.23,
  "sum_of_cost_timesteps": 84,
  "makespan_timesteps": 42,
  "planner_stats": {
    "solution_events": [
      {
        "elapsed_seconds": 1.23,
        "makespan_timesteps": 42
      }
    ]
  }
}
```

Runner output keeps only CSV summaries by default. Pass `--keep-metrics-json`
to preserve the full per-trial metrics under `<output-root>/metrics/`; this is
recommended for reproducing stage breakdowns and configuration ablations.

## Direct Executable Use

The workload executables can also be called directly. For example:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/mobile_parallel_n4_arc_seed0.json

./build/apps/planar_manipulator_cross \
  --scenario cross \
  --num-robots 4 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/planar_cross_n4_arc_seed0.json

./build/apps/panda_cage \
  --num-robots 2 \
  --task-index 0 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/panda_cage_n2_arc_seed0.json
```

Installed direct Panda or Planar3 executable runs resolve the packaged
resources relative to the installed `bin/` directory. Explicit resource paths
under `/path/to/comotion-install/share/comotion/resources/` can still be passed when
overriding the packaged models.

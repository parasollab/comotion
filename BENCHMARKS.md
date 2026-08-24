# Benchmarks

The public benchmark surface consists of the exact paper reproduction runners
plus smaller exploratory runners:

- Parallel ARC 2D and Panda Cage paper matrices
- Parallel ARC conflict-assignment and optimistic-batching ablations
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
On Linux, worker processes arm a parent-death signal so interrupted trials do
not leave nested planning or collision-checking workers running.

Detailed collision-validation timing is disabled for the final paper profiles
because it adds counters to hot paths. Set
`COMOTION_VALIDATION_INSTRUMENTATION=1` only for dedicated profiling runs.

## Build

```bash
cmake -S . -B build -DCOMOTION_BUILD_APPS=ON
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
cmake --build build --target heterogeneous_corridor
```

### MR-dRRT local connector

MR-dRRT defaults to the paper's prioritized local connector. For each
ConnectToTarget candidate it finds one path per robot on the individual PRM
roadmaps, builds the start/goal interference graph described in Section 4.2 of
Solovey et al., and executes the paths in topological priority order. Emitted
paths include the resulting waits and sequential motion.

Use `--drrt-local-connector synchronized` to select the legacy direct
synchronized anchor-to-goal interpolation. The accepted values are
`prioritized` (default) and `synchronized`.

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

### Parallel ARC Paper Reproduction

These are the canonical first-release entry points for the final P-ARC
experiments:

```bash
python3 benchmarks/scripts/run_parallel_arc_2d.py --dry-run
python3 benchmarks/scripts/run_parallel_arc_panda.py --dry-run

python3 benchmarks/scripts/run_parallel_arc_2d.py \
  --output-root benchmarks/results/parallel_arc_2d
python3 benchmarks/scripts/run_parallel_arc_panda.py \
  --output-root benchmarks/results/parallel_arc_panda
```

Both runners execute one top-level trial at a time, allow planners to use up to
16 internal workers, retain per-trial metrics, record the complete effective
command matrix, and resume compatible completed trials. The 2D suite uses 30
seeds and a 30-second time limit. The Panda suite uses team sizes 4, 8, and 16,
five tasks, ten seeds per task, and a 100-second time limit.

The final profiles are also the workload-executable defaults for these cases:

| Benchmark | ARC window profile | Local repair | P-ARC profile |
|---|---|---|---|
| Mobile Parallel/Circle | initial 200; linear step 200; symmetric initial-valid expansion; C-space margin/range 2/2 | composite, 5,000 samples; simplify initial paths only | initial-solution OR on; duplicate repair attempts off; horizon 400 |
| Planar Cross | initial 100; exponential factor 1.05; initial-valid linear step 10 and asymmetric expansion; C-space margin/range 1/0.5 | composite, 50,000 samples; simplify initial and conflict paths | initial-solution OR on; duplicate repair attempts off; horizon 400 |
| Panda Cage | initial 20; exponential factor 1.05; initial-valid linear step 20 and asymmetric expansion; C-space margin/range 2/2 | composite, 250,000 samples with makespan metric; simplify initial paths only | initial-solution OR on; duplicate repair attempts on; horizon 200 |

All three use VAMP, resolution 128, synchronous repair batches, greedy conflict
selection, segment-parallel conflict finding, optimistic independent conflict
batches, and `cyclic_cover_greedy` worker assignment. The runners pass these
values explicitly as well as relying on defaults, preventing later default
changes from silently altering reproduction runs.

The 2D runner first evaluates ARC/P-ARC worker scaling, then the main method
comparison, reusing matching ARC and P-ARC-16 rows. It includes the first team
size at which ARC succeeds on at most half of its trials, then stops scaling
that scenario. The Panda runner applies the same zero-success pruning rule used
by the final campaign when advancing from 4 to 8 to 16 robots.

The paper ablations are separate public entry points:

```bash
python3 benchmarks/scripts/run_parallel_arc_conflict_ablation.py --dry-run
python3 benchmarks/scripts/run_parallel_arc_optimistic_ablation.py \
  --num-seeds 10 --task-indices 0,1,2,3,4 --time-limit 100 --jobs 1
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
Add `--plot-backends` when one output contains multiple validation backends.
In that mode, algorithm color is held fixed while validation backend is encoded
by line style: VAMP solid, sphere dashed, and FCL dotted. Runtime axes use a
log scale, with one shared lower bound for all plots that have the same timeout.

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

ParallelARC can plan different robots in parallel during initial solution
construction. The final P-ARC profiles also enable initial-solution OR, letting
spare initial-planning workers race duplicate attempts and keep the first
successful path. Generic runs can disable it with
`--no-parallel-arc-initial-solution-or`.

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
`paper-horizon-ablation`. The final profile uses cyclic-cover greedy assignment;
round-robin, pair-cover, balanced pair-cover, and pair-first greedy remain
available for controlled comparisons.

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

### Planner/Backend Replication

```bash
python3 benchmarks/scripts/run_planner_trials.py \
  --scenarios default \
  --methods default \
  --backends default \
  --cores 8
```

This runner expands to the paper-scale planner trials for the supported
scenario families: `panda_cage`, `flying_spheres`, and
`heterogeneous_corridor`. The `flying_spheres` scenario maps to the
`mobile_robot_2d_crossing --scenario parallel` app, `panda_cage` uses the
built-in Panda cage tasks directly, and `heterogeneous_corridor` runs the
mixed corridor cases with twice as many spheres as Pandas.

Defaults use the paper's main cumulative-success trial counts and limits:
flying spheres use 30 seeds with a 10 second limit for each robot count; Panda
cage uses five built-in tasks and ten seeds per task, with a 150 second limit;
heterogeneous corridor uses task 0 with 10 seeds and a 150 second limit. Use
`--num-trials`, `--time-limit`, or `--time-limits
flying_spheres=10,panda_cage=150,heterogeneous_corridor=150` to override
those defaults. Default team sizes are flying spheres n=4,8,16,32,64,128,
Panda cage n=4,8,16, and heterogeneous corridor p4/s8,p8/s16,p16/s32. The
runner deliberately uses the same 50 Panda trials (five tasks by ten seeds) at
all three Panda team sizes. These uniform defaults expand the paper's narrower
method-specific extension runs; staged pruning prevents configurations that
already failed at a smaller size from consuming time at larger sizes.

Non-dry runs are staged by scenario and increasing team size. If a
method/backend combination solves zero trials for one task index at a team
size, the runner records that in `pruned_combinations.json` and skips the same
combination and task index at larger team sizes in that scenario. Panda cage
task JSON is generated separately for each team size, so its task index is used
as a cross-size pruning key rather than a literal same-instance guarantee. The
heterogeneous cases use the canonical `mr-ompl` p4/s8, p8/s16, and p16/s32 task
files under `resources/benchmarks`; the manifest records each selected file and
SHA-256 digest. Those files define the exact ordered robots, Panda bases and
orientations, starts/goals, sphere radii, environment, and obstacles. Pruning
remains backend-specific, so failure with one collision backend does not
suppress the others.

The default planner parameter file is
`benchmarks/configs/planner_trial_params.json`. For flying spheres and Panda
cage, it intentionally extrapolates each scenario's effective n=8 profile to
every requested team size. The representative heterogeneous p4/s8 profile is
likewise shared by p4/s8, p8/s16, and p16/s32. These are representative tuning
profiles, not independent per-size sweeps. Planner-specific overrides can be
placed under `planners` at the global, scenario, or robot-count level; the
legacy key `methods` is still accepted for the same purpose. Shared defaults
are resolved first, then planner-specific overrides.

Those planner/backend replication profiles reproduce the VA-MRMP evaluation
and intentionally override workload-executable defaults where that paper used
different settings. Use `run_parallel_arc_2d.py` and
`run_parallel_arc_panda.py`, not `run_planner_trials.py`, for the final P-ARC
parameter profiles.

The heterogeneous PP-ST-RRT profile uses initial batch size 4096, initial time
factor 2, time-factor increase 2, first-solution return, and rewiring off.
Priorities are seed-shuffled, with Pandas shuffled first and flying spheres
shuffled separately after them.

VAMP planning strategy is selected by the planner parameter file. The paper
default uses combined validation ordering with rake packing for every planner.
The validation-timing app still selects each named VAMP variant explicitly when
comparing validation strategies.

Use a dry run to inspect every planned case and a sample of commands without
creating an output directory:

```bash
python3 benchmarks/scripts/run_planner_trials.py --dry-run
```

Add `--dry-run-limit 0` to print every command.

The default dry run reports 5,400 trials before pruning, split across all 12
paper-reported team sizes in this uniform suite.

The runner is resumable. Each trial writes a stable `trials/.../trial.json`
record plus the raw metrics JSON. Re-running with the same `--output-root`
skips completed trials by default only when the record's configuration
signature matches the current command and parameters. If you change planner
parameters, build paths, time limits, or other trial inputs, pass
`--existing overwrite` or choose a new `--output-root`.

Use `plot_results.py` to generate plots from an existing `results.csv`, such as
the full planner/backend replication output:

```bash
python3 benchmarks/scripts/plot_results.py \
  benchmarks/results/mrm_in_ms_full_suite_tuned_20260709_185756/results.csv \
  --plot-kind success \
  --plot-backends
```

With `--plot-backends`, each scenario/team-size plot uses one color per
algorithm and one line style per validation backend. The legend is split into
`Algorithms` for the colored solid lines and `Validation` for backend line
styles. Runtime axes use a log scale, with one shared lower bound for all plots
that have the same timeout.

### Parameter Sweeps

```bash
python3 benchmarks/scripts/run_param_sweep.py \
  --config benchmarks/configs/heterogenous_param_sweep.json \
  --apps heterogenous_p4_s8 \
  --cores 8 \
  --best-params-output benchmarks/configs/heterogenous_params.json
```

The sweep runner is app-generic: the JSON config supplies each app executable,
base arguments, timeout, methods, trials per parameter set, and method-specific
parameter sets or Cartesian grids. Ranking uses higher `success_count` first,
then lower mean effective `planning_time_seconds`; makespan is not used for
selecting best parameters.

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
heterogeneous_corridor_p4_s8
heterogeneous_corridor_p8_s16
heterogeneous_corridor_p16_s32
```

`--cases default` expands to a small smoke/evaluation set. Named paper groups
are also accepted: `paper`, `paper_2d`, `paper_mobile_cross`,
`paper_mobile_circle`, `paper_planar_cross`, `paper_panda`, and
`paper_conflict_ablation`. Panda cases are task-based and use `--task-indices`
to select built-in tasks. The heterogeneous corridor cases place Pandas first
in the robot order, followed by flying spheres, matching the priority order
used by the paper-style scenario.

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
case,case_title,task_index,seed,method,collision_backend,time_limit_seconds,success,first_solution_time_seconds,planning_time_seconds,makespan_timesteps,sum_of_cost_timesteps,metrics_json
```

`solution_events.csv` columns:

```text
case,task_index,seed,method,collision_backend,elapsed_seconds,makespan_timesteps
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

./build/apps/heterogeneous_corridor \
  --num-pandas 4 \
  --num-spheres 8 \
  --task-index 0 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/heterogeneous_corridor_p4_s8_arc_seed0.json
```

Installed direct Panda or Planar3 executable runs resolve the packaged
resources relative to the installed `bin/` directory. Explicit resource paths
under `/path/to/comotion-install/share/comotion/resources/` can still be passed when
overriding the packaged models.

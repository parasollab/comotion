# CoMotion Path Viewer

Web-based visualization for CoMotion path result JSON files.

## Running the Viewer

**Important:** Serve the project from the **repository root** so asset paths in the JSON (e.g. `resources/panda/...`) resolve correctly.

```bash
# From repo root
npx serve .
# or
python -m http.server 8000
```

Then open: `http://localhost:8000/viewer/`

When the page URL path contains `viewer` (e.g. `/viewer/`), the viewer resolves `resources/` and `?file=` paths against the **parent** of that segment (the repo root). If this script is loaded as `/viewer/js/app.js`, the same repo root is inferred from `import.meta.url`.

If you serve **only** the `viewer/` directory (e.g. `python -m http.server --directory viewer`), the repository includes a symlink **`viewer/resources` → `../resources`** so `/resources/panda/...` is still valid. On Windows without symlink support, either serve from the **repository root** or set `?assetBase=` to a URL where `resources/` is reachable.

Direct app runs write viewer-compatible result JSON when `--output-paths` or
`--output-endpoint-paths` is passed. For example:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --output-endpoint-paths \
  --output-dir benchmarks/results/viewer_demo
```

Then load the generated result with:
`http://localhost:8000/viewer/?file=benchmarks/results/viewer_demo/mobile_robot_2d_crossing_parallel_n4_seed0_EndpointPath_result.json`

## Loading Results

- **File picker:** Click "Load JSON" and select a `*_result.json` file.
- **URL parameter:** Add `?file=<path-to-result-json>` (path relative to server root).

## ARC Process Playback

When `--output-paths` and `--track-arc-history` are both used with `arc`,
`ao_arc`, or `parallel_arc`, the generated `*_result.json` also contains an
`arc_visualization` trace. The trace is embedded
in the same result file and records:

- every complete global path set produced after individual planning or a repair batch;
- the conflict batch associated with each path set (one conflict for ARC);
- the local repair path for every robot in each applied subproblem.

Loading one of these files selects **ARC process** mode automatically. During
conflict detection, robots advance in gray; robots freeze and turn red when
their conflict is reached, and playback holds that conflict timestep for the
duration of 10 ordinary playback steps at 1×. The equivalent hold steps scale
with playback speed (for example, 200 steps at 20×), keeping the conflict
visible for the same real-time duration. During repair, unrelated robots are
hidden and each robot-disjoint subproblem receives its own color. A final
conflict-free path set turns green. Use the **Mode** selector to switch back to
ordinary final-solution playback. The **Speed** selector changes the playback
rate in both modes and provides options through 100×.

Example:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --algorithm parallel_arc \
  --parallel-arc-worker-processes 2 \
  --output-paths \
  --track-arc-history \
  --output-dir benchmarks/results/viewer_demo
```

The ARC trace is opt-in and retained only when both flags are present. Using
either flag alone neither captures nor writes ARC history, so normal benchmark
runs do not pay the memory cost of storing intermediate paths.

The pure playback timeline regression can be run with:

```bash
node viewer/tests/arc-playback.test.mjs
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Left | Step backward 1 |
| Right | Step forward 1 |
| Shift+Left | Step backward 10 |
| Shift+Right | Step forward 10 |
| Home | Jump to first timestep |
| End | Jump to last timestep |
| Space | Play / Pause |

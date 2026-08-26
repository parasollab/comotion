# CoMotion Path Viewer

The browser viewer displays path-result JSON produced by CoMotion applications.
It supports final-path playback for FCL, sphere, and VAMP runs, plus ARC process
playback for ARC-family planners.

## Create a Result

Build CoMotion using the main [installation instructions](../README.md#installation),
then write a result from any supported application:

```bash
./build/apps/panda_cage \
  --num-robots 8 \
  --output-endpoint-paths \
  --output-dir benchmarks/results/viewer_demo
```

This creates a start/goal-only result without running the planner. Use
`--output-paths` instead to run the default ARC planner and save its solution.

## Run the Viewer

Serve the repository root so robot models and meshes resolve correctly:

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000/viewer/`.

Load a `*_result.json` file with either:

- **Load JSON** in the viewer; or
- `?file=<path>`, where the path is relative to the repository root.

For the example above:

```text
http://localhost:8000/viewer/?file=benchmarks/results/viewer_demo/panda_cage_n8_task0_seed0_EndpointPath_result.json
```

## ARC Process Playback

ARC process playback supports `arc`, `ao_arc`, and `parallel_arc`. Enable
both history flags when creating the result:

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

Loading the result selects ARC process mode automatically. Use the **Mode**
selector to switch between the ARC process and final path.

## Controls

| Key | Action |
|---|---|
| Left | Step backward |
| Right | Step forward |
| Shift+Left | Step backward 10 steps |
| Shift+Right | Step forward 10 steps |
| Home | First timestep |
| End | Last timestep |
| Space | Play or pause |

Run the viewer timeline test with:

```bash
node viewer/tests/arc-playback.test.mjs
```

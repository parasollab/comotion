import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  ARC_CONFLICT_HOLD_TIMESTEPS,
  arcFrameDurationTimesteps,
  buildArcTimeline,
  configAtPath,
  firstReachedConflict,
} from "../js/arc-playback.js";

const trace = {
  arc_visualization: {
    iterations: [
      {
        paths: [
          [[0], [1], [2], [3]],
          [[3], [2], [1], [0]],
        ],
        timesteps: 4,
        conflict_scan_completed: true,
        conflicts: [{ robot_i: 0, robot_j: 1, robots: [0, 1], timestep: 1 }],
        repairs: [{
          conflict_index: 0,
          robots: [0, 1],
          paths: [
            [[1], [1.5], [2]],
            [[2], [2.5], [1]],
          ],
        }],
      },
      {
        paths: [
          [[0], [1], [2], [3]],
          [[3], [2.5], [1.5], [0]],
        ],
        timesteps: 4,
        conflict_scan_completed: true,
        conflicts: [],
        repairs: [],
      },
    ],
  },
};

const timeline = buildArcTimeline(trace);
assert.equal(timeline.length, 9);
assert.deepEqual(
  timeline.map((frame) => frame.phase),
  ["paths", "paths", "repairs", "repairs", "repairs", "paths", "paths", "paths", "paths"]
);
assert.equal(firstReachedConflict(trace.arc_visualization.iterations[0], 0, 0), null);
assert.equal(firstReachedConflict(trace.arc_visualization.iterations[0], 0, 1)?.timestep, 1);
assert.deepEqual(configAtPath([[0], [1]], 20), [1]);
assert.equal(timeline.at(-1).solution, true);
assert.equal(ARC_CONFLICT_HOLD_TIMESTEPS, 10);
assert.equal(arcFrameDurationTimesteps(trace, timeline[0]), 1);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1]), 10);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1], 0.25), 2.5);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1], 20), 200);
assert.equal(arcFrameDurationTimesteps(trace, timeline[2]), 1);

const viewerHtml = readFileSync(new URL("../index.html", import.meta.url), "utf8");
assert.match(viewerHtml, /<option value="50">50×<\/option>/);
assert.match(viewerHtml, /<option value="100">100×<\/option>/);

console.log("arc-playback.test: OK");

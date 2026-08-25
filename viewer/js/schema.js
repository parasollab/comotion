/**
 * Parse and validate CoMotion result JSON schema.
 * Provides path clamping for variable-length robot paths.
 */

const SCHEMA_VERSION = "1.0";

/**
 * Get config for robot at timestep t, clamping to last config if path is shorter.
 * @param {Object} robot - Robot object with path and path_length
 * @param {number} t - Timestep index
 * @returns {number[]} Joint configuration
 */
function configAt(robot, t) {
  const path = robot.path || [];
  const pathLength = robot.path_length ?? path.length;
  if (t >= pathLength || t >= path.length) {
    return path.length > 0 ? path[path.length - 1] : [];
  }
  return path[t];
}

function normalizeArcVisualization(data) {
  const trace = data.arc_visualization;
  if (!trace || typeof trace !== "object" || !Array.isArray(trace.iterations)) {
    data.arc_visualization = null;
    return;
  }

  const robotCount = data.robots.length;
  const iterations = [];
  for (const rawIteration of trace.iterations) {
    if (!rawIteration || !Array.isArray(rawIteration.paths)) continue;
    if (
      rawIteration.paths.length !== robotCount ||
      rawIteration.paths.some((path) => !Array.isArray(path))
    ) {
      console.warn("Ignoring ARC visualization iteration with invalid path set");
      continue;
    }

    iterations.push({
      paths: rawIteration.paths,
      timesteps:
        rawIteration.timesteps ??
        Math.max(...rawIteration.paths.map((path) => path.length), 0),
      conflict_scan_completed: rawIteration.conflict_scan_completed === true,
      conflicts: Array.isArray(rawIteration.conflicts)
        ? rawIteration.conflicts.filter(
            (conflict) =>
              conflict &&
              Number.isInteger(conflict.robot_i) &&
              Number.isInteger(conflict.robot_j) &&
              Number.isFinite(Number(conflict.timestep))
          )
        : [],
      repairs: Array.isArray(rawIteration.repairs)
        ? rawIteration.repairs.filter(
            (repair) =>
              repair &&
              Array.isArray(repair.robots) &&
              Array.isArray(repair.paths) &&
              repair.robots.length === repair.paths.length &&
              repair.paths.every((path) => Array.isArray(path))
          )
        : [],
    });
  }

  data.arc_visualization = iterations.length > 0
    ? {
        ...trace,
        workers: Math.max(1, Number(trace.workers) || 1),
        iterations,
      }
    : null;
}

/**
 * Parse and validate result JSON. Returns null if invalid.
 * @param {string} text - Raw JSON string
 * @returns {Object|null} Parsed result or null
 */
function parseResult(text) {
  try {
    const data = JSON.parse(text);
    if (!data.robots || !Array.isArray(data.robots)) {
      console.error("Invalid schema: missing or invalid robots array");
      return null;
    }
    if (!data.schema_version) {
      console.warn("No schema_version; assuming v1.0");
      data.schema_version = SCHEMA_VERSION;
    }
    data.timesteps = data.timesteps ?? Math.max(...data.robots.map(r => (r.path || []).length), 0);
    data.obstacles = data.obstacles || [];
    if (
      data.obstacles.length === 0 &&
      Array.isArray(data.benchmark?.context?.obstacles)
    ) {
      data.obstacles = data.benchmark.context.obstacles;
    }
    normalizeArcVisualization(data);
    return data;
  } catch (e) {
    console.error("JSON parse error:", e);
    return null;
  }
}

export { parseResult, configAt, normalizeArcVisualization, SCHEMA_VERSION };

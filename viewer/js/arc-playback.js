function configAtPath(path, timestep) {
  if (!Array.isArray(path) || path.length === 0) return [];
  return path[Math.max(0, Math.min(timestep, path.length - 1))];
}

function hasArcVisualization(data) {
  return Array.isArray(data?.arc_visualization?.iterations) &&
    data.arc_visualization.iterations.length > 0;
}

const ARC_CONFLICT_HOLD_TIMESTEPS = 10;

function buildArcTimeline(data) {
  if (!hasArcVisualization(data)) return [];
  const frames = [];
  data.arc_visualization.iterations.forEach((iteration, iterationIndex) => {
    const pathEnd = Math.max(
      0,
      (Number(iteration.timesteps) ||
        Math.max(...iteration.paths.map((path) => path.length), 0)) - 1
    );
    const conflicts = iteration.conflicts || [];
    const conflictEnd = conflicts.length > 0
      ? Math.max(...conflicts.map((conflict) => Number(conflict.timestep) || 0))
      : pathEnd;
    const detectionEnd = Math.max(0, Math.min(pathEnd, conflictEnd));
    for (let timestep = 0; timestep <= detectionEnd; ++timestep) {
      frames.push({
        phase: "paths",
        iterationIndex,
        timestep,
        phaseEnd: detectionEnd,
        solution:
          iteration.conflict_scan_completed &&
          conflicts.length === 0 &&
          timestep === detectionEnd,
      });
    }

    if (conflicts.length > 0 && iteration.repairs.length > 0) {
      const repairEnd = Math.max(
        0,
        Math.max(
          ...iteration.repairs.flatMap((repair) =>
            repair.paths.map((path) => path.length)
          ),
          1
        ) - 1
      );
      for (let timestep = 0; timestep <= repairEnd; ++timestep) {
        frames.push({
          phase: "repairs",
          iterationIndex,
          timestep,
          phaseEnd: repairEnd,
        });
      }
    }
  });
  return frames;
}

function arcFrameDurationTimesteps(data, frame, playbackSpeed = 1) {
  if (!frame || frame.phase !== "paths") return 1;
  const iteration =
    data?.arc_visualization?.iterations?.[frame.iterationIndex];
  const isConflictTimestep = (iteration?.conflicts || []).some(
    (conflict) => Number(conflict.timestep) === frame.timestep
  );
  const speed =
    Number.isFinite(playbackSpeed) && playbackSpeed > 0 ? playbackSpeed : 1;
  return isConflictTimestep ? ARC_CONFLICT_HOLD_TIMESTEPS * speed : 1;
}

function conflictRobots(conflict) {
  if (Array.isArray(conflict.robots) && conflict.robots.length > 0)
    return conflict.robots;
  return [conflict.robot_i, conflict.robot_j];
}

function firstReachedConflict(iteration, robotIndex, timestep) {
  return (iteration.conflicts || []).reduce((earliest, conflict) => {
    if (
      !conflictRobots(conflict).includes(robotIndex) ||
      Number(conflict.timestep) > timestep
    ) {
      return earliest;
    }
    return !earliest ||
      Number(conflict.timestep) < Number(earliest.timestep)
      ? conflict
      : earliest;
  }, null);
}

export {
  ARC_CONFLICT_HOLD_TIMESTEPS,
  arcFrameDurationTimesteps,
  buildArcTimeline,
  configAtPath,
  conflictRobots,
  firstReachedConflict,
  hasArcVisualization,
};

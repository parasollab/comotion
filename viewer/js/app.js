/**
 * CoMotion path visualization viewer.
 * Three.js-based timestep-by-timestep path playback.
 */

import * as THREE from "three";
import { TrackballControls } from "three/examples/jsm/controls/TrackballControls.js";
import { parseResult, configAt } from "./schema.js";
import {
  arcFrameDurationTimesteps,
  buildArcTimeline,
  configAtPath,
  conflictRobots,
  firstReachedConflict,
} from "./arc-playback.js";
import { createURDFLoader, loadURDFAsync } from "./urdf-loader.js?v=9";

// Panda 7-DOF joint names (matches planner config order)
const PANDA_JOINT_NAMES = [
  "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
  "panda_joint5", "panda_joint6", "panda_joint7",
];

// State
let scene, camera, renderer, controls;
let ambientLight = null;
let hemisphereLight = null;
let sunLight = null;
let fillLight = null;
let rimLight = null;
/** When true: key/fill lighting with shadows. When false: bright ambient, directional lights off. */
let lightingKeyMode = true;
/** When true: URDF robots show collision primitives; when false: visual meshes only. */
let showRobotCollisionGeometry = false;
/** When true: hide 3D geometry and render a flat z=0 cross-section. */
let showCrossSection2D = false;
let resultData = null;
let currentTimestep = 0;
let playbackMode = "solution";
let arcTimeline = [];
let robotMeshes = [];
let obstacleMeshes = [];
let crossSection2DGroup = null;
let isPlaying = false;
let playTimerId = null;
let playbackSpeedMultiplier = 1;
const PLAY_FPS = 12;
const ARC_PATH_COLOR = 0xaeb4bb;
const ARC_CONFLICT_COLOR = 0xd62828;
const ARC_SOLUTION_COLOR = 0x2f9e44;
const ARC_GROUP_COLORS = [
  0x1976d2,
  0xf57c00,
  0x7b1fa2,
  0x00897b,
  0xc2185b,
  0x689f38,
];
const CROSS_SECTION_Z = 0;
const CROSS_SECTION_EPS = 1e-6;
const PLANAR3_LINK_LENGTH = 0.3;
const PLANAR3_LINK_RADIUS = 0.05;
const DEFAULT_PANDA_VISUAL_URDF_PATH = "resources/panda/panda.urdf";
const PANDA_RESOURCE_REVISION = "20260824-real-visual-1";

function scaleRgbHex(hex, k) {
  const r = Math.round(((hex >> 16) & 0xff) * k);
  const g = Math.round(((hex >> 8) & 0xff) * k);
  const b = Math.round((hex & 0xff) * k);
  return (r << 16) | (g << 8) | b;
}

/** Prior obstacle gray (0xaaaaaa), lightened by 40% per RGB channel. */
const OBSTACLE_COLOR = scaleRgbHex(0xaaaaaa, 1.4);
const OBSTACLE_OPACITY = 0.45;

const ROBOT_COLOR_PALETTES = [
  {
    id: "neutral",
    label: "Neutral",
    emissiveIntensity: 1.15,
    receiveShadows: false,
    colors: [
      0x00ffff, // electric cyan
      0xff00ff, // electric magenta
      0xffff00, // electric yellow
      0x39ff14, // laser green
      0xff073a, // neon red
      0x00ffcc, // aqua glow
      0xff1493, // hot pink
      0xccff00, // acid chartreuse
      0xff5f1f, // blaze orange
      0x00b7ff, // plasma blue
      0xee00ff, // neon violet
      0xaaff00, // toxic lime
      0xffea00, // highlighter yellow
      0x00ff66, // green glow
      0xff0099, // punch pink
      0x33ffff, // ice cyan
      0xff3300, // hot vermilion
      0xbfff00, // volt green
      0x00ff99, // mint beam
      0xff00cc, // hot magenta
      0x99ff00, // neon grass
      0xff9900, // signal orange
      0x00ffef, // bright turquoise
      0xda00ff, // ultraviolet
    ],
  },
  {
    id: "royal",
    label: "Royal",
    colors: [
      0x0033a0, // royal blue
      0xd4af37, // metallic gold
      0x6a0dad, // royal purple
      0x005a32, // deep emerald
      0x9b111e, // ruby
      0x0047ab, // cobalt
      0x702963, // byzantium
      0x0f52ba, // sapphire
      0x800020, // burgundy
      0x006b54, // jade
      0xc04000, // mahogany orange
      0x32127a, // persian indigo
      0xb31b1b, // garnet
      0x008080, // regal teal
      0x4b0082, // indigo
      0xe0b0ff, // mauve
      0xbf5700, // burnt gold
      0x5d3fd3, // iris
      0x006400, // royal green
      0x8b008b, // deep magenta
      0x1f305e, // midnight blue
      0xa67c00, // antique gold
      0x722f37, // wine
      0x2e8b57, // sea emerald
    ],
  },
  {
    id: "neon",
    label: "Neon",
    emissiveIntensity: 2.4,
    receiveShadows: false,
    colors: [
      0x00ffff, // bright cyan
      0xff00cc, // neon pink
      0xff5a00, // bright orange
      0xffff00, // highlighter yellow
      0x39ff14, // laser green
      0xff007f, // hot rose
      0x00bfff, // electric sky
      0xff3300, // blaze vermilion
      0xccff00, // acid chartreuse
      0xff00ff, // pure magenta
      0x00ff99, // glowing mint
      0xff9900, // signal orange
      0x7df9ff, // electric blue
      0xff1493, // hot pink
      0xadff2f, // green yellow
      0xff6600, // safety orange
      0x00ffef, // neon turquoise
      0xfe019a, // fluorescent pink
      0xfefe22, // fluorescent yellow
      0xff00aa, // electric fuchsia
      0x00ff33, // laser lime
      0xff6ec7, // neon cotton candy
      0xff2400, // scarlet neon
      0xb6ff00, // toxic lime
    ],
  },
];

const ROBOT_COLOR_PALETTE_BY_ID = new Map(ROBOT_COLOR_PALETTES.map((palette) => [palette.id, palette]));
let selectedRobotPaletteId = ROBOT_COLOR_PALETTES[0].id;

function currentRobotPaletteSpec() {
  return ROBOT_COLOR_PALETTE_BY_ID.get(selectedRobotPaletteId) || ROBOT_COLOR_PALETTES[0];
}

function currentRobotColorPalette() {
  return currentRobotPaletteSpec().colors;
}

function robotColorHexForIndex(index) {
  const palette = currentRobotColorPalette();
  return palette[index % palette.length];
}

function robotColorStyleKey(hex) {
  return `robot-${selectedRobotPaletteId}-${hex.toString(16).padStart(6, "0")}`;
}

function createRobotSurfaceMaterial(hex, styleKey = robotColorStyleKey(hex)) {
  const glow = currentRobotPaletteSpec().emissiveIntensity ?? 0;
  const material = new THREE.MeshLambertMaterial({
    color: hex,
    emissive: glow > 0 ? hex : 0x000000,
    emissiveIntensity: glow,
    vertexColors: false,
    transparent: false,
    opacity: 1,
    depthWrite: true,
    side: THREE.DoubleSide,
  });
  material.toneMapped = glow <= 0;
  material.userData.comotionSolidColor = hex;
  material.userData.comotionColorStyleKey = styleKey;
  return material;
}

// DOM refs
let timestepEl, playPauseBtn, sliderEl, playbackModeSelectEl, playbackSpeedSelectEl;
let cameraPanelEl = null;

function fmtNum(v, digits = 4) {
  if (!Number.isFinite(v)) return "";
  const s = v.toFixed(digits);
  return s.replace(/\.?0+$/, "") || "0";
}

function parseInput(id, fallback = 0) {
  const el = document.getElementById(id);
  if (!el) return fallback;
  const v = parseFloat(String(el.value).trim());
  return Number.isFinite(v) ? v : fallback;
}

function syncCameraInputsFromOrbit() {
  if (!camera || !controls || !cameraPanelEl) return;
  if (cameraPanelEl.contains(document.activeElement)) return;

  const p = camera.position;
  const t = controls.target;
  const tx = document.getElementById("cam-tx");
  const ty = document.getElementById("cam-ty");
  const tz = document.getElementById("cam-tz");
  const px = document.getElementById("cam-px");
  const py = document.getElementById("cam-py");
  const pz = document.getElementById("cam-pz");
  const az = document.getElementById("cam-az");
  const pol = document.getElementById("cam-pol");
  const rEl = document.getElementById("cam-r");
  if (!tx || !px || !az || !rEl) return;

  tx.value = fmtNum(t.x);
  ty.value = fmtNum(t.y);
  tz.value = fmtNum(t.z);
  px.value = fmtNum(p.x);
  py.value = fmtNum(p.y);
  pz.value = fmtNum(p.z);

  const offset = new THREE.Vector3().subVectors(p, t);
  const sph = new THREE.Spherical().setFromVector3(offset);
  const azDeg = THREE.MathUtils.radToDeg(sph.theta);
  az.value = fmtNum(THREE.MathUtils.euclideanModulo(azDeg, 360), 2);
  pol.value = fmtNum(THREE.MathUtils.radToDeg(sph.phi), 2);
  rEl.value = fmtNum(sph.radius, 4);
}

function applyCameraFromInputs() {
  if (!camera || !controls) return;

  const tx = parseInput("cam-tx", 0);
  const ty = parseInput("cam-ty", 0);
  const tz = parseInput("cam-tz", 0);
  controls.target.set(tx, ty, tz);

  const rEl = document.getElementById("cam-r");
  const rStr = rEl ? String(rEl.value).trim() : "";
  const r = rStr === "" ? NaN : parseFloat(rStr);

  if (Number.isFinite(r) && r > 0) {
    const azDeg = parseInput("cam-az", NaN);
    const polDeg = parseInput("cam-pol", NaN);
    if (Number.isFinite(azDeg) && Number.isFinite(polDeg)) {
      const azRad = THREE.MathUtils.degToRad(
        THREE.MathUtils.euclideanModulo(azDeg, 360)
      );
      const sph = new THREE.Spherical(
        r,
        THREE.MathUtils.degToRad(polDeg),
        azRad
      );
      const offset = new THREE.Vector3().setFromSpherical(sph);
      camera.position.copy(controls.target).add(offset);
    } else {
      camera.position.set(
        parseInput("cam-px", 2),
        parseInput("cam-py", 2),
        parseInput("cam-pz", 2)
      );
    }
  } else {
    camera.position.set(
      parseInput("cam-px", 2),
      parseInput("cam-py", 2),
      parseInput("cam-pz", 2)
    );
  }

  camera.lookAt(controls.target);
  controls.update();
}

function obstacleWorldCenter(obs) {
  const p = obs.pose?.position;
  if (p && p.length >= 3) return new THREE.Vector3(p[0], p[1], p[2]);
  return null;
}

function expandByPoint(min, max, point) {
  min.min(point);
  max.max(point);
}

function expandByRadius(min, max, center, radius) {
  const r = Number.isFinite(radius) ? radius : 0;
  min.min(new THREE.Vector3(center.x - r, center.y - r, center.z - r));
  max.max(new THREE.Vector3(center.x + r, center.y + r, center.z + r));
}

function expandObstacleBounds(min, max, obs) {
  const center = obstacleWorldCenter(obs);
  if (!center) return;
  if (obs.type === "sphere") {
    expandByRadius(min, max, center, obs.geometry?.radius ?? 0);
    return;
  }
  if (obs.type === "cylinder") {
    const radius = obs.geometry?.radius ?? 0;
    const halfHeight = obs.geometry?.half_height ?? 0;
    const axis = obs.pose?.axis || [0, 1, 0];
    const axisVec = new THREE.Vector3(axis[0], axis[1], axis[2]);
    if (axisVec.lengthSq() > 1e-12) axisVec.normalize();
    else axisVec.set(0, 1, 0);
    const offset = axisVec.multiplyScalar(halfHeight);
    expandByRadius(min, max, center.clone().sub(offset), radius);
    expandByRadius(min, max, center.clone().add(offset), radius);
    return;
  }
  expandByPoint(min, max, center);
}

function computeResultBounds(data) {
  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  let count = 0;
  const expand = (point) => {
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z)) {
      return;
    }
    expandByPoint(min, max, point);
    count++;
  };

  for (const obs of data.obstacles || []) {
    const before = count;
    expandObstacleBounds(min, max, obs);
    if (count === before && obstacleWorldCenter(obs)) count++;
  }

  for (const robot of data.robots || []) {
    if (robot.robot_type === "sphere") {
      for (const cfg of robot.path || []) {
        if (cfg && cfg.length >= 3) expand(new THREE.Vector3(cfg[0], cfg[1], cfg[2]));
      }
    }
    const bp = robot.base_pose?.position;
    if (bp && bp.length >= 3) expand(new THREE.Vector3(bp[0], bp[1], bp[2]));
  }

  if (count === 0) return null;
  return { min, max };
}

function isMobileRobot2DResult(data) {
  const context = data.benchmark?.context;
  if (context?.suite === "mobile_robot_2d_crossing") return true;
  return (data.robots || []).length > 0 &&
    data.robots.every((robot) => robot.robot_type === "sphere");
}

function isPlanarTopDownResult(data) {
  const context = data.benchmark?.context;
  if (context?.suite === "planar_manipulator_cross") return true;

  const robots = data.robots || [];
  return robots.length > 0 && robots.every(isPlanar3Robot);
}

function isPlanar3Robot(robot) {
  const type = String(robot.robot_type || "").toLowerCase();
  const urdfPath = String(
    robot.visual_urdf_path || robot.urdf_path || robot.collision_urdf_path || ""
  ).toLowerCase();
  return type === "planar3" || urdfPath.includes("planar3");
}

/**
 * True when obstacles match the fixed 1×2×1 panda cage (12 corner edge cylinders).
 */
function isStandardPandaCageObstacles(obstacles) {
  if (!obstacles || obstacles.length < 12) return false;
  const eps = 0.08;
  const has = (x, y, z) =>
    obstacles.some((o) => {
      if (o.type !== "cylinder") return false;
      const c = obstacleWorldCenter(o);
      if (!c) return false;
      return (
        Math.abs(c.x - x) < eps &&
        Math.abs(c.y - y) < eps &&
        Math.abs(c.z - z) < eps
      );
    });
  return (
    has(-0.5, -1, 0) &&
    has(0.5, -1, 0) &&
    has(-0.5, 1, 0) &&
    has(0.5, 1, 0) &&
    has(0, -1, -0.5) &&
    has(0, 1, 0.5)
  );
}

/**
 * Face the robot side of the standard cage with the long (Y) axis horizontal on screen.
 * Uses world +Z as camera up so +Y maps to screen left–right.
 */
function applyStandardPandaCageCamera(data) {
  if (!camera || !controls || !isStandardPandaCageObstacles(data.obstacles)) return false;

  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  const expand = (v) => {
    min.min(v);
    max.max(v);
  };

  for (const obs of data.obstacles) {
    const c = obstacleWorldCenter(obs);
    if (c) expand(c);
  }
  for (const robot of data.robots) {
    const bp = robot.base_pose?.position;
    if (bp && bp.length >= 3) expand(new THREE.Vector3(bp[0], bp[1], bp[2]));
  }

  const center = new THREE.Vector3().addVectors(min, max).multiplyScalar(0.5);
  const size = new THREE.Vector3().subVectors(max, min);
  const span = Math.max(size.x, size.y, size.z, 0.5);

  let nNeg = 0;
  let nPos = 0;
  for (const robot of data.robots) {
    const bp = robot.base_pose?.position;
    if (!bp || bp.length < 3) continue;
    if (bp[0] < -0.2) nNeg++;
    else if (bp[0] > 0.2) nPos++;
  }
  const eyeSign = nPos > nNeg ? -1 : 1;

  const dist = Math.max(2.4, span * 1.35);
  camera.up.set(0, 0, 1);
  controls.target.copy(center);
  camera.position.set(center.x + eyeSign * dist, center.y, center.z + 0.08);
  camera.lookAt(controls.target);
  controls.update();

  if (typeof controls.target0 !== "undefined") {
    controls.target0.copy(controls.target);
    controls.position0.copy(camera.position);
    controls.up0.copy(camera.up);
  }
  syncCameraInputsFromOrbit();
  return true;
}

function updateCameraClipAndShadow(span, dist) {
  camera.near = Math.max(0.01, dist / 1000);
  camera.far = Math.max(100, dist + span * 4);
  camera.updateProjectionMatrix();
  if (sunLight?.shadow?.camera) {
    const sc = sunLight.shadow.camera;
    const half = Math.max(10, span * 0.75);
    sc.left = -half;
    sc.right = half;
    sc.top = half;
    sc.bottom = -half;
    sc.far = Math.max(220, dist + span * 4);
    sc.updateProjectionMatrix();
  }
}

function applyFittedResultCamera(data) {
  if (!camera || !controls) return;
  const bounds = computeResultBounds(data);
  if (!bounds) return;

  const center = new THREE.Vector3().addVectors(bounds.min, bounds.max).multiplyScalar(0.5);
  const size = new THREE.Vector3().subVectors(bounds.max, bounds.min);
  const span = Math.max(size.x, size.y, size.z, 1);
  const fov = THREE.MathUtils.degToRad(camera.fov);
  const aspect = Math.max(camera.aspect || 1, 1e-6);

  if (isMobileRobot2DResult(data) || isPlanarTopDownResult(data)) {
    const fitY = Math.max(size.y, 1) / (2 * Math.tan(fov / 2));
    const fitX = Math.max(size.x, 1) / (2 * Math.tan(fov / 2) * aspect);
    const dist = Math.max(8, Math.max(fitX, fitY) * 1.2);
    // Top-down XY view: world +X stays horizontal, world +Y stays vertical.
    camera.up.set(0, 1, 0);
    controls.target.copy(center);
    camera.position.set(center.x, center.y, center.z + dist);
    camera.lookAt(controls.target);
    updateCameraClipAndShadow(span, dist);
  } else {
    const dir = new THREE.Vector3(1, -1, 0.7).normalize();
    const dist = Math.max(2.4, span * 1.8);
    camera.up.set(0, 0, 1);
    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(dir, dist);
    camera.lookAt(controls.target);
    updateCameraClipAndShadow(span, dist);
  }

  controls.update();
  if (typeof controls.target0 !== "undefined") {
    controls.target0.copy(controls.target);
    controls.position0.copy(camera.position);
    controls.up0.copy(camera.up);
  }
  syncCameraInputsFromOrbit();
}

function applyTopDownCameraToBounds(bounds) {
  if (!camera || !controls || !bounds) return;
  const center = new THREE.Vector3().addVectors(bounds.min, bounds.max).multiplyScalar(0.5);
  const size = new THREE.Vector3().subVectors(bounds.max, bounds.min);
  const fov = THREE.MathUtils.degToRad(camera.fov);
  const aspect = Math.max(camera.aspect || 1, 1e-6);
  const fitY = Math.max(size.y, 1) / (2 * Math.tan(fov / 2));
  const fitX = Math.max(size.x, 1) / (2 * Math.tan(fov / 2) * aspect);
  const span = Math.max(size.x, size.y, 1);
  const dist = Math.max(8, Math.max(fitX, fitY) * 1.22);

  camera.up.set(0, 1, 0);
  controls.target.set(center.x, center.y, CROSS_SECTION_Z);
  camera.position.set(center.x, center.y, CROSS_SECTION_Z + dist);
  camera.lookAt(controls.target);
  updateCameraClipAndShadow(span, dist);
  controls.update();

  if (typeof controls.target0 !== "undefined") {
    controls.target0.copy(controls.target);
    controls.position0.copy(camera.position);
    controls.up0.copy(camera.up);
  }
  syncCameraInputsFromOrbit();
}

function applyCrossSectionCamera() {
  if (!resultData) return;
  let bounds = null;
  if (crossSection2DGroup && crossSection2DGroup.children.length > 0) {
    crossSection2DGroup.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(crossSection2DGroup);
    if (!box.isEmpty()) bounds = { min: box.min, max: box.max };
  }
  applyTopDownCameraToBounds(bounds || computeResultBounds(resultData));
}

function applyResultCamera(data) {
  if (!applyStandardPandaCageCamera(data)) {
    applyFittedResultCamera(data);
  }
}

function initCameraPanel() {
  cameraPanelEl = document.getElementById("camera-panel");
  const applyBtn = document.getElementById("camera-apply");
  if (!cameraPanelEl || !applyBtn) return;

  applyBtn.addEventListener("click", () => {
    applyCameraFromInputs();
    syncCameraInputsFromOrbit();
  });

  cameraPanelEl.addEventListener("keydown", (e) => {
    if (e.key === "Enter" && e.target instanceof HTMLInputElement) {
      e.preventDefault();
      applyCameraFromInputs();
      syncCameraInputsFromOrbit();
    }
  });
}

/**
 * Create a pose matrix from position and quaternion_xyzw.
 */
function poseToMatrix(pose) {
  const pos = pose.position || [0, 0, 0];
  const q = pose.quaternion_xyzw || [0, 0, 0, 1];
  const m = new THREE.Matrix4();
  m.compose(
    new THREE.Vector3(pos[0], pos[1], pos[2]),
    new THREE.Quaternion(q[0], q[1], q[2], q[3]),
    new THREE.Vector3(1, 1, 1)
  );
  return m;
}

/**
 * Build obstacle meshes from result data.
 */
function buildObstacles(data) {
  const group = new THREE.Group();
  const obstacleMat = new THREE.MeshLambertMaterial({
    color: OBSTACLE_COLOR,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: OBSTACLE_OPACITY,
    depthWrite: false,
  });
  let sphereCount = 0;
  let cylinderCount = 0;

  const types = data.obstacles.map((o) => o.type || "?");
  console.log("[viewer] buildObstacles:", data.obstacles.length, "obstacles, types:", types);

  data.obstacles.forEach((obs) => {
    if (obs.type === "sphere") {
      const r = obs.geometry?.radius ?? 0.2;
      const geom = new THREE.SphereGeometry(r, 24, 24);
      const mesh = new THREE.Mesh(geom, obstacleMat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      const m = poseToMatrix(obs.pose || {});
      mesh.applyMatrix4(m);
      group.add(mesh);
      sphereCount++;
    } else if (obs.type === "cylinder") {
      const r = Math.max(obs.geometry?.radius ?? 0.1, 0.06);
      const halfHeight = obs.geometry?.half_height ?? 0.5;
      const height = 2 * halfHeight;
      const geom = new THREE.CylinderGeometry(r, r, height, 24);
      const mesh = new THREE.Mesh(geom, obstacleMat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      const pos = obs.pose?.position || [0, 0, 0];
      const axis = obs.pose?.axis || [0, 1, 0];
      mesh.position.set(pos[0], pos[1], pos[2]);
      const axisVec = new THREE.Vector3(axis[0], axis[1], axis[2]);
      const len = axisVec.length();
      if (len > 1e-6) {
        axisVec.normalize();
        const defaultY = new THREE.Vector3(0, 1, 0);
        const dot = defaultY.dot(axisVec);
        const quat = new THREE.Quaternion();
        if (dot < -0.9999) {
          quat.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
        } else {
          quat.setFromUnitVectors(defaultY, axisVec);
        }
        mesh.applyQuaternion(quat);
      }
      group.add(mesh);
      cylinderCount++;
    }
  });

  if (sphereCount > 0 || cylinderCount > 0) {
    console.log(`Built ${data.obstacles.length} obstacles (${sphereCount} spheres, ${cylinderCount} cylinders)`);
  }
  return group;
}

function asFiniteNumber(value, fallback) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function disposeMaterial(material) {
  if (Array.isArray(material)) {
    material.forEach((m) => m?.dispose?.());
    return;
  }
  material?.dispose?.();
}

function disposeObjectTree(root) {
  root.traverse((obj) => {
    obj.geometry?.dispose?.();
    disposeMaterial(obj.material);
  });
}

function clearCrossSection2DGroup() {
  if (!crossSection2DGroup) return;
  scene?.remove(crossSection2DGroup);
  disposeObjectTree(crossSection2DGroup);
  crossSection2DGroup = null;
}

function createCrossSectionFillMaterial(colorHex, opacity = 0.88) {
  return new THREE.MeshBasicMaterial({
    color: colorHex,
    transparent: opacity < 1,
    opacity,
    side: THREE.DoubleSide,
    depthTest: false,
    depthWrite: false,
  });
}

function createCrossSectionLineMaterial(colorHex, opacity = 0.82) {
  return new THREE.LineBasicMaterial({
    color: colorHex,
    transparent: opacity < 1,
    opacity,
    depthTest: false,
    depthWrite: false,
  });
}

function addCircle2DCrossSection(
  group,
  center,
  radius,
  colorHex,
  zOffset,
  opacity,
  renderOrder,
  outlineColor = scaleRgbHex(colorHex, 0.5)
) {
  if (!Number.isFinite(radius) || radius <= CROSS_SECTION_EPS) return false;
  const z = CROSS_SECTION_Z + zOffset;
  const segments = 48;
  const fill = new THREE.Mesh(
    new THREE.CircleGeometry(radius, segments),
    createCrossSectionFillMaterial(colorHex, opacity)
  );
  fill.position.set(center.x, center.y, z);
  fill.renderOrder = renderOrder;
  fill.frustumCulled = false;
  group.add(fill);

  const points = [];
  for (let i = 0; i < segments; ++i) {
    const a = (i / segments) * Math.PI * 2;
    points.push(new THREE.Vector3(
      center.x + Math.cos(a) * radius,
      center.y + Math.sin(a) * radius,
      z + 0.001
    ));
  }
  const outline = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(points),
    createCrossSectionLineMaterial(outlineColor)
  );
  outline.renderOrder = renderOrder + 1;
  outline.frustumCulled = false;
  group.add(outline);
  return true;
}

function addPolygon2DCrossSection(
  group,
  points,
  colorHex,
  zOffset,
  opacity,
  renderOrder,
  outlineColor = scaleRgbHex(colorHex, 0.5)
) {
  if (!points || points.length < 3) return false;
  const z = CROSS_SECTION_Z + zOffset;
  const shape = new THREE.Shape();
  shape.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; ++i) {
    shape.lineTo(points[i].x, points[i].y);
  }
  shape.closePath();

  const fill = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    createCrossSectionFillMaterial(colorHex, opacity)
  );
  fill.position.z = z;
  fill.renderOrder = renderOrder;
  fill.frustumCulled = false;
  group.add(fill);

  const outlinePoints = points.map((p) => new THREE.Vector3(p.x, p.y, z + 0.001));
  const outline = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(outlinePoints),
    createCrossSectionLineMaterial(outlineColor)
  );
  outline.renderOrder = renderOrder + 1;
  outline.frustumCulled = false;
  group.add(outline);
  return true;
}

function addSphereCrossSection2D(
  group,
  center,
  radius,
  colorHex,
  zOffset,
  opacity,
  renderOrder,
  outlineColor
) {
  const dz = center.z - CROSS_SECTION_Z;
  if (Math.abs(dz) > radius + CROSS_SECTION_EPS) return false;
  const sectionRadius = Math.sqrt(Math.max(0, radius * radius - dz * dz));
  return addCircle2DCrossSection(
    group,
    new THREE.Vector2(center.x, center.y),
    sectionRadius,
    colorHex,
    zOffset,
    opacity,
    renderOrder,
    outlineColor
  );
}

function orientedRectanglePoints(center, axis2, halfLength, halfWidth) {
  const a = axis2.clone().normalize();
  const p = new THREE.Vector2(-a.y, a.x);
  const c = new THREE.Vector2(center.x, center.y);
  return [
    c.clone().addScaledVector(a, -halfLength).addScaledVector(p, -halfWidth),
    c.clone().addScaledVector(a, halfLength).addScaledVector(p, -halfWidth),
    c.clone().addScaledVector(a, halfLength).addScaledVector(p, halfWidth),
    c.clone().addScaledVector(a, -halfLength).addScaledVector(p, halfWidth),
  ];
}

function addCylinderCrossSection2D(
  group,
  center,
  axis,
  radius,
  halfLength,
  colorHex,
  zOffset,
  opacity,
  renderOrder,
  outlineColor
) {
  if (!Number.isFinite(radius) || radius <= CROSS_SECTION_EPS) return false;
  const axis2 = new THREE.Vector2(axis.x, axis.y);
  if (axis2.lengthSq() <= CROSS_SECTION_EPS) {
    if (Math.abs(center.z - CROSS_SECTION_Z) > halfLength + CROSS_SECTION_EPS) {
      return false;
    }
    return addCircle2DCrossSection(
      group,
      new THREE.Vector2(center.x, center.y),
      radius,
      colorHex,
      zOffset,
      opacity,
      renderOrder,
      outlineColor
    );
  }

  const dz = center.z - CROSS_SECTION_Z;
  if (Math.abs(dz) > radius + CROSS_SECTION_EPS) return false;
  const halfWidth = Math.sqrt(Math.max(0, radius * radius - dz * dz));
  const points = orientedRectanglePoints(center, axis2, halfLength, halfWidth);
  return addPolygon2DCrossSection(
    group,
    points,
    colorHex,
    zOffset,
    opacity,
    renderOrder,
    outlineColor
  );
}

function addResultObstacles2DCrossSection(group, data) {
  const outlineColor = 0x555555;
  let count = 0;
  for (const obs of data.obstacles || []) {
    const center = obstacleWorldCenter(obs);
    if (!center) continue;
    if (obs.type === "sphere") {
      const radius = asFiniteNumber(obs.geometry?.radius, 0.2);
      if (addSphereCrossSection2D(
        group,
        center,
        radius,
        OBSTACLE_COLOR,
        0,
        OBSTACLE_OPACITY,
        10,
        outlineColor
      )) count++;
      continue;
    }
    if (obs.type === "cylinder") {
      const axis = obs.pose?.axis || [0, 1, 0];
      const axisVec = new THREE.Vector3(axis[0], axis[1], axis[2]);
      if (axisVec.lengthSq() > CROSS_SECTION_EPS) axisVec.normalize();
      else axisVec.set(0, 1, 0);
      const radius = asFiniteNumber(obs.geometry?.radius, 0.1);
      const halfLength = asFiniteNumber(obs.geometry?.half_height, 0.5);
      if (addCylinderCrossSection2D(
        group,
        center,
        axisVec,
        radius,
        halfLength,
        OBSTACLE_COLOR,
        0,
        OBSTACLE_OPACITY,
        10,
        outlineColor
      )) count++;
    }
  }
  return count;
}

function geometryPrimitiveKind(geometry) {
  if (!geometry) return "";
  const type = String(geometry.type || "").toLowerCase();
  const params = geometry.parameters || {};
  if (type.includes("sphere")) return "sphere";
  if (type.includes("cylinder")) return "cylinder";
  if (Number.isFinite(Number(params.height))) return "cylinder";
  if (Number.isFinite(Number(params.radius))) return "sphere";
  return "";
}

function geometryBoundingBox(geometry) {
  if (!geometry) return null;
  if (!geometry.boundingBox) geometry.computeBoundingBox?.();
  return geometry.boundingBox || null;
}

function addSphereMeshCrossSection2D(group, mesh, colorHex, zOffset, opacity, renderOrder) {
  const box = geometryBoundingBox(mesh.geometry);
  if (!box) return false;
  const center = box.getCenter(new THREE.Vector3()).applyMatrix4(mesh.matrixWorld);
  const size = box.getSize(new THREE.Vector3());
  const localRadius = asFiniteNumber(
    mesh.geometry.parameters?.radius,
    Math.max(size.x, size.y, size.z) * 0.5
  );
  const scale = mesh.getWorldScale(new THREE.Vector3());
  const worldRadius = localRadius * Math.max(Math.abs(scale.x), Math.abs(scale.y), Math.abs(scale.z), 1e-9);
  return addSphereCrossSection2D(
    group,
    center,
    worldRadius,
    colorHex,
    zOffset,
    opacity,
    renderOrder
  );
}

function largestDimensionIndex(size) {
  const dims = [size.x, size.y, size.z];
  let axis = 0;
  for (let i = 1; i < dims.length; ++i) {
    if (dims[i] > dims[axis]) axis = i;
  }
  return axis;
}

function unitAxis(index) {
  if (index === 0) return new THREE.Vector3(1, 0, 0);
  if (index === 1) return new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3(0, 0, 1);
}

function addCylinderMeshCrossSection2D(group, mesh, colorHex, zOffset, opacity, renderOrder) {
  const box = geometryBoundingBox(mesh.geometry);
  if (!box) return false;
  const size = box.getSize(new THREE.Vector3());
  const axisIndex = largestDimensionIndex(size);
  const dims = [size.x, size.y, size.z];
  const localAxis = unitAxis(axisIndex);
  const centerLocal = box.getCenter(new THREE.Vector3());
  const halfLengthLocal = dims[axisIndex] * 0.5;
  const start = centerLocal.clone()
    .addScaledVector(localAxis, -halfLengthLocal)
    .applyMatrix4(mesh.matrixWorld);
  const end = centerLocal.clone()
    .addScaledVector(localAxis, halfLengthLocal)
    .applyMatrix4(mesh.matrixWorld);
  const center = centerLocal.clone().applyMatrix4(mesh.matrixWorld);
  const axis2 = new THREE.Vector2(end.x - start.x, end.y - start.y);
  const halfLength = axis2.length() * 0.5;

  const radialDims = dims.filter((_, i) => i !== axisIndex);
  const localRadius = Math.max(radialDims[0] || 0, radialDims[1] || 0) * 0.5;
  const scale = mesh.getWorldScale(new THREE.Vector3());
  const worldRadius = localRadius * Math.max(Math.abs(scale.x), Math.abs(scale.y), Math.abs(scale.z), 1e-9);

  if (axis2.lengthSq() <= CROSS_SECTION_EPS) {
    const halfWorldLength = start.distanceTo(end) * 0.5;
    return addCylinderCrossSection2D(
      group,
      center,
      new THREE.Vector3(0, 0, 1),
      worldRadius,
      halfWorldLength,
      colorHex,
      zOffset,
      opacity,
      renderOrder
    );
  }

  return addCylinderCrossSection2D(
    group,
    center,
    new THREE.Vector3(axis2.x, axis2.y, 0),
    worldRadius,
    halfLength,
    colorHex,
    zOffset,
    opacity,
    renderOrder
  );
}

function collectPrimitiveMeshesUnder(root, collisionMode) {
  const meshes = new Set();
  const isBranch = collisionMode ? isUrdfColliderNode : isUrdfVisualNode;
  root.traverse((obj) => {
    if (!isBranch(obj)) return;
    obj.traverse((child) => {
      if (child.isMesh && geometryPrimitiveKind(child.geometry)) {
        meshes.add(child);
      }
    });
  });

  if (meshes.size === 0) {
    root.traverse((child) => {
      if (child.isMesh && geometryPrimitiveKind(child.geometry)) {
        meshes.add(child);
      }
    });
  }
  return Array.from(meshes);
}

function activeUrdfRootForCrossSection(entry) {
  if (showRobotCollisionGeometry && entry.collisionUrdfRobot) {
    return entry.collisionUrdfRobot;
  }
  return entry.urdfRobot;
}

function yawFromQuaternionXYZW(q) {
  if (!q || q.length < 4) return 0;
  const x = asFiniteNumber(q[0], 0);
  const y = asFiniteNumber(q[1], 0);
  const z = asFiniteNumber(q[2], 0);
  const w = asFiniteNumber(q[3], 1);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function robotBasePose2D(robot) {
  const pose = robot.base_pose || {};
  const pos = pose.position || [0, 0, 0];
  const yaw = Number.isFinite(Number(pose.yaw))
    ? Number(pose.yaw)
    : yawFromQuaternionXYZW(pose.quaternion_xyzw);
  return {
    position: new THREE.Vector2(
      asFiniteNumber(pos[0], 0),
      asFiniteNumber(pos[1], 0)
    ),
    yaw,
  };
}

function addPlanar3CollisionCircle(group, center, colorHex) {
  return addCircle2DCrossSection(
    group,
    center,
    PLANAR3_LINK_RADIUS,
    colorHex,
    0.014,
    0.9,
    34
  );
}

function displayConfigForEntry(entry) {
  return entry.playbackConfig ?? configAt(entry.robot, currentTimestep);
}

function addPlanar3RobotVisualCrossSection2D(group, entry, colorHex) {
  const cfg = displayConfigForEntry(entry);
  const base = robotBasePose2D(entry.robot);
  let joint = base.position.clone();
  let theta = base.yaw;
  let count = 0;

  addPlanar3CollisionCircle(group, joint, colorHex);

  for (let i = 0; i < 3; ++i) {
    theta += asFiniteNumber(cfg?.[i], 0);
    const axis = new THREE.Vector2(Math.cos(theta), Math.sin(theta));
    const center = joint.clone().addScaledVector(axis, PLANAR3_LINK_LENGTH * 0.5);
    const points = orientedRectanglePoints(
      center,
      axis,
      PLANAR3_LINK_LENGTH * 0.5,
      PLANAR3_LINK_RADIUS
    );
    if (addPolygon2DCrossSection(group, points, colorHex, 0.012, 0.9, 30)) {
      count++;
    }

    joint = joint.clone().addScaledVector(axis, PLANAR3_LINK_LENGTH);
    addPlanar3CollisionCircle(group, joint, colorHex);
  }

  return count;
}

function addPlanar3RobotCollisionCrossSection2D(group, entry, colorHex) {
  const cfg = displayConfigForEntry(entry);
  const base = robotBasePose2D(entry.robot);
  let joint = base.position.clone();
  let theta = base.yaw;
  let count = addPlanar3CollisionCircle(group, joint, colorHex) ? 1 : 0;
  const collisionOffsets = [0.075, 0.15, 0.225];

  for (let i = 0; i < 3; ++i) {
    theta += asFiniteNumber(cfg?.[i], 0);
    const axis = new THREE.Vector2(Math.cos(theta), Math.sin(theta));
    for (const offset of collisionOffsets) {
      const center = joint.clone().addScaledVector(axis, offset);
      if (addPlanar3CollisionCircle(group, center, colorHex)) count++;
    }
    joint = joint.clone().addScaledVector(axis, PLANAR3_LINK_LENGTH);
    if (addPlanar3CollisionCircle(group, joint, colorHex)) count++;
  }

  return count;
}

function addPlanar3RobotCrossSection2D(group, entry, colorHex) {
  if (showRobotCollisionGeometry) {
    return addPlanar3RobotCollisionCrossSection2D(group, entry, colorHex);
  }
  return addPlanar3RobotVisualCrossSection2D(group, entry, colorHex);
}

function addRobotEntry2DCrossSection(group, entry) {
  const colorHex = entry.colorHex ?? 0x3366cc;
  let count = 0;
  if (isPlanar3Robot(entry.robot)) {
    return addPlanar3RobotCrossSection2D(group, entry, colorHex);
  }

  if (entry.urdfRobot) {
    const root = activeUrdfRootForCrossSection(entry);
    root?.updateMatrixWorld?.(true);
    for (const mesh of collectPrimitiveMeshesUnder(root, showRobotCollisionGeometry)) {
      mesh.updateWorldMatrix?.(true, false);
      const kind = geometryPrimitiveKind(mesh.geometry);
      if (kind === "sphere") {
        if (addSphereMeshCrossSection2D(group, mesh, colorHex, 0.012, 0.9, 30)) count++;
      } else if (kind === "cylinder") {
        if (addCylinderMeshCrossSection2D(group, mesh, colorHex, 0.012, 0.9, 30)) count++;
      }
    }
    return count;
  }

  if (entry.mesh) {
    entry.mesh.updateWorldMatrix?.(true, false);
    const kind = geometryPrimitiveKind(entry.mesh.geometry);
    if (kind === "sphere") {
      if (addSphereMeshCrossSection2D(group, entry.mesh, colorHex, 0.012, 0.9, 30)) {
        count++;
      }
    }
  }
  return count;
}

function refreshCrossSection2D() {
  if (!scene || !resultData) return;
  clearCrossSection2DGroup();
  const group = new THREE.Group();
  group.name = "z0_cross_section_2d";
  addResultObstacles2DCrossSection(group, resultData);
  robotMeshes.forEach((entry) => {
    if (entry.playbackVisible !== false)
      addRobotEntry2DCrossSection(group, entry);
  });
  group.visible = showCrossSection2D;
  crossSection2DGroup = group;
  scene.add(group);
}

function createRobotLineMaterial(hex, styleKey = robotColorStyleKey(hex)) {
  const material = new THREE.LineBasicMaterial({
    color: hex,
    transparent: false,
    opacity: 1,
    depthWrite: true,
    depthTest: true,
  });
  material.userData.comotionSolidColor = hex;
  material.userData.comotionColorStyleKey = styleKey;
  return material;
}

function isUrdfVisualNode(obj) {
  return obj.isURDFVisual === true || obj.type === "URDFVisual";
}

function isUrdfColliderNode(obj) {
  return obj.isURDFCollider === true || obj.type === "URDFCollider";
}

/**
 * Toggle URDF visual vs collision branches. For panda_spherized.urdf, collision is mostly
 * sphere primitives (urdf-loader URDFCollider groups under each link, each holding a Mesh).
 * Visuals are URDFVisual groups (OBJ/STL meshes). Visibility is set on those groups; child
 * meshes inherit. Requires URDFLoader.parseCollision=true at load time.
 */
function setUrdfRobotGeometryVisibility(urdfRobot, collisionMode) {
  urdfRobot.traverse((obj) => {
    if (isUrdfVisualNode(obj)) obj.visible = !collisionMode;
    if (isUrdfColliderNode(obj)) obj.visible = !!collisionMode;
  });
}

function countUrdfColliderRoots(urdfRobot) {
  let n = 0;
  urdfRobot.traverse((obj) => {
    if (isUrdfColliderNode(obj)) n++;
  });
  return n;
}

function summarizeUrdfGeometry(urdfRobot) {
  const visualMeshes = new Set();
  const collisionMeshes = new Set();
  urdfRobot.traverse((branch) => {
    const target = isUrdfVisualNode(branch)
      ? visualMeshes
      : isUrdfColliderNode(branch)
        ? collisionMeshes
        : null;
    if (!target) return;
    branch.traverse((obj) => {
      if (obj.isMesh) target.add(obj);
    });
  });

  const summarize = (meshes) => {
    let triangles = 0;
    for (const mesh of meshes) {
      const drawCount = mesh.geometry?.index?.count ??
        mesh.geometry?.attributes?.position?.count ?? 0;
      triangles += Math.floor(drawCount / 3);
    }
    return { meshes: meshes.size, triangles };
  };
  return {
    visual: summarize(visualMeshes),
    collision: summarize(collisionMeshes),
  };
}

function applyRobotGeometryModeToScene() {
  const show3D = !showCrossSection2D;
  robotMeshes.forEach((entry) => {
    const { mesh, urdfRobot, collisionUrdfRobot } = entry;
    const showRobot = entry.playbackVisible !== false;
    if (mesh) mesh.visible = show3D && showRobot;

    if (!show3D || !showRobot) {
      if (urdfRobot) urdfRobot.visible = false;
      if (collisionUrdfRobot && collisionUrdfRobot !== urdfRobot) {
        collisionUrdfRobot.visible = false;
      }
      return;
    }

    if (collisionUrdfRobot && collisionUrdfRobot !== urdfRobot) {
      if (urdfRobot) {
        urdfRobot.visible = !showRobotCollisionGeometry;
        setUrdfRobotGeometryVisibility(urdfRobot, false);
      }
      collisionUrdfRobot.visible = showRobotCollisionGeometry;
      setUrdfRobotGeometryVisibility(collisionUrdfRobot, true);
      return;
    }

    if (urdfRobot) {
      urdfRobot.visible = true;
      setUrdfRobotGeometryVisibility(urdfRobot, showRobotCollisionGeometry);
    }
  });
}

function applySceneDisplayMode() {
  const show3D = !showCrossSection2D;
  obstacleMeshes.forEach((mesh) => {
    mesh.visible = show3D;
  });
  if (crossSection2DGroup) {
    crossSection2DGroup.visible = showCrossSection2D;
  }
  if (controls) {
    controls.noRotate = showCrossSection2D;
  }
  applyRobotGeometryModeToScene();
}

function syncGeometryToggleButton() {
  const btn = document.getElementById("geometry-toggle");
  if (!btn) return;
  btn.textContent = showRobotCollisionGeometry
    ? "Showing collision"
    : "Showing visual";
  btn.title = showRobotCollisionGeometry
    ? "Showing URDF collision primitives. Click for visual meshes."
    : "Showing URDF visual meshes. Click for collision geometry.";
}

function toggleRobotGeometryMode() {
  showRobotCollisionGeometry = !showRobotCollisionGeometry;
  console.info(
    `[viewer] Showing ${showRobotCollisionGeometry ? "collision" : "visual"} geometry`
  );
  if (showRobotCollisionGeometry) {
    let total = 0;
    robotMeshes.forEach(({ urdfRobot, collisionUrdfRobot }) => {
      const root = collisionUrdfRobot || urdfRobot;
      if (root) total += countUrdfColliderRoots(root);
    });
    if (total === 0) {
      console.warn(
        "[viewer] No URDF collision groups found (0 URDFCollider nodes). " +
          "Reload with cache disabled or hard-refresh; ensure urdf-loader.js loads collision " +
          "(parseCollision=true). Without <collision> geometry, only visuals exist."
      );
    }
  }
  applyRobotGeometryModeToScene();
  if (showCrossSection2D) refreshCrossSection2D();
  syncGeometryToggleButton();
}

function syncCrossSectionToggleButton() {
  const btn = document.getElementById("cross-section-toggle");
  if (!btn) return;
  btn.disabled = !resultData;
  btn.textContent = showCrossSection2D ? "3D view" : "2D z=0";
  btn.title = showCrossSection2D
    ? "Showing the z=0 cross-section. Click to restore the 3D scene."
    : "Show a flat z=0 cross-section of spheres and cylinders.";
}

function setCrossSection2DMode(enabled) {
  showCrossSection2D = !!enabled;
  if (showCrossSection2D) {
    refreshCrossSection2D();
  }
  applySceneDisplayMode();
  syncCrossSectionToggleButton();
  if (resultData) {
    if (showCrossSection2D) applyCrossSectionCamera();
    else applyResultCamera(resultData);
  }
}

function toggleCrossSection2DMode() {
  if (!resultData) return;
  setCrossSection2DMode(!showCrossSection2D);
}

function isStyledRobotMaterial(material, colorHex, requireDoubleSide, styleKey) {
  return material?.userData?.comotionSolidColor === colorHex &&
    material?.userData?.comotionColorStyleKey === styleKey &&
    material.transparent === false &&
    material.opacity === 1 &&
    material.depthWrite === true &&
    material.depthTest === true &&
    material.colorWrite !== false &&
    material.wireframe !== true &&
    (!requireDoubleSide || material.side === THREE.DoubleSide);
}

function disposeMaterialList(material) {
  if (Array.isArray(material)) {
    material.forEach((m) => m?.dispose?.());
    return;
  }
  material?.dispose?.();
}

function ensureMaterial(material, colorHex, requireDoubleSide, styleKey, factory) {
  if (Array.isArray(material)) {
    const styled = material.length > 0 &&
      material.every((m) => isStyledRobotMaterial(m, colorHex, requireDoubleSide, styleKey));
    if (styled) return material;
    disposeMaterialList(material);
    return material.length > 0 ? material.map(() => factory(colorHex, styleKey)) : factory(colorHex, styleKey);
  }
  if (isStyledRobotMaterial(material, colorHex, requireDoubleSide, styleKey)) {
    return material;
  }
  disposeMaterialList(material);
  return factory(colorHex, styleKey);
}

function applyRobotSolidColor(root, colorHex) {
  const styleKey = robotColorStyleKey(colorHex);
  const receiveShadows = currentRobotPaletteSpec().receiveShadows !== false;
  root?.traverse?.((c) => {
    if (c.isMesh) {
      c.material = ensureMaterial(
        c.material,
        colorHex,
        true,
        styleKey,
        createRobotSurfaceMaterial
      );
      c.visible = true;
      c.frustumCulled = false;
      c.castShadow = true;
      c.receiveShadow = receiveShadows;
      return;
    }
    if (c.isLineSegments || c.isLine || c.isLineLoop) {
      c.material = ensureMaterial(
        c.material,
        colorHex,
        false,
        styleKey,
        createRobotLineMaterial
      );
      c.visible = true;
      c.frustumCulled = false;
      c.castShadow = false;
      c.receiveShadow = false;
    }
  });
}

function refreshRobotSolidColors() {
  robotMeshes.forEach((entry) => {
    if (entry.playbackVisible === false) return;
    if ((entry.materialStylePassesRemaining ?? 0) <= 0) return;
    if (entry.mesh) applyRobotSolidColor(entry.mesh, entry.colorHex);
    for (const root of uniqueRobotRoots(entry.urdfRobot, entry.collisionUrdfRobot)) {
      applyRobotSolidColor(root, entry.colorHex);
    }
    entry.materialStylePassesRemaining--;
  });
}

function applyRobotPaletteToScene() {
  robotMeshes.forEach((entry, robotIndex) => {
    const colorHex = robotColorHexForIndex(robotIndex);
    entry.colorHex = colorHex;
    entry.materialStylePassesRemaining = 120;
    if (entry.mesh) applyRobotSolidColor(entry.mesh, colorHex);
    for (const root of uniqueRobotRoots(entry.urdfRobot, entry.collisionUrdfRobot)) {
      applyRobotSolidColor(root, colorHex);
    }
  });
  if (showCrossSection2D) refreshCrossSection2D();
}

function setRobotColorPalette(paletteId) {
  if (!ROBOT_COLOR_PALETTE_BY_ID.has(paletteId)) return;
  selectedRobotPaletteId = paletteId;
  if (playbackMode === "arc") {
    robotMeshes.forEach((entry) => {
      entry.colorHex = null;
    });
    updateRobotsForTimestep();
  } else {
    applyRobotPaletteToScene();
  }
}

function initRobotPaletteSelect() {
  const select = document.getElementById("palette-select");
  if (!select) return;
  select.innerHTML = "";
  for (const palette of ROBOT_COLOR_PALETTES) {
    const option = document.createElement("option");
    option.value = palette.id;
    option.textContent = palette.label;
    select.appendChild(option);
  }
  select.value = selectedRobotPaletteId;
  select.addEventListener("change", (event) => {
    setRobotColorPalette(event.target.value);
  });
}

/**
 * Build placeholder robot when URDF is unavailable.
 * For sphere robots (robot_type === "sphere"), creates a sphere with robot_radius.
 * Otherwise creates a small box.
 */
function buildPlaceholderRobot(robot, colorHex) {
  const isSphere = robot.robot_type === "sphere";
  const radius = robot.robot_radius ?? 0.5;
  const geom = isSphere
    ? new THREE.SphereGeometry(radius, 24, 24)
    : new THREE.BoxGeometry(0.15, 0.15, 0.15);
  const mesh = new THREE.Mesh(geom, createRobotSurfaceMaterial(colorHex));
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return {
    mesh,
    robot,
    urdfRobot: null,
    colorHex,
    playbackConfig: null,
    playbackVisible: true,
    materialStylePassesRemaining: 120,
  };
}

/**
 * Apply joint config to URDF robot.
 */
function collectUrdfJoints(urdfRobot) {
  if (urdfRobot.userData?.jointLookup) return urdfRobot.userData.jointLookup;
  const joints = { ...(urdfRobot.joints || {}) };
  urdfRobot.traverse?.((obj) => {
    if ((obj.isURDFJoint === true || obj.type === "URDFJoint") && obj.name) {
      joints[obj.name] = obj;
    }
  });
  urdfRobot.userData = urdfRobot.userData || {};
  urdfRobot.userData.jointLookup = joints;
  return joints;
}

function applyJointConfig(urdfRobot, robot, config) {
  if (!config || config.length === 0) return;
  const jointNames =
    Array.isArray(robot.joint_names) && robot.joint_names.length > 0
      ? robot.joint_names
      : PANDA_JOINT_NAMES;
  const n = Math.min(config.length, jointNames.length);
  const values = {};
  for (let i = 0; i < n; i++) {
    values[jointNames[i]] = config[i];
  }

  let applied = false;
  if (typeof urdfRobot.setJointValues === "function") {
    try {
      urdfRobot.setJointValues(values);
      applied = true;
    } catch (err) {
      if (!robot._jointApplyWarned) {
        console.warn("[viewer] setJointValues failed; falling back to per-joint updates", err);
        robot._jointApplyWarned = true;
      }
    }
  }

  const joints = collectUrdfJoints(urdfRobot);
  for (const [name, value] of Object.entries(values)) {
    const joint = joints[name];
    if (joint && typeof joint.setJointValue === "function") {
      joint.setJointValue(value);
      applied = true;
    }
  }

  if (!applied && !robot._jointApplyWarned) {
    console.warn("[viewer] no matching URDF joints found for", robot.name || robot.robot_type, jointNames);
    robot._jointApplyWarned = true;
  }
  urdfRobot.updateMatrixWorld?.(true);
}

function uniqueRobotRoots(urdfRobot, collisionUrdfRobot) {
  if (collisionUrdfRobot && collisionUrdfRobot !== urdfRobot) {
    return urdfRobot ? [urdfRobot, collisionUrdfRobot] : [collisionUrdfRobot];
  }
  return urdfRobot ? [urdfRobot] : [];
}

function applyRobotConfig(entry, cfg) {
  const { mesh, robot, urdfRobot, collisionUrdfRobot } = entry;
  entry.playbackConfig = cfg;
  const pose = robot.base_pose || {
    position: [0, 0, 0],
    quaternion_xyzw: [0, 0, 0, 1],
  };
  const pos = pose.position || [0, 0, 0];
  const q = pose.quaternion_xyzw || [0, 0, 0, 1];
  if (urdfRobot) {
    for (const root of uniqueRobotRoots(urdfRobot, collisionUrdfRobot)) {
      root.position.set(pos[0], pos[1], pos[2]);
      root.quaternion.set(q[0], q[1], q[2], q[3]);
      applyJointConfig(root, robot, cfg);
    }
  } else if (mesh) {
    if (robot.robot_type === "sphere" && cfg && cfg.length >= 3) {
      mesh.position.set(cfg[0], cfg[1], cfg[2]);
    } else {
      mesh.position.set(pos[0], pos[1], pos[2]);
    }
    mesh.quaternion.set(q[0], q[1], q[2], q[3]);
  }
}

function setRobotEntryColor(entry, colorHex) {
  if (entry.colorHex === colorHex) return;
  entry.colorHex = colorHex;
  entry.materialStylePassesRemaining = 3;
  if (entry.mesh) applyRobotSolidColor(entry.mesh, colorHex);
  for (const root of uniqueRobotRoots(entry.urdfRobot, entry.collisionUrdfRobot)) {
    applyRobotSolidColor(root, colorHex);
  }
}

function updateSolutionPlayback() {
  robotMeshes.forEach((entry, robotIndex) => {
    entry.playbackVisible = true;
    applyRobotConfig(entry, configAt(entry.robot, currentTimestep));
    setRobotEntryColor(entry, robotColorHexForIndex(robotIndex));
  });
}

function updateArcPlayback() {
  const frame = arcTimeline[currentTimestep];
  if (!frame) return;
  const iteration =
    resultData.arc_visualization.iterations[frame.iterationIndex];

  if (frame.phase === "paths") {
    robotMeshes.forEach((entry, robotIndex) => {
      const firstConflict = firstReachedConflict(
        iteration,
        robotIndex,
        frame.timestep
      );
      const robotTimestep = firstConflict
        ? Number(firstConflict.timestep)
        : frame.timestep;
      entry.playbackVisible = true;
      applyRobotConfig(
        entry,
        configAtPath(iteration.paths[robotIndex], robotTimestep)
      );
      setRobotEntryColor(
        entry,
        frame.solution
          ? ARC_SOLUTION_COLOR
          : firstConflict
            ? ARC_CONFLICT_COLOR
            : ARC_PATH_COLOR
      );
    });
  } else {
    robotMeshes.forEach((entry, robotIndex) => {
      const repairIndex = iteration.repairs.findIndex((repair) =>
        repair.robots.includes(robotIndex)
      );
      if (repairIndex < 0) {
        entry.playbackVisible = false;
        return;
      }
      const repair = iteration.repairs[repairIndex];
      const robotPathIndex = repair.robots.indexOf(robotIndex);
      entry.playbackVisible = true;
      applyRobotConfig(
        entry,
        configAtPath(repair.paths[robotPathIndex], frame.timestep)
      );
      const colorIndex = Number.isInteger(repair.conflict_index)
        ? repair.conflict_index
        : repairIndex;
      setRobotEntryColor(
        entry,
        ARC_GROUP_COLORS[colorIndex % ARC_GROUP_COLORS.length]
      );
    });
  }

}

/**
 * Update robot poses for the active playback mode.
 */
function updateRobotsForTimestep() {
  if (!resultData) return;
  if (playbackMode === "arc") updateArcPlayback();
  else updateSolutionPlayback();
  applyRobotGeometryModeToScene();
  if (showCrossSection2D) refreshCrossSection2D();
}

/**
 * Update UI.
 */
function playbackFrameCount() {
  if (!resultData) return 0;
  return playbackMode === "arc"
    ? arcTimeline.length
    : Math.max(0, Number(resultData.timesteps) || 0);
}

function clearArcLegend() {
  const legend = document.getElementById("arc-legend");
  if (legend) legend.replaceChildren();
}

function addArcLegendItem(colorHex, label) {
  const legend = document.getElementById("arc-legend");
  if (!legend) return;
  const item = document.createElement("span");
  item.className = "arc-legend-item";
  const swatch = document.createElement("span");
  swatch.className = "arc-swatch";
  swatch.style.backgroundColor = `#${colorHex.toString(16).padStart(6, "0")}`;
  const text = document.createElement("span");
  text.textContent = label;
  item.append(swatch, text);
  legend.appendChild(item);
}

function updateArcStageBox(frame) {
  const box = document.getElementById("arc-stage-box");
  const label = document.getElementById("arc-stage-label");
  if (!box || !label) return;

  if (playbackMode !== "arc" || !frame) {
    box.hidden = true;
    return;
  }

  const complete = frame.solution === true;
  const resolving = !complete && frame.phase === "repairs";
  label.textContent = complete
    ? "Complete"
    : resolving
      ? "Conflict Resolution"
      : "Conflict Detection";
  label.classList.toggle("conflict-detection", !complete && !resolving);
  label.classList.toggle("conflict-resolution", resolving);
  label.classList.toggle("complete", complete);
  box.hidden = false;
}

function initArcStageBox() {
  const box = document.getElementById("arc-stage-box");
  if (!box) return;

  const toolbar = document.getElementById("toolbar");
  const initialTop = Math.max(12, (toolbar?.getBoundingClientRect().bottom || 0) + 12);
  box.style.left = "12px";
  box.style.top = `${initialTop}px`;

  let drag = null;
  const clampPosition = (left, top) => ({
    left: Math.max(0, Math.min(left, window.innerWidth - box.offsetWidth)),
    top: Math.max(0, Math.min(top, window.innerHeight - box.offsetHeight)),
  });

  box.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    event.preventDefault();
    event.stopPropagation();
    const rect = box.getBoundingClientRect();
    drag = {
      pointerId: event.pointerId,
      offsetX: event.clientX - rect.left,
      offsetY: event.clientY - rect.top,
    };
    box.setPointerCapture(event.pointerId);
  });

  box.addEventListener("pointermove", (event) => {
    if (!drag || event.pointerId !== drag.pointerId) return;
    event.preventDefault();
    event.stopPropagation();
    const position = clampPosition(
      event.clientX - drag.offsetX,
      event.clientY - drag.offsetY
    );
    box.style.left = `${position.left}px`;
    box.style.top = `${position.top}px`;
  });

  const finishDrag = (event) => {
    if (!drag || event.pointerId !== drag.pointerId) return;
    drag = null;
    if (box.hasPointerCapture(event.pointerId)) {
      box.releasePointerCapture(event.pointerId);
    }
  };
  box.addEventListener("pointerup", finishDrag);
  box.addEventListener("pointercancel", finishDrag);

  window.addEventListener("resize", () => {
    if (box.hidden) return;
    const rect = box.getBoundingClientRect();
    const position = clampPosition(rect.left, rect.top);
    box.style.left = `${position.left}px`;
    box.style.top = `${position.top}px`;
  });
}

function updateArcPanel() {
  const panel = document.getElementById("arc-panel");
  const stage = document.getElementById("arc-stage");
  const detail = document.getElementById("arc-detail");
  if (!panel || !stage || !detail) return;
  if (playbackMode !== "arc" || arcTimeline.length === 0) {
    panel.hidden = true;
    updateArcStageBox(null);
    clearArcLegend();
    return;
  }

  panel.hidden = false;
  clearArcLegend();
  const frame = arcTimeline[currentTimestep];
  updateArcStageBox(frame);
  const iteration =
    resultData.arc_visualization.iterations[frame.iterationIndex];
  const roundLabel =
    `Round ${frame.iterationIndex + 1} of ` +
    `${resultData.arc_visualization.iterations.length}`;
  const planner = resultData.arc_visualization.planner || resultData.solver || "ARC";

  if (frame.phase === "paths") {
    if (frame.solution) {
      stage.textContent = `${planner} · ${roundLabel} · Solution found`;
      detail.textContent =
        `No conflicts in the complete path set · path timestep ` +
        `${frame.timestep} of ${frame.phaseEnd}`;
      addArcLegendItem(ARC_SOLUTION_COLOR, "valid solution");
      return;
    }

    stage.textContent = `${planner} · ${roundLabel} · Conflict detection`;
    if (!iteration.conflict_scan_completed) {
      detail.textContent =
        `Saved path set · path timestep ${frame.timestep} of ${frame.phaseEnd} · ` +
        `conflict scan did not complete`;
      addArcLegendItem(ARC_PATH_COLOR, "unchecked path");
      return;
    }

    const reached = iteration.conflicts.filter(
      (conflict) => Number(conflict.timestep) <= frame.timestep
    ).length;
    const conflictPauseTimesteps = arcFrameDurationTimesteps(
      resultData,
      frame,
      playbackSpeedMultiplier
    );
    const conflictPause =
      conflictPauseTimesteps > 1
        ? ` · paused for ${fmtNum(conflictPauseTimesteps, 2)} playback steps`
        : "";
    detail.textContent =
      `Path timestep ${frame.timestep} of ${frame.phaseEnd} · ` +
      `${reached} of ${iteration.conflicts.length} conflicts reached` +
      conflictPause;
    addArcLegendItem(ARC_PATH_COLOR, "advancing");
    iteration.conflicts.forEach((conflict, conflictIndex) => {
      addArcLegendItem(
        ARC_CONFLICT_COLOR,
        `C${conflictIndex + 1}: robots ${conflictRobots(conflict).join(", ")} ` +
          `at t=${conflict.timestep}`
      );
    });
    return;
  }

  const workerCount = resultData.arc_visualization.workers || 1;
  stage.textContent =
    `${planner} · ${roundLabel} · ` +
    `${workerCount > 1 ? "Parallel local repair" : "Local repair"}`;
  detail.textContent =
    `Local step ${frame.timestep} of ${frame.phaseEnd} · ` +
    `${iteration.repairs.length} subproblem` +
    `${iteration.repairs.length === 1 ? "" : "s"} · ${workerCount} worker` +
    `${workerCount === 1 ? "" : "s"}`;
  iteration.repairs.forEach((repair, repairIndex) => {
    const colorIndex = Number.isInteger(repair.conflict_index)
      ? repair.conflict_index
      : repairIndex;
    addArcLegendItem(
      ARC_GROUP_COLORS[colorIndex % ARC_GROUP_COLORS.length],
      `C${colorIndex + 1}: robots ${repair.robots.join(", ")}`
    );
  });
}

function syncPlaybackModeControl() {
  if (!playbackModeSelectEl) return;
  const arcOption = playbackModeSelectEl.querySelector('option[value="arc"]');
  if (arcOption) arcOption.disabled = arcTimeline.length === 0;
  playbackModeSelectEl.value = playbackMode;
  const paletteSelect = document.getElementById("palette-select");
  if (paletteSelect) paletteSelect.disabled = playbackMode === "arc";
}

function stopPlayback() {
  isPlaying = false;
  if (playTimerId !== null) clearTimeout(playTimerId);
  playTimerId = null;
}

function setPlaybackMode(mode) {
  const nextMode = mode === "arc" && arcTimeline.length > 0 ? "arc" : "solution";
  stopPlayback();
  playbackMode = nextMode;
  currentTimestep = 0;
  syncPlaybackModeControl();
  updateRobotsForTimestep();
  updateUI();
}

function updateUI() {
  if (!resultData) {
    if (timestepEl) timestepEl.textContent = "Timestep 0 / 0";
    if (playPauseBtn) {
      playPauseBtn.textContent = "Play";
      playPauseBtn.disabled = true;
    }
    if (sliderEl) sliderEl.disabled = true;
    updateArcPanel();
    syncPlaybackModeControl();
    syncCrossSectionToggleButton();
    return;
  }
  const frameCount = playbackFrameCount();
  const maxT = Math.max(0, frameCount - 1);
  if (timestepEl) {
    timestepEl.textContent = playbackMode === "arc"
      ? `Frame ${currentTimestep + 1} / ${Math.max(1, frameCount)}`
      : `Timestep ${currentTimestep} / ${maxT}`;
  }
  if (sliderEl) {
    sliderEl.disabled = frameCount <= 1;
    sliderEl.value = currentTimestep;
    sliderEl.max = maxT;
  }
  if (playPauseBtn) {
    playPauseBtn.disabled = frameCount <= 1;
    playPauseBtn.textContent = isPlaying ? "Pause" : "Play";
  }
  syncPlaybackModeControl();
  updateArcPanel();
  syncCrossSectionToggleButton();
}

/**
 * Set timestep and update scene/UI.
 */
function setTimestep(t) {
  if (!resultData) return;
  const maxT = Math.max(0, playbackFrameCount() - 1);
  currentTimestep = Math.max(0, Math.min(t, maxT));
  updateRobotsForTimestep();
  updateUI();
}

/**
 * Step forward/backward.
 */
function step(delta) {
  setTimestep(currentTimestep + delta);
}

function currentFrameDurationTimesteps() {
  if (playbackMode !== "arc") return 1;
  return arcFrameDurationTimesteps(
    resultData,
    arcTimeline[currentTimestep],
    playbackSpeedMultiplier
  );
}

function currentPlaybackDelayMs() {
  return (
    (1000 * currentFrameDurationTimesteps()) /
    (PLAY_FPS * playbackSpeedMultiplier)
  );
}

function scheduleNextPlaybackStep() {
  if (!isPlaying) return;
  playTimerId = setTimeout(() => {
    playTimerId = null;
    if (!isPlaying) return;
    if (currentTimestep >= playbackFrameCount() - 1) {
      stopPlayback();
      updateUI();
      return;
    }
    step(1);
    scheduleNextPlaybackStep();
  }, currentPlaybackDelayMs());
}

function restartPlaybackTimer() {
  if (!isPlaying) return;
  if (playTimerId !== null) clearTimeout(playTimerId);
  playTimerId = null;
  scheduleNextPlaybackStep();
}

/**
 * Toggle play/pause.
 */
function togglePlay() {
  if (playbackFrameCount() <= 1) return;
  isPlaying = !isPlaying;
  if (isPlaying) {
    scheduleNextPlaybackStep();
  } else {
    if (playTimerId !== null) clearTimeout(playTimerId);
    playTimerId = null;
  }
  updateUI();
}

/**
 * Keyboard handler.
 */
function onKeyDown(e) {
  if (!resultData) return;
  switch (e.code) {
    case "ArrowLeft":
      step(e.shiftKey ? -10 : -1);
      e.preventDefault();
      break;
    case "ArrowRight":
      step(e.shiftKey ? 10 : 1);
      e.preventDefault();
      break;
    case "Home":
      setTimestep(0);
      e.preventDefault();
      break;
    case "End":
      setTimestep(playbackFrameCount() - 1);
      e.preventDefault();
      break;
    case "Space":
      togglePlay();
      e.preventDefault();
      break;
  }
}

/**
 * Initialize Three.js scene.
 */
function initScene() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
  camera.position.set(2, 2, 2);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  document.getElementById("canvas-container").appendChild(renderer.domElement);

  controls = new TrackballControls(camera, renderer.domElement);
  controls.rotateSpeed = 1.35;
  controls.staticMoving = false;
  controls.dynamicDampingFactor = 0.05;

  ambientLight = new THREE.AmbientLight(0xffffff, 0.38);
  scene.add(ambientLight);

  hemisphereLight = new THREE.HemisphereLight(0xffffff, 0xffffff, 0.48);
  scene.add(hemisphereLight);

  sunLight = new THREE.DirectionalLight(0xffffff, 1.15);
  sunLight.position.set(60, -80, 120);
  sunLight.target.position.set(0, 0, 0);
  scene.add(sunLight.target);
  scene.add(sunLight);
  sunLight.castShadow = true;
  sunLight.shadow.mapSize.set(2048, 2048);
  sunLight.shadow.bias = -0.00015;
  sunLight.shadow.normalBias = 0.015;
  const sc = sunLight.shadow.camera;
  sc.near = 0.5;
  sc.far = 220;
  sc.left = -10;
  sc.right = 10;
  sc.top = 10;
  sc.bottom = -10;
  sc.updateProjectionMatrix();

  fillLight = new THREE.DirectionalLight(0xffffff, 0.72);
  fillLight.position.set(-80, 55, 75);
  fillLight.target.position.set(0, 0, 0);
  scene.add(fillLight.target);
  scene.add(fillLight);
  fillLight.castShadow = false;

  rimLight = new THREE.DirectionalLight(0xe8fbff, 0.42);
  rimLight.position.set(-50, -90, 95);
  rimLight.target.position.set(0, 0, 0);
  scene.add(rimLight.target);
  scene.add(rimLight);
  rimLight.castShadow = false;

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    controls.handleResize();
  });

  window.addEventListener("keydown", onKeyDown);
}

function syncLightingToggleButton() {
  const btn = document.getElementById("lighting-toggle");
  if (!btn) return;
  btn.textContent = lightingKeyMode ? "Key + fill" : "Ambient only";
  btn.title = lightingKeyMode
    ? "Angled key, broad fill, and shadows. Click for ambient-only lighting."
    : "Even ambient lighting. Click to restore key, fill, and shadows.";
}

function applyLightingMode() {
  if (!ambientLight || !hemisphereLight || !sunLight || !fillLight || !rimLight) return;
  if (lightingKeyMode) {
    ambientLight.intensity = 0.38;
    hemisphereLight.intensity = 0.48;
    sunLight.intensity = 1.15;
    sunLight.visible = true;
    sunLight.castShadow = true;
    fillLight.intensity = 0.72;
    fillLight.visible = true;
    rimLight.intensity = 0.42;
    rimLight.visible = true;
  } else {
    ambientLight.intensity = 1.15;
    hemisphereLight.intensity = 0.18;
    sunLight.intensity = 0;
    sunLight.visible = false;
    sunLight.castShadow = false;
    fillLight.intensity = 0;
    fillLight.visible = false;
    rimLight.intensity = 0;
    rimLight.visible = false;
  }
}

function toggleLightingMode() {
  lightingKeyMode = !lightingKeyMode;
  applyLightingMode();
  syncLightingToggleButton();
}

function withTrailingSlash(url) {
  return url.endsWith("/") ? url : `${url}/`;
}

function rawRobotVisualUrdfPath(robot) {
  return robot.visual_urdf_path || robot.urdf_path || "";
}

function isPandaRobot(robot) {
  const type = String(robot.robot_type || "").toLowerCase();
  const path = String(rawRobotVisualUrdfPath(robot)).toLowerCase();
  return type === "panda" || path.includes("/panda/");
}

function isPandaSpherizedUrdf(path) {
  const normalized = String(path || "").replace(/\\/g, "/").toLowerCase();
  return normalized === "panda/panda_spherized.urdf" ||
    normalized.endsWith("/panda/panda_spherized.urdf");
}

function robotVisualUrdfCandidates(robot) {
  const path = rawRobotVisualUrdfPath(robot);
  if (!path) return [];
  const candidates =
    isPandaRobot(robot) && isPandaSpherizedUrdf(path)
      ? [DEFAULT_PANDA_VISUAL_URDF_PATH, path]
      : [path];
  return candidates.filter((candidate, index) => candidates.indexOf(candidate) === index);
}

function robotVisualUrdfPath(robot) {
  return robotVisualUrdfCandidates(robot)[0] || "";
}

function robotCollisionUrdfPath(robot) {
  if (robot.collision_urdf_path) return robot.collision_urdf_path;
  if (robot.planning_urdf_path) return robot.planning_urdf_path;

  const visualPath = rawRobotVisualUrdfPath(robot);
  const lowerType = String(robot.robot_type || "").toLowerCase();
  const lowerPath = String(visualPath || "").toLowerCase();
  const isPlanar3 = lowerType === "planar3" || lowerPath.includes("planar3");
  if (!isPlanar3) return "";

  if (/planar3\.urdf$/i.test(visualPath)) {
    return visualPath.replace(/planar3\.urdf$/i, "planar3_spherized.urdf");
  }
  if (!visualPath) return "resources/planar3/planar3_spherized.urdf";
  return "";
}

function assetUrl(assetBase, path) {
  const url = /^https?:\/\//i.test(path) ? new URL(path) : new URL(path, assetBase);
  const normalizedPath = url.pathname.replace(/\\/g, "/").toLowerCase();
  if (normalizedPath.endsWith("/panda/panda.urdf") ||
      normalizedPath.endsWith("/panda/panda_spherized.urdf")) {
    url.searchParams.set("assetRevision", PANDA_RESOURCE_REVISION);
  }
  return url.href;
}

/**
 * Base URL for repo-root assets (URDF, meshes, result JSON from ?file=).
 * - Optional override: ?assetBase=https://host/repo/ or ?assetBase=../ (resolved vs this page)
 * - If this module is served as .../viewer/js/app.js, repo root is two levels up (../../).
 * - If the page URL path contains "viewer" (e.g. /viewer/), use the parent directory.
 * - Otherwise use the site origin. For python -m http.server --directory viewer, add a
 *   symlink viewer/resources -> ../resources so /resources/... exists on that server.
 */
function getAssetBaseUrl() {
  const params = new URLSearchParams(window.location.search);
  const override = params.get("assetBase");
  if (override) {
    const resolved = new URL(override, window.location.href).href;
    return withTrailingSlash(resolved);
  }
  try {
    const moduleUrl = new URL(import.meta.url);
    if (moduleUrl.pathname.includes("/viewer/js/")) {
      return withTrailingSlash(new URL("../../", moduleUrl).href);
    }
  } catch (_) {
    /* import.meta.url unavailable */
  }
  if (window.location.pathname.includes("/viewer")) {
    return withTrailingSlash(new URL("../", window.location.href).href);
  }
  return withTrailingSlash(`${window.location.origin}/`);
}

/**
 * Wait until every loadMeshCb (OBJ/STL) has invoked the URDF completion callback.
 * LoadingManager.onLoad is unreliable here (ordering with fetch, duplicates, etc.).
 */
async function waitForUrdfMeshCallbacks(getPending, timeoutMs = 120000) {
  const t0 = performance.now();
  while (getPending() > 0) {
    if (performance.now() - t0 > timeoutMs) {
      console.warn("[viewer] timed out waiting for URDF mesh files; coloring whatever loaded");
      break;
    }
    await new Promise((r) => setTimeout(r, 16));
  }
}

/**
 * Load URDF, wait until async mesh files finish populating the tree, then apply robot color.
 * URDFLoader's load() resolves after parse; geometry is added in mesh load callbacks later.
 */
async function loadUrdfRobotWithSolidColor(assetBase, urdfUrl, colorHex, geometryKind) {
  const loader = createURDFLoader(assetBase, colorHex);
  const getPending =
    typeof loader.getPendingMeshLoads === "function"
      ? () => loader.getPendingMeshLoads()
      : () => 0;
  const urdfRobot = await loadURDFAsync(loader, urdfUrl, {
    parseVisual: geometryKind === "visual",
    parseCollision: geometryKind === "collision",
  });
  await waitForUrdfMeshCallbacks(getPending);
  applyRobotSolidColor(urdfRobot, colorHex);
  setUrdfRobotGeometryVisibility(urdfRobot, showRobotCollisionGeometry);
  const geometrySummary = summarizeUrdfGeometry(urdfRobot);
  urdfRobot.userData.comotionGeometrySummary = geometrySummary;
  console.info(
    "[viewer] Loaded URDF geometry",
    urdfUrl,
    JSON.stringify(geometrySummary)
  );
  return urdfRobot;
}

async function loadFirstUrdfRobotWithSolidColor(assetBase, urdfPaths, colorHex) {
  let firstError = null;
  for (const urdfPath of urdfPaths) {
    try {
      const urdfUrl = assetUrl(assetBase, urdfPath);
      return {
        urdfPath,
        urdfRobot: await loadUrdfRobotWithSolidColor(
          assetBase,
          urdfUrl,
          colorHex,
          "visual"
        ),
      };
    } catch (err) {
      if (!firstError) firstError = err;
      console.warn("[viewer] URDF load failed for", urdfPath, err);
    }
  }
  throw firstError || new Error("No URDF paths available");
}

/**
 * Load result and build scene. Loads URDFs when urdf_path is present.
 */
async function loadResult(data) {
  stopPlayback();
  resultData = data;
  arcTimeline = buildArcTimeline(data);
  playbackMode = arcTimeline.length > 0 ? "arc" : "solution";
  currentTimestep = 0;
  syncPlaybackModeControl();

  clearCrossSection2DGroup();
  obstacleMeshes.forEach((m) => scene.remove(m));
  robotMeshes.forEach((r) => {
    if (r.urdfRobot) scene.remove(r.urdfRobot);
    if (r.collisionUrdfRobot && r.collisionUrdfRobot !== r.urdfRobot) {
      scene.remove(r.collisionUrdfRobot);
    }
    if (r.mesh) scene.remove(r.mesh);
  });
  obstacleMeshes = [];
  robotMeshes = [];

  const obsGroup = buildObstacles(data);
  scene.add(obsGroup);
  obstacleMeshes.push(obsGroup);

  const assetBase = getAssetBaseUrl();

  for (const [robotIndex, robot] of data.robots.entries()) {
    const robotColor = robotColorHexForIndex(robotIndex);
    const urdfPaths = robotVisualUrdfCandidates(robot);
    const collisionUrdfPath = robotCollisionUrdfPath(robot);
    if (urdfPaths.length > 0) {
      try {
        const { urdfPath, urdfRobot } = await loadFirstUrdfRobotWithSolidColor(
          assetBase,
          urdfPaths,
          robotColor
        );
        if (urdfPath !== rawRobotVisualUrdfPath(robot)) {
          console.info(
            "[viewer] Using panda visual URDF",
            urdfPath,
            "for",
            robot.name || robot.robot_type
          );
        }
        let collisionUrdfRobot = null;
        if (collisionUrdfPath) {
          try {
            const collisionUrl = assetUrl(assetBase, collisionUrdfPath);
            collisionUrdfRobot = await loadUrdfRobotWithSolidColor(
              assetBase,
              collisionUrl,
              robotColor,
              "collision"
            );
            console.info(
              "[viewer] Loaded collision-only geometry for",
              robot.name || robot.robot_type,
              collisionUrdfPath
            );
          } catch (err) {
            console.warn(
              "[viewer] Collision URDF load failed for",
              collisionUrdfPath,
              err
            );
          }
        }

        robotMeshes.push({
          mesh: null,
          robot,
          urdfRobot,
          collisionUrdfRobot,
          colorHex: robotColor,
          playbackConfig: null,
          playbackVisible: true,
          materialStylePassesRemaining: 120,
        });
        scene.add(urdfRobot);
        if (collisionUrdfRobot) scene.add(collisionUrdfRobot);
        applyRobotGeometryModeToScene();
      } catch (err) {
        console.warn("URDF load failed for", urdfPaths, err);
        const ph = buildPlaceholderRobot(robot, robotColor);
        robotMeshes.push(ph);
        scene.add(ph.mesh);
      }
    } else {
      const ph = buildPlaceholderRobot(robot, robotColor);
      robotMeshes.push(ph);
      scene.add(ph.mesh);
    }
  }

  setTimestep(0);
  updateUI();
  applySceneDisplayMode();
  if (showCrossSection2D) applyCrossSectionCamera();
  else applyResultCamera(data);
}

/**
 * Load JSON from file input.
 */
function loadFromFile(file) {
  const reader = new FileReader();
  reader.onload = async (e) => {
    const data = parseResult(e.target.result);
    if (data) {
      await loadResult(data);
    } else {
      if (timestepEl) timestepEl.textContent = "Timestep 0 / 0";
      alert("Invalid or unsupported JSON format.");
    }
  };
  reader.readAsText(file);
}

/**
 * Load JSON from URL. Path is relative to server root.
 */
async function loadFromUrl(path) {
  try {
    if (timestepEl) timestepEl.textContent = "Loading...";
    const url = path.startsWith("http")
      ? path
      : new URL(path.replace(/^\//, ""), getAssetBaseUrl()).href;
    const res = await fetch(url);
    const text = await res.text();
    if (!res.ok) {
      throw new Error(`HTTP ${res.status}: ${path}`);
    }
    const data = parseResult(text);
    if (data) await loadResult(data);
    else {
      if (timestepEl) timestepEl.textContent = "Timestep 0 / 0";
      alert("Invalid or unsupported JSON format.");
    }
  } catch (err) {
    if (timestepEl) timestepEl.textContent = "Timestep 0 / 0";
    alert("Failed to load: " + err.message);
  }
}

/**
 * Animation loop.
 */
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  syncCameraInputsFromOrbit();
  refreshRobotSolidColors();
  renderer.render(scene, camera);
}

/**
 * Main init.
 */
function init() {
  initScene();
  initCameraPanel();
  initArcStageBox();
  syncCameraInputsFromOrbit();

  timestepEl = document.getElementById("timestep");
  playPauseBtn = document.getElementById("play-pause");
  sliderEl = document.getElementById("timestep-slider");
  playbackModeSelectEl = document.getElementById("playback-mode-select");
  playbackSpeedSelectEl = document.getElementById("playback-speed-select");
  initRobotPaletteSelect();

  document.getElementById("file-input").addEventListener("change", (e) => {
    const f = e.target.files[0];
    if (f) {
      if (timestepEl) timestepEl.textContent = "Loading...";
      loadFromFile(f);
    }
  });

  if (playPauseBtn) playPauseBtn.addEventListener("click", togglePlay);

  if (playbackModeSelectEl) {
    playbackModeSelectEl.addEventListener("change", (event) => {
      setPlaybackMode(event.target.value);
    });
  }

  if (playbackSpeedSelectEl) {
    playbackSpeedMultiplier =
      Number(playbackSpeedSelectEl.value) || playbackSpeedMultiplier;
    playbackSpeedSelectEl.addEventListener("change", (event) => {
      const nextSpeed = Number(event.target.value);
      if (!Number.isFinite(nextSpeed) || nextSpeed <= 0) return;
      playbackSpeedMultiplier = nextSpeed;
      updateUI();
      restartPlaybackTimer();
    });
  }

  if (sliderEl) {
    sliderEl.addEventListener("input", (e) => setTimestep(parseInt(e.target.value, 10)));
  }

  const lightingBtn = document.getElementById("lighting-toggle");
  if (lightingBtn) {
    applyLightingMode();
    syncLightingToggleButton();
    lightingBtn.addEventListener("click", toggleLightingMode);
  }

  const geometryBtn = document.getElementById("geometry-toggle");
  if (geometryBtn) {
    syncGeometryToggleButton();
    geometryBtn.addEventListener("click", toggleRobotGeometryMode);
  }

  const crossSectionBtn = document.getElementById("cross-section-toggle");
  if (crossSectionBtn) {
    syncCrossSectionToggleButton();
    crossSectionBtn.addEventListener("click", toggleCrossSection2DMode);
  }

  // URL param ?file=<path-to-result-json>
  const params = new URLSearchParams(window.location.search);
  const fileParam = params.get("file");
  if (fileParam) {
    loadFromUrl(fileParam);
  }

  updateUI();
  animate();
}

init();

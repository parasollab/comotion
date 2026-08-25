/**
 * URDF loader with OBJ mesh support for Panda and similar robots.
 * Wraps urdf-loader and adds OBJLoader for .obj files.
 */

import * as THREE from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader from "https://unpkg.com/urdf-loader@0.12.6/src/URDFLoader.js";

/**
 * Resolve mesh path from urdf-loader (may be absolute URL or path-relative to URDF dir).
 * @param {string} path - Path passed by URDFLoader
 * @param {string} assetBase - Repo root base URL (trailing slash optional)
 * @param {string} meshBase - URDF directory base URL (trailing slash optional)
 */
function resolveMeshUrl(path, assetBase, meshBase = assetBase) {
  if (/^https?:\/\//i.test(path)) {
    return path;
  }
  const sanitizedPath = path.replace(/^package:\/\//i, "");
  const baseInput = meshBase || assetBase;
  const base = baseInput.endsWith("/") ? baseInput : `${baseInput}/`;
  return new URL(sanitizedPath, base).href;
}

function shouldEmitMeshDebug() {
  try {
    return new URLSearchParams(window.location.search).has("viewerMeshDebug");
  } catch (_) {
    return false;
  }
}

function emitMeshDebug(message) {
  console.info(message);
  if (!shouldEmitMeshDebug()) return;
  try {
    const url = new URL("/__viewer_debug", window.location.origin);
    url.searchParams.set("m", message.slice(0, 900));
    fetch(url.href, { method: "GET", cache: "no-store" }).catch(() => {});
  } catch (_) {
    /* Debug beacons are best-effort only. */
  }
}

function summarizeObjText(text) {
  const counts = { object: 0, meshFace: 0, line: 0, usemtl: 0 };
  for (const line of text.split(/\r?\n/)) {
    if (line.startsWith("o ")) counts.object++;
    else if (line.startsWith("f ")) counts.meshFace++;
    else if (line.startsWith("l ")) counts.line++;
    else if (line.startsWith("usemtl ")) counts.usemtl++;
  }
  return counts;
}

function stripObjLineRecords(text) {
  let stripped = 0;
  const output = text
    .split(/\r?\n/)
    .filter((line) => {
      if (/^l\s+/.test(line)) {
        stripped++;
        return false;
      }
      return true;
    })
    .join("\n");
  return { text: output, stripped };
}

function countLoadedDrawables(root) {
  const counts = { mesh: 0, line: 0, triangles: 0 };
  root?.traverse?.((obj) => {
    if (obj.isMesh) {
      counts.mesh++;
      const indexCount = obj.geometry?.index?.count;
      const positionCount = obj.geometry?.attributes?.position?.count;
      counts.triangles += Math.floor((indexCount ?? positionCount ?? 0) / 3);
      return;
    }
    if (obj.isLine || obj.isLineSegments || obj.isLineLoop) {
      counts.line++;
    }
  });
  return counts;
}

function createFallbackSurfaceMaterial(colorHex) {
  return new THREE.MeshLambertMaterial({
    color: colorHex,
    transparent: false,
    opacity: 1,
    depthWrite: true,
    side: THREE.DoubleSide,
  });
}

function createFallbackLineMaterial(colorHex) {
  return new THREE.LineBasicMaterial({
    color: colorHex,
    transparent: false,
    opacity: 1,
    depthWrite: true,
  });
}

function replaceMaterial(material, replacementFactory) {
  if (Array.isArray(material)) {
    material.forEach((m) => m?.dispose?.());
    return material.map(() => replacementFactory());
  }
  material?.dispose?.();
  return replacementFactory();
}

function applyOpaqueFallbackMaterial(root, fallbackColorHex) {
  root.traverse?.((obj) => {
    if (obj.isMesh) {
      obj.material = replaceMaterial(
        obj.material,
        () => createFallbackSurfaceMaterial(fallbackColorHex)
      );
      return;
    }
    if (obj.isLine || obj.isLineSegments || obj.isLineLoop) {
      obj.material = replaceMaterial(
        obj.material,
        () => createFallbackLineMaterial(fallbackColorHex)
      );
    }
  });
}

/**
 * Create a URDFLoader with OBJ and STL support.
 * Returns the URDFLoader instance with {@link getPendingMeshLoads} attached (avoids
 * `const { loader } = createURDFLoader()` breaking when callers expect the loader itself).
 * @param {string} assetBase - Base URL for resolving relative paths (e.g. repo root)
 * @param {number} fallbackColorHex - Opaque fallback color for OBJ/STL materials
 * @returns {URDFLoader & { getPendingMeshLoads: () => number }}
 */
export function createURDFLoader(assetBase, fallbackColorHex = 0x808080) {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.parseVisual = true;
  loader.parseCollision = true;
  let pendingMeshLoads = 0;
  let meshBaseUrl = assetBase;
  let meshAssetRevision = "";

  loader.setMeshBaseUrl = (baseUrl) => {
    meshBaseUrl = baseUrl;
    loader.workingPath = baseUrl;
    // URDFLoader otherwise expands package://meshes/... to /meshes/...
    // because its default package root is the empty string. Our packaged
    // URDF resources keep their package directories beside the URDF.
    loader.packages = baseUrl.replace(/\/$/, "");
  };

  loader.setMeshAssetRevision = (revision) => {
    meshAssetRevision = revision || "";
  };

  loader.loadMeshCb = function (path, mgr, onComplete) {
    pendingMeshLoads++;
    const wrapped = (obj, err) => {
      try {
        onComplete(obj, err);
      } finally {
        pendingMeshLoads--;
      }
    };

    let fullPath = resolveMeshUrl(path, assetBase, meshBaseUrl);
    if (meshAssetRevision) {
      const revisedUrl = new URL(fullPath);
      revisedUrl.searchParams.set("assetRevision", meshAssetRevision);
      fullPath = revisedUrl.href;
    }
    const pathForExt = fullPath.split("?")[0];
    if (/\.obj$/i.test(pathForExt)) {
      const objLoader = new OBJLoader(mgr);
      fetch(fullPath)
        .then((res) => {
          if (!res.ok) throw new Error(`HTTP ${res.status}: ${fullPath}`);
          return res.text();
        })
        .then((objText) => {
          const before = summarizeObjText(objText);
          const { text: meshOnlyObjText, stripped } = stripObjLineRecords(objText);
          const group = objLoader.parse(meshOnlyObjText);
          applyOpaqueFallbackMaterial(group, fallbackColorHex);
          const after = countLoadedDrawables(group);
          if (stripped > 0 || /\/link5\.obj$/i.test(pathForExt)) {
            emitMeshDebug(
              `[viewer mesh] ${path}: faces=${before.meshFace} lineRecords=${before.line} ` +
                `strippedLines=${stripped} loadedMeshes=${after.mesh} ` +
                `loadedLines=${after.line} loadedTriangles=${after.triangles}`
            );
          }
          if (group.isMesh) {
            wrapped(group);
          } else if (group.children.length === 1 && group.children[0].isMesh) {
            wrapped(group.children[0]);
          } else {
            wrapped(group);
          }
        })
        .catch((err) => wrapped(null, err));
    } else if (/\.stl$/i.test(pathForExt)) {
      const stlLoader = new STLLoader(mgr);
      stlLoader.load(
        fullPath,
        (geom) => {
          const mesh = new THREE.Mesh(
            geom,
            createFallbackSurfaceMaterial(fallbackColorHex)
          );
          wrapped(mesh);
        },
        undefined,
        (err) => wrapped(null, err)
      );
    } else {
      console.warn("URDFLoader: No loader for", path);
      wrapped(null, new Error("Unsupported mesh format"));
    }
  };

  loader.getPendingMeshLoads = () => pendingMeshLoads;
  return loader;
}

/**
 * Load URDF from URL and return a promise.
 * @param {URDFLoader} loader
 * @param {string} urdfUrl - Full URL to URDF file
 * @param {{parseVisual?: boolean, parseCollision?: boolean}} options
 * @returns {Promise<URDFRobot>}
 */
export function loadURDFAsync(loader, urdfUrl, options = {}) {
  return new Promise((resolve, reject) => {
    loader.parseVisual = options.parseVisual ?? true;
    loader.parseCollision = options.parseCollision ?? true;
    const urdfBase = new URL("./", urdfUrl).href;
    const assetRevision = new URL(urdfUrl).searchParams.get("assetRevision") || "";
    loader.workingPath = urdfBase;
    if (typeof loader.setMeshBaseUrl === "function") {
      loader.setMeshBaseUrl(urdfBase);
    }
    if (typeof loader.setMeshAssetRevision === "function") {
      loader.setMeshAssetRevision(assetRevision);
    }
    loader.load(urdfUrl, resolve, undefined, reject);
  });
}

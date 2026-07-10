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

/**
 * Create a URDFLoader with OBJ and STL support.
 * Returns the URDFLoader instance with {@link getPendingMeshLoads} attached (avoids
 * `const { loader } = createURDFLoader()` breaking when callers expect the loader itself).
 * @param {string} assetBase - Base URL for resolving relative paths (e.g. repo root)
 * @returns {URDFLoader & { getPendingMeshLoads: () => number }}
 */
export function createURDFLoader(assetBase) {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.parseVisual = true;
  loader.parseCollision = true;
  let pendingMeshLoads = 0;
  let meshBaseUrl = assetBase;

  loader.setMeshBaseUrl = (baseUrl) => {
    meshBaseUrl = baseUrl;
    loader.workingPath = baseUrl;
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

    const fullPath = resolveMeshUrl(path, assetBase, meshBaseUrl);
    const pathForExt = fullPath.split("?")[0];
    if (/\.obj$/i.test(pathForExt)) {
      const objLoader = new OBJLoader(mgr);
      objLoader.load(
        fullPath,
        (group) => {
          if (group.isMesh) {
            wrapped(group);
          } else if (group.children.length === 1 && group.children[0].isMesh) {
            wrapped(group.children[0]);
          } else {
            wrapped(group);
          }
        },
        undefined,
        (err) => wrapped(null, err)
      );
    } else if (/\.stl$/i.test(pathForExt)) {
      const stlLoader = new STLLoader(mgr);
      stlLoader.load(
        fullPath,
        (geom) => {
          const mesh = new THREE.Mesh(
            geom,
            new THREE.MeshLambertMaterial({ color: 0x808080, side: THREE.DoubleSide })
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
 * @returns {Promise<URDFRobot>}
 */
export function loadURDFAsync(loader, urdfUrl) {
  return new Promise((resolve, reject) => {
    loader.parseVisual = true;
    loader.parseCollision = true;
    const urdfBase = new URL("./", urdfUrl).href;
    loader.workingPath = urdfBase;
    if (typeof loader.setMeshBaseUrl === "function") {
      loader.setMeshBaseUrl(urdfBase);
    }
    loader.load(urdfUrl, resolve, undefined, reject);
  });
}

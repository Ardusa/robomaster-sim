/**
 * Live URDF reconstruction via three.js + urdf-loader.
 * Meshes resolve package://robomaster_description/meshes/* → /meshes/*
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

const JOINT_NAMES = [
  "front_left_wheel_joint",
  "front_right_wheel_joint",
  "rear_left_wheel_joint",
  "rear_right_wheel_joint",
  "arm_1_joint",
  "arm_2_joint",
  "gripper_m_joint",
  "gripper_r_joint",
];

export function createRobot3d(container, { loadingEl, resetBtn } = {}) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d11);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  camera.position.set(0.7, 0.55, 0.7);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.12, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.25;
  controls.maxDistance = 4;

  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const key = new THREE.DirectionalLight(0xffffff, 1.1);
  key.position.set(1.5, 2.5, 1.2);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x88aacc, 0.35);
  fill.position.set(-1.2, 0.8, -1);
  scene.add(fill);

  const grid = new THREE.GridHelper(2, 20, 0x2c3644, 0x1a222c);
  grid.position.y = 0;
  scene.add(grid);

  // ROS: z-up. three.js: y-up. Wrap the robot so joint math stays in ROS frame.
  const rosRoot = new THREE.Group();
  rosRoot.rotation.x = -Math.PI / 2;
  scene.add(rosRoot);

  let robot = null;
  let disposed = false;
  const defaultCam = camera.position.clone();
  const defaultTarget = controls.target.clone();

  function setLoading(text, show = true) {
    if (!loadingEl) return;
    loadingEl.hidden = !show;
    if (text) loadingEl.textContent = text;
  }

  function resize() {
    const w = container.clientWidth || 1;
    const h = container.clientHeight || 1;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }

  const ro = new ResizeObserver(resize);
  ro.observe(container);
  resize();

  function animate() {
    if (disposed) return;
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  if (resetBtn) {
    resetBtn.addEventListener("click", () => {
      camera.position.copy(defaultCam);
      controls.target.copy(defaultTarget);
      controls.update();
    });
  }

  async function load() {
    setLoading("Fetching robot description…", true);
    let urdfText = "";
    for (let attempt = 0; attempt < 40; attempt++) {
      const res = await fetch("/api/robot_description");
      if (res.ok) {
        urdfText = await res.text();
        break;
      }
      await new Promise((r) => setTimeout(r, 500));
    }
    if (!urdfText) {
      setLoading("robot_description unavailable", true);
      return;
    }

    setLoading("Loading CAD meshes…", true);
    const manager = new THREE.LoadingManager();
    manager.onLoad = () => setLoading("", false);
    manager.onError = (url) => {
      console.warn("mesh load failed:", url);
    };

    const loader = new URDFLoader(manager);
    // resolvePath does packages[pkg] + '/' + relPath, so '' → /meshes/foo.dae
    loader.packages = {
      robomaster_description: "",
    };
    loader.workingPath = "/";
    loader.parseCollision = false;

    try {
      const robotObj = loader.parse(urdfText);
      robotObj.rotation.set(0, 0, 0);
      // Ignore collisions for a cleaner viz; keep visuals only.
      if (typeof robotObj.setJointValue === "function") {
        /* URDFRobot */
      }
      robotObj.traverse((c) => {
        if (c.isMesh) {
          c.castShadow = false;
          c.receiveShadow = false;
        }
        // Hide collision-only geom when tagged.
        if (c.isURDFCollider) {
          c.visible = false;
        }
      });
      rosRoot.clear();
      rosRoot.add(robotObj);
      robot = robotObj;
      setLoading("", false);
    } catch (err) {
      console.error(err);
      setLoading(`Failed to parse URDF: ${err.message || err}`, true);
    }
  }

  function setJoint(name, value) {
    const j = robot.joints?.[name];
    if (j && typeof robot.setJointValue === "function") {
      robot.setJointValue(name, value);
    } else if (j && typeof j.setJointValue === "function") {
      j.setJointValue(value);
    }
  }

  function onState(state) {
    if (!robot || !state) return;
    const joints = state.joints || {};
    for (const name of JOINT_NAMES) {
      if (joints[name] == null) continue;
      setJoint(name, joints[name]);
    }
    // Opposite URDF axes, same command value; synthesize if states omit it.
    if (joints.gripper_m_joint != null && joints.gripper_r_joint == null) {
      setJoint("gripper_r_joint", joints.gripper_m_joint);
    }

    // Keep the robot pinned at the widget origin. Odometry describes travel
    // through the world, but this operator view follows the chassis and only
    // animates its joints.
    robot.position.set(0, 0, 0);
    robot.rotation.set(0, 0, 0);
  }

  function dispose() {
    disposed = true;
    ro.disconnect();
    renderer.dispose();
    if (renderer.domElement.parentNode === container) {
      container.removeChild(renderer.domElement);
    }
  }

  return { load, onState, dispose, resize };
}

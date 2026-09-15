// src/viewer/RobotScene.tsx
import { useMemo, useEffect } from "react";
import * as THREE from "three";

// src/viewer/coordinateAdapter.ts
import { Quaternion, Vector3 } from "three";
var SCENE_ROTATION_X = -Math.PI / 2;
function quaternionToThree(q) {
  return new Quaternion(q[0], q[1], q[2], q[3]);
}
function transformToThree(t) {
  return {
    position: new Vector3(t.position[0], t.position[1], t.position[2]),
    quaternion: quaternionToThree(t.orientation)
  };
}
function axisToThree(axis) {
  return new Vector3(axis[0], axis[1], axis[2]).normalize();
}

// src/viewer/RobotScene.tsx
import { jsx, jsxs } from "react/jsx-runtime";
function makeGeometry(g) {
  const [a = 0, b = 0, c = 0] = g.size;
  switch (g.type) {
    case "box":
      return new THREE.BoxGeometry(a * 2, b * 2, c * 2);
    case "sphere":
      return new THREE.SphereGeometry(a, 24, 16);
    case "cylinder":
      return new THREE.CylinderGeometry(a, a, b * 2, 24);
    case "capsule":
      return new THREE.CapsuleGeometry(a, Math.max(0, b * 2 - a * 2), 8, 16);
    case "ellipsoid":
      return new THREE.SphereGeometry(1, 24, 16);
    case "plane":
      return new THREE.PlaneGeometry(a * 2, b * 2);
    case "mesh":
      return new THREE.BoxGeometry(0.01, 0.01, 0.01);
    default:
      return null;
  }
}
function GeomMesh({ g }) {
  const geometry = useMemo(() => makeGeometry(g), [g]);
  const { position, quaternion } = useMemo(
    () => transformToThree(g.transform),
    [g.transform]
  );
  const color = useMemo(
    () => new THREE.Color(g.rgba[0] ?? 0.5, g.rgba[1] ?? 0.5, g.rgba[2] ?? 0.5),
    [g.rgba]
  );
  const posArr = [position.x, position.y, position.z];
  const quatArr = [
    quaternion.x,
    quaternion.y,
    quaternion.z,
    quaternion.w
  ];
  useEffect(() => () => geometry?.dispose(), [geometry]);
  if (!geometry) {
    return /* @__PURE__ */ jsxs("mesh", { name: "geom-unknown", position: posArr, children: [
      /* @__PURE__ */ jsx("boxGeometry", { args: [0.02, 0.02, 0.02] }),
      /* @__PURE__ */ jsx("meshBasicMaterial", { color: "#ff00ff", wireframe: true })
    ] });
  }
  return /* @__PURE__ */ jsx(
    "mesh",
    {
      name: `geom:${g.type}`,
      geometry,
      position: posArr,
      quaternion: quatArr,
      children: /* @__PURE__ */ jsx("meshStandardMaterial", { color, metalness: 0.2, roughness: 0.6 })
    }
  );
}
function AxisIndicator({
  axis,
  length = 0.05
}) {
  const { dir, quaternion } = useMemo(() => {
    const d = axisToThree(axis);
    const q = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      d
    );
    return { dir: d, quaternion: q };
  }, [axis]);
  const color = useMemo(() => {
    const [x, y, z] = [Math.abs(dir.x), Math.abs(dir.y), Math.abs(dir.z)];
    if (z >= x && z >= y) return new THREE.Color(2254591);
    if (y >= x) return new THREE.Color(2276164);
    return new THREE.Color(16724787);
  }, [dir]);
  return (
    // ★ 给轴指示器一个可辨识的 name。
    //   没有它时，"这个 joint 下有几个子 group"就无法区分
    //   "轴指示器" 与 "子 link/joint" —— 检查只能靠数个数，那是脆弱的判据
    //   （本文件曾经就因此把一个正确的实现判成失败）。
    /* @__PURE__ */ jsxs("group", { name: `axis:${axis.join(",")}`, quaternion, children: [
      /* @__PURE__ */ jsxs("mesh", { position: [0, 0, length / 2], children: [
        /* @__PURE__ */ jsx("cylinderGeometry", { args: [18e-4, 18e-4, length, 8] }),
        /* @__PURE__ */ jsx("meshBasicMaterial", { color })
      ] }),
      /* @__PURE__ */ jsxs("mesh", { position: [0, 0, length], children: [
        /* @__PURE__ */ jsx("coneGeometry", { args: [45e-4, 0.012, 12] }),
        /* @__PURE__ */ jsx("meshBasicMaterial", { color })
      ] })
    ] })
  );
}
function NodeView({
  vm,
  node,
  eeByLink,
  showAxes
}) {
  const { posArr, quatArr } = useMemo(() => {
    if (node.kind === "link" && node.parentKey !== null) {
      return {
        posArr: [0, 0, 0],
        quatArr: [0, 0, 0, 1]
      };
    }
    const { position, quaternion } = transformToThree(node.localTransform);
    return {
      posArr: [position.x, position.y, position.z],
      quatArr: [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    };
  }, [node]);
  const childNodes = node.childKeys.map((k) => vm.nodes.find((n) => n.key === k)).filter((n) => n !== void 0);
  const eeIdsHere = node.kind === "link" ? eeByLink.get(node.id) ?? [] : [];
  return /* @__PURE__ */ jsxs(
    "group",
    {
      name: node.key,
      position: posArr,
      quaternion: quatArr,
      children: [
        node.kind === "link" && node.geometries.map((g, i) => /* @__PURE__ */ jsx(GeomMesh, { g }, `${node.key}-g${i}`)),
        eeIdsHere.map((eeId) => /* @__PURE__ */ jsx(EndEffectorMarker, { vm, eeId }, `ee-${eeId}`)),
        showAxes && node.kind === "joint" && node.axis !== null && node.isMovable && /* @__PURE__ */ jsx(AxisIndicator, { axis: node.axis }),
        childNodes.map((c) => /* @__PURE__ */ jsx(NodeView, { vm, node: c, eeByLink, showAxes }, c.key))
      ]
    }
  );
}
function FrameAxes({
  scale = 0.1,
  origin
}) {
  return /* @__PURE__ */ jsx("axesHelper", { args: [scale], position: origin ?? [0, 0, 0] });
}
function EndEffectorMarker({
  vm,
  eeId
}) {
  const ee = vm.endEffectors.find((e) => e.id === eeId);
  const posArr = useMemo(() => {
    if (!ee) return [0, 0, 0];
    const { position } = transformToThree(ee.transform);
    return [position.x, position.y, position.z];
  }, [ee]);
  const quatArr = useMemo(() => {
    if (!ee) return [0, 0, 0, 1];
    const { quaternion } = transformToThree(ee.transform);
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
  }, [ee]);
  if (!ee) return null;
  return /* @__PURE__ */ jsxs("group", { name: `ee:${ee.id}`, position: posArr, quaternion: quatArr, children: [
    /* @__PURE__ */ jsxs("mesh", { name: "ee-sphere", children: [
      /* @__PURE__ */ jsx("sphereGeometry", { args: [8e-3, 16, 12] }),
      /* @__PURE__ */ jsx("meshBasicMaterial", { color: 16755200, transparent: true, opacity: 0.85 })
    ] }),
    /* @__PURE__ */ jsx("axesHelper", { args: [0.04] })
  ] });
}
function RobotScene({
  vm,
  showAxes = true,
  showEndEffector = true,
  showWorldFrame = true
}) {
  const roots = useMemo(
    () => vm.nodes.filter((n) => n.parentKey === null),
    [vm]
  );
  const eeByLink = useMemo(() => {
    const m = /* @__PURE__ */ new Map();
    for (const ee of vm.endEffectors) {
      const arr = m.get(ee.linkId) ?? [];
      arr.push(ee.id);
      m.set(ee.linkId, arr);
    }
    return m;
  }, [vm]);
  return /* @__PURE__ */ jsxs("group", { name: "robot-root", children: [
    roots.map((r) => /* @__PURE__ */ jsx(
      NodeView,
      {
        vm,
        node: r,
        eeByLink: showEndEffector ? eeByLink : /* @__PURE__ */ new Map(),
        showAxes
      },
      r.key
    )),
    showWorldFrame && /* @__PURE__ */ jsx(FrameAxes, { scale: 0.12 })
  ] });
}
export {
  AxisIndicator,
  EndEffectorMarker,
  FrameAxes,
  RobotScene,
  makeGeometry
};

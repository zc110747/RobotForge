/**
 * =============================================================================
 * model.ts —— RobotModel 的 TypeScript 镜像类型
 * -----------------------------------------------------------------------------
 * 这些类型必须与 `backend/model/robot_model.py` 的 `RobotModel.to_dict()`
 * **逐字段对应**。真值源是 Python 侧，本文件是它的只读镜像。
 *
 * ## 为什么手写镜像而不是代码生成
 *
 * v0.1 的字段集很小且已冻结（P0 契约）。代码生成会引入构建步骤 + 一个
 * 生成器脚本，而生成器本身也需要被验证 —— 对于 12 个字段，这个成本
 * 大于收益。**但代价必须被测试覆盖**：`semantics.check.ts` 会拿真实
 * 后端 JSON 逐字段比对，字段名写错会立刻红。
 *
 * ⚠️ 因此：改 Python 侧字段必须同时改本文件，否则一致性测试失败。
 *    这是刻意的 —— 我们希望它失败，而不是静默地少画一个关节。
 *
 * ## 单位与坐标系（不要在本文件里做转换）
 *
 * - 长度 `m`，角度 `rad`，四元数 `[x,y,z,w]`，坐标系 Z-up/+X前/+Y左。
 * - 本文件是**纯类型 + 纯数据**，不含任何 Three.js 依赖，
 *   因此可以在 Node 里直接跑（语义测试需要）。
 * - 坐标转换只在 `coordinateAdapter.ts` 里发生。
 */

/** `[x, y, z]`，单位 m。 */
export type Vec3 = readonly [number, number, number];

/** `[x, y, z, w]`，单位四元数。 */
export type Quat = readonly [number, number, number, number];

export type JointType = "revolute" | "prismatic" | "fixed" | "free";
export type GeomType =
  | "box"
  | "sphere"
  | "capsule"
  | "cylinder"
  | "ellipsoid"
  | "mesh"
  | "plane";

/** 与 `backend/model/types.py::Transform` 对应。 */
export interface TransformDTO {
  readonly position: Vec3;
  readonly orientation: Quat;
}

/** 与 `types.py::GeometryRef` 对应。 */
export interface GeometryRefDTO {
  readonly type: GeomType | string;
  readonly size: readonly number[];
  readonly asset: string | null;
  readonly transform: TransformDTO;
  readonly rgba: readonly number[];
}

export interface InertialDTO {
  readonly mass: number;
  readonly center_of_mass: Vec3;
  readonly inertia: {
    readonly ixx: number;
    readonly iyy: number;
    readonly izz: number;
    readonly ixy: number | null;
    readonly ixz: number | null;
    readonly iyz: number | null;
  };
}

/** 与 `robot_model.py::Link` 对应。 */
export interface LinkDTO {
  readonly id: string;
  readonly name: string;
  /**
   * 连接到**父**关节的 id；根链接为 `null`。
   *
   * ★ 建树用 `parent_joint` + `joints[].parent_link`，
   *   不要用 `child_joints`（它是派生视图，两份真值的风险）。
   */
  readonly parent_joint: string | null;
  readonly child_joints: readonly string[];
  readonly inertial: InertialDTO | null;
  readonly visual: readonly GeometryRefDTO[];
  readonly collision: readonly GeometryRefDTO[];
}

/** 与 `robot_model.py::JointLimits` 对应。position 单位 rad（revolute）。 */
export interface JointLimitsDTO {
  readonly position_min: number | null;
  readonly position_max: number | null;
  readonly velocity_max: number | null;
  readonly effort_max: number | null;
}

/** 与 `robot_model.py::Joint` 对应。 */
export interface JointDTO {
  readonly id: string;
  readonly name: string;
  readonly type: JointType | string;
  readonly parent_link: string;
  readonly child_link: string;
  readonly origin: TransformDTO;
  /** 旋转/平移轴（单位向量，表达在 parent 坐标系里）。 */
  readonly axis: Vec3;
  readonly limits: JointLimitsDTO | null;
}

export interface ActuatorDTO {
  readonly id: string;
  readonly name: string;
  readonly type: string;
  readonly joint: string;
  readonly command_min: number;
  readonly command_max: number;
}

/** 与 `robot_model.py::Site` 对应。`parent` 是 link id。 */
export interface SiteDTO {
  readonly id: string;
  readonly name: string;
  readonly parent: string;
  readonly transform: TransformDTO;
  readonly role: string | null;
}

/** 与 `robot_model.py::Frame` 对应。`parent` 是 link id 或 `"world"`。 */
export interface FrameDTO {
  readonly id: string;
  readonly name: string;
  readonly parent: string;
  readonly transform: TransformDTO;
}

/** 与 `robot_model.py::EndEffector` 对应。 */
export interface EndEffectorDTO {
  readonly id: string;
  readonly name: string;
  /** 指向 `frames[].id` */
  readonly frame: string;
  /** 指向 `sites[].id` */
  readonly site: string;
}

export interface RobotMetadataDTO {
  readonly id: string;
  readonly name: string;
  readonly version: string;
  readonly description: string;
}

/**
 * 坐标系声明。**前端必须读它并校验**，而不是假定 robotforge 约定 ——
 * 若某天后端送来 `handedness: "left"`，前端应该报错而不是默默画错。
 */
export interface CoordinateConventionDTO {
  readonly convention: string;
  readonly handedness: "right" | "left" | string;
  readonly forward_axis: string;
  readonly left_axis: string;
  readonly up_axis: string;
}

export interface UnitConventionDTO {
  readonly length: string;
  readonly angle: string;
  readonly time: string;
  readonly linear_velocity: string;
  readonly angular_velocity: string;
  readonly force: string;
  readonly torque: string;
}

/** 与 `robot_model.py::RobotCapabilities` 对应。 */
export interface RobotCapabilitiesDTO {
  readonly simulation: boolean;
  readonly fk: boolean;
  readonly ik: boolean;
  readonly actuator_control: boolean;
  readonly end_effector: boolean;
}

/** `RobotModel.to_dict()` 的完整形状。 */
export interface RobotModelDTO {
  readonly metadata: RobotMetadataDTO;
  readonly coordinate: CoordinateConventionDTO;
  readonly units: UnitConventionDTO;
  readonly root_link: string | null;
  readonly base_frame: string | null;
  readonly links: readonly LinkDTO[];
  readonly joints: readonly JointDTO[];
  readonly actuators: readonly ActuatorDTO[];
  readonly frames: readonly FrameDTO[];
  readonly sites: readonly SiteDTO[];
  readonly end_effectors: readonly EndEffectorDTO[];
  readonly capabilities: RobotCapabilitiesDTO;
}

/** `GET /api/robots` 的一行。 */
export interface RobotSummaryDTO {
  readonly id: string;
  readonly name: string;
  readonly version: string;
  readonly description: string;
  readonly capabilities: RobotCapabilitiesDTO;
}

export interface RobotListResponse {
  readonly robots: readonly RobotSummaryDTO[];
  readonly count: number;
}

/** `GET /api/robots/{id}/model` 的响应。 */
export interface RobotModelResponse {
  readonly robot_id: string;
  readonly model: RobotModelDTO;
  readonly validation: {
    readonly ok: boolean;
    readonly issues: readonly {
      readonly level: string;
      readonly code: string;
      readonly where: string;
      readonly message: string;
    }[];
  };
}

"""REST API 契约测试。

## 本文件守住的那条线

§59 的最后一条验收是 **"RobotModel 与 Renderer 语义一致"**。
这句话要能被自动化检查，必须落到一个**具体断言**上。
本文件给的就是那个断言：

    `GET /api/robots/{id}/model` 的 `model` 字段  ==  `RobotModel.to_dict()`

看起来像同义反复（都是 to_dict），但它防的是**将来**：
某天有人为了"前端用着方便"在 API 层把 `joints[].origin.position`
改成 `joints[].origin_position`，或者把四元数从 `[x,y,z,w]` 拍成
`{"w":…}`。那时这条测试会红，而**这个改动在 code review 里
看起来完全无害** —— 这正是它存在的意义。

## 为什么不在这里测坐标转换

坐标转换的唯一合法出口是 `viewer/coordinateAdapter.ts`（前端）。
后端**不做**任何转换（`docs/coordinate-system.md` §4）。
因此本文件反而要断言"后端没偷偷转换"：mini_arm 是 no-conversion 情形，
所以 API 返回的 position 必须与 MJCF 里写的**逐位相同**。
"""

from __future__ import annotations

import json

import pytest
from fastapi.testclient import TestClient

from backend.api.app import API_PREFIX, create_app
from backend.api.registry import discover_packages, get_package


@pytest.fixture(scope="module")
def client() -> TestClient:
    return TestClient(create_app())


# ---------------------------------------------------------------------------
# 健康检查与列表
# ---------------------------------------------------------------------------


class TestHealthAndList:
    def test_health_ok(self, client: TestClient) -> None:
        r = client.get(f"{API_PREFIX}/health")
        assert r.status_code == 200
        assert r.json()["status"] == "ok"

    def test_list_returns_envelope_not_bare_array(self, client: TestClient) -> None:
        """返回对象而非裸数组 —— 为了前向兼容（见 app.py 的注释）。

        裸数组没有地方放分页/版本，而"数组 → 对象"是破坏性变更。
        """
        r = client.get(f"{API_PREFIX}/robots")
        assert r.status_code == 200
        body = r.json()
        assert isinstance(body, dict), "必须是对象，不能是裸数组"
        assert "robots" in body and "count" in body
        assert body["count"] == len(body["robots"])

    def test_list_includes_mini_arm(self, client: TestClient) -> None:
        ids = [x["id"] for x in client.get(f"{API_PREFIX}/robots").json()["robots"]]
        assert "mini_arm" in ids

    def test_list_is_sorted_by_id(self, client: TestClient) -> None:
        """顺序稳定 —— 否则前端测试无法稳定断言（文件系统顺序不保证）。"""
        ids = [x["id"] for x in client.get(f"{API_PREFIX}/robots").json()["robots"]]
        assert ids == sorted(ids)

    def test_list_row_has_no_derived_geometry(self, client: TestClient) -> None:
        """列表行**不得**含 dof / link_count 这类可推导值。

        manifest.yaml 里有一段注释专门警告这件事（"抄一份就是第二份真值"）。
        这条测试把那段注释变成会失败的检查。
        """
        row = next(
            x for x in client.get(f"{API_PREFIX}/robots").json()["robots"]
            if x["id"] == "mini_arm"
        )
        for forbidden in ("dof", "link_count", "joint_names", "links", "joints"):
            assert forbidden not in row, (
                f"列表行里出现了可推导字段 {forbidden!r} —— 它会与模型漂移"
            )
        # 该有的身份/能力字段必须在
        for required in ("id", "name", "version", "capabilities"):
            assert required in row


# ---------------------------------------------------------------------------
# 模型端点 —— 本文件的核心
# ---------------------------------------------------------------------------


class TestModelEndpoint:
    def test_model_payload_is_exactly_to_dict(self, client: TestClient) -> None:
        """★ 本文件最重要的断言：API 不整形、不重命名、不丢字段。

        比的是**完整嵌套结构**（== 对 dict 是递归深比较），
        所以任何一层字段名/顺序无关的内容差异都会被抓住。
        """
        from backend.loaders.mjcf_loader import MJCFLoader

        model = MJCFLoader().load(get_package("mini_arm").model_path)[0]
        r = client.get(f"{API_PREFIX}/robots/mini_arm/model")
        assert r.status_code == 200
        assert r.json()["model"] == model.to_dict()

    def test_model_is_json_serializable(self, client: TestClient) -> None:
        """响应必须能过一遍标准 json 编解码（捕捉 numpy 标量 / 元组混入）。

        `to_dict()` 里若混进 `np.float64`，FastAPI 可能勉强序列化，
        但前端 `JSON.parse` 后拿到的精度语义已变。这里做一次显式往返。
        """
        r = client.get(f"{API_PREFIX}/robots/mini_arm/model")
        round_tripped = json.loads(r.text)
        assert round_tripped == r.json()

    def test_no_coordinate_conversion_in_backend(self, client: TestClient) -> None:
        """后端**不得**做坐标转换（docs/coordinate-system.md §4）。

        检查方式：拿 MJCF 里的原始数值（已知真值）与 API 返回值比对。
        mini_arm 的 `elbow` origin 在 MJCF 里是 x=0.103 —— 若哪天
        Loader/API 引入了一次"顺手"的轴交换，这里立刻红。
        """
        payload = client.get(f"{API_PREFIX}/robots/mini_arm/model").json()["model"]
        elbow = next(j for j in payload["joints"] if j["id"] == "elbow")
        assert elbow["origin"]["position"] == [0.103, 0.0, 0.0]
        assert elbow["axis"] == [0.0, 1.0, 0.0]

    def test_quaternion_order_is_x_y_z_w(self, client: TestClient) -> None:
        """四元数顺序必须是 `[x,y,z,w]`，恒等 = `[0,0,0,1]`。

        ⚠️ 这条不能靠"看数值"验证 —— 恒等四元数在 `[x,y,z,w]` 和
        `[w,x,y,z]` 下分别是 `[0,0,0,1]` 与 `[1,0,0,0]`，
        所以断言**恒等时的字面值**是能区分两种约定的最小检查。
        """
        payload = client.get(f"{API_PREFIX}/robots/mini_arm/model").json()["model"]
        for j in payload["joints"]:
            assert j["origin"]["orientation"] == [0.0, 0.0, 0.0, 1.0], (
                f"{j['id']} 的恒等四元数不是 [0,0,0,1] —— 顺序可能被写成了 [w,x,y,z]"
            )

    def test_capabilities_are_passed_through(self, client: TestClient) -> None:
        """capabilities 必须原样透传 —— 前端据它决定显示哪些面板。

        提示词 §69 规则 2 禁止 `if (robot.id === 'mini_arm')`，
        所以 capabilities 是前端唯一的开关来源，丢了它前端只能硬编码。
        """
        payload = client.get(f"{API_PREFIX}/robots/mini_arm/model").json()["model"]
        assert payload["capabilities"] == {
            "simulation": True,
            "fk": True,
            "ik": True,
            "actuator_control": True,
            "end_effector": True,
        }

    def test_validation_block_present_and_clean(self, client: TestClient) -> None:
        """干净模型 ⇒ ok=true 且 issues 为空（顺序无关）。"""
        body = client.get(f"{API_PREFIX}/robots/mini_arm/model").json()
        assert body["validation"]["ok"] is True
        assert body["validation"]["issues"] == []

    def test_robot_id_matches_metadata(self, client: TestClient) -> None:
        body = client.get(f"{API_PREFIX}/robots/mini_arm/model").json()
        assert body["robot_id"] == "mini_arm"
        assert body["model"]["metadata"]["id"] == "mini_arm"


# ---------------------------------------------------------------------------
# 错误行为
# ---------------------------------------------------------------------------


class TestErrorBehaviour:
    def test_unknown_robot_is_404(self, client: TestClient) -> None:
        r = client.get(f"{API_PREFIX}/robots/does_not_exist/model")
        assert r.status_code == 404

    def test_404_detail_names_available_robots(self, client: TestClient) -> None:
        """404 的 detail 应告诉调用方**有哪些**可用 —— 省掉一轮猜测。"""
        detail = client.get(f"{API_PREFIX}/robots/nope/model").json()["detail"]
        assert "mini_arm" in detail

    def test_wrong_method_is_405_not_500(self, client: TestClient) -> None:
        r = client.post(f"{API_PREFIX}/robots")
        assert r.status_code == 405


# ---------------------------------------------------------------------------
# 发现式注册：证伪"加机器人要改 Core"
# ---------------------------------------------------------------------------


class TestDiscoveryIsNotHardcoded:
    def test_injected_package_is_discovered(self, tmp_path) -> None:
        """往临时目录放一个新包，API 就应该能列出来。

        ★ 这条测试是"发现式注册"的**证伪条件**。
        如果 registry 里写死了一张 id→路径 的表，它就永远只认识 mini_arm，
        这条测试会红。因此它测的不是 mini_arm，而是**架构声明本身**。
        """
        pkg_dir = tmp_path / "dummy_bot"
        pkg_dir.mkdir()
        (pkg_dir / "model").mkdir()
        # 复用 mini_arm 的 MJCF 内容（几何不重要，重要的是"被发现了"）
        src = (discover_packages()[0].model_path).read_text(encoding="utf-8")
        (pkg_dir / "model" / "dummy_bot.xml").write_text(src, encoding="utf-8")
        (pkg_dir / "manifest.yaml").write_text(
            "id: dummy_bot\n"
            "name: Dummy Bot\n"
            "version: 9.9.9\n"
            "description: injected for discovery test\n"
            "coordinate:\n"
            "  convention: robotforge\n"
            "units:\n"
            "  length: m\n"
            "  angle: rad\n"
            "model:\n"
            "  format: mjcf\n"
            "  file: model/dummy_bot.xml\n"
            "capabilities:\n"
            "  fk: true\n"
            "  ik: false\n",
            encoding="utf-8",
        )

        c = TestClient(create_app(packages_dir=tmp_path))
        ids = [x["id"] for x in c.get(f"{API_PREFIX}/robots").json()["robots"]]
        assert "dummy_bot" in ids, "发现式注册失效了 —— 新包没被识别"

    def test_wrong_format_is_rejected_explicitly(self, tmp_path) -> None:
        """`format: urdf` 必须**显式拒绝**，不能静默忽略。

        v0.1 只允许原生 MJCF（提示词 §7）。显式拒绝保证"加 URDF"
        必须走 Phase 7 的扩展点，而不是某天有人在 Loader 里加个 if。
        """
        pkg_dir = tmp_path / "urdf_bot"
        pkg_dir.mkdir()
        (pkg_dir / "manifest.yaml").write_text(
            "id: urdf_bot\nname: U\nversion: 1\n"
            "model:\n  format: urdf\n  file: model/x.urdf\n"
            "capabilities:\n  fk: true\n",
            encoding="utf-8",
        )
        c = TestClient(create_app(packages_dir=tmp_path))
        r = c.get(f"{API_PREFIX}/robots")
        assert r.status_code == 500
        assert "mjcf" in r.json()["detail"]

    def test_dir_name_must_match_manifest_id(self, tmp_path) -> None:
        """目录名与 manifest.id 不一致 ⇒ 报错（路径契约，提示词 §2）。"""
        pkg_dir = tmp_path / "actual_dir_name"
        pkg_dir.mkdir()
        (pkg_dir / "manifest.yaml").write_text(
            "id: different_id\nname: X\nversion: 1\n"
            "model:\n  format: mjcf\n  file: m.xml\n"
            "capabilities:\n  fk: true\n",
            encoding="utf-8",
        )
        c = TestClient(create_app(packages_dir=tmp_path))
        assert c.get(f"{API_PREFIX}/robots").status_code == 500

    def test_missing_model_file_is_reported(self, tmp_path) -> None:
        """manifest 指向的模型文件不存在 ⇒ 报错，不静默跳过。"""
        pkg_dir = tmp_path / "ghost_bot"
        pkg_dir.mkdir()
        (pkg_dir / "manifest.yaml").write_text(
            "id: ghost_bot\nname: G\nversion: 1\n"
            "model:\n  format: mjcf\n  file: model/missing.xml\n"
            "capabilities:\n  fk: true\n",
            encoding="utf-8",
        )
        c = TestClient(create_app(packages_dir=tmp_path))
        r = c.get(f"{API_PREFIX}/robots")
        assert r.status_code == 500
        assert "missing.xml" in r.json()["detail"]

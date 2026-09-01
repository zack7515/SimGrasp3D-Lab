"""模擬場景設定的資料模型與驗證規則。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


Vector3 = tuple[float, float, float]
Color = tuple[float, float, float]


def _vector3(value: list[float] | tuple[float, ...], field_name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{field_name} 必須包含 3 個數值")
    return (float(value[0]), float(value[1]), float(value[2]))


def _positive(values: tuple[float, ...], field_name: str) -> None:
    if any(value <= 0.0 for value in values):
        raise ValueError(f"{field_name} 的所有數值都必須大於 0")


def _color(value: list[float] | tuple[float, ...]) -> Color:
    result = _vector3(value, "color")
    if any(channel < 0.0 or channel > 1.0 for channel in result):
        raise ValueError("color 必須位於 0 到 1 之間")
    return result


@dataclass(frozen=True)
class PoseSpec:
    """以公尺與角度描述剛體在父座標系中的姿態。"""

    xyz: Vector3
    rpy_deg: Vector3

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PoseSpec:
        return cls(
            xyz=_vector3(data.get("xyz", [0.0, 0.0, 0.0]), "pose.xyz"),
            rpy_deg=_vector3(data.get("rpy_deg", [0.0, 0.0, 0.0]), "pose.rpy_deg"),
        )


@dataclass(frozen=True)
class TableSpec:
    """桌面的尺寸、姿態與點雲取樣設定。"""

    size: Vector3
    pose: PoseSpec
    color: Color
    point_count: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> TableSpec:
        size = _vector3(data["size"], "table.size")
        _positive(size, "table.size")
        return cls(
            size=size,
            pose=PoseSpec.from_dict(data["pose"]),
            color=_color(data.get("color", [0.5, 0.5, 0.5])),
            point_count=int(data.get("point_count", 5000)),
        )


@dataclass(frozen=True)
class ObjectSpec:
    """可放入場景的盒、圓柱或球體。"""

    name: str
    shape: str
    dimensions: tuple[float, ...]
    pose: PoseSpec
    color: Color
    point_count: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ObjectSpec:
        shape = str(data["shape"]).lower()
        dimensions = tuple(float(value) for value in data["dimensions"])
        expected_dimension_count = {"box": 3, "cylinder": 2, "sphere": 1}
        if shape not in expected_dimension_count:
            raise ValueError(f"不支援的物件形狀：{shape}")
        if len(dimensions) != expected_dimension_count[shape]:
            raise ValueError(f"{shape} 的 dimensions 數量不正確")
        _positive(dimensions, f"object[{data['name']}].dimensions")
        return cls(
            name=str(data["name"]),
            shape=shape,
            dimensions=dimensions,
            pose=PoseSpec.from_dict(data["pose"]),
            color=_color(data.get("color", [0.5, 0.5, 0.5])),
            point_count=int(data.get("point_count", 2500)),
        )


@dataclass(frozen=True)
class RobotLinkSpec:
    """一個旋轉關節及其後方連桿的簡化模型。"""

    name: str
    joint_axis: str
    joint_angle_deg: float
    translation: Vector3
    radius: float
    joint_limits_deg: tuple[float, float]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotLinkSpec:
        axis = str(data["joint_axis"]).lower()
        if axis not in {"x", "y", "z"}:
            raise ValueError(f"joint_axis 必須是 x、y 或 z：{axis}")
        radius = float(data["radius"])
        _positive((radius,), f"robot.links[{data['name']}].radius")
        limits = tuple(float(value) for value in data.get("joint_limits_deg", [-180.0, 180.0]))
        if len(limits) != 2 or limits[0] >= limits[1]:
            raise ValueError(
                f"robot.links[{data['name']}].joint_limits_deg 必須是遞增的兩個角度"
            )
        joint_angle_deg = float(data["joint_angle_deg"])
        if not limits[0] <= joint_angle_deg <= limits[1]:
            raise ValueError(
                f"robot.links[{data['name']}].joint_angle_deg 超出關節限制"
            )
        return cls(
            name=str(data["name"]),
            joint_axis=axis,
            joint_angle_deg=joint_angle_deg,
            translation=_vector3(data["translation"], "robot.link.translation"),
            radius=radius,
            joint_limits_deg=(limits[0], limits[1]),
        )


@dataclass(frozen=True)
class GripperSpec:
    """平行夾爪的手掌、手指與開口尺寸。"""

    palm_size: Vector3
    finger_size: Vector3
    opening: float
    color: Color

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> GripperSpec:
        palm_size = _vector3(data["palm_size"], "gripper.palm_size")
        finger_size = _vector3(data["finger_size"], "gripper.finger_size")
        opening = float(data["opening"])
        _positive(palm_size, "gripper.palm_size")
        _positive(finger_size, "gripper.finger_size")
        _positive((opening,), "gripper.opening")
        return cls(
            palm_size=palm_size,
            finger_size=finger_size,
            opening=opening,
            color=_color(data.get("color", [0.8, 0.8, 0.8])),
        )


@dataclass(frozen=True)
class RobotSpec:
    """簡化序列式機械手與夾爪模型。"""

    name: str
    base_pose: PoseSpec
    base_size: Vector3
    base_color: Color
    links: tuple[RobotLinkSpec, ...]
    gripper: GripperSpec
    points_per_link: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotSpec:
        base_size = _vector3(data["base_size"], "robot.base_size")
        _positive(base_size, "robot.base_size")
        links = tuple(RobotLinkSpec.from_dict(item) for item in data["links"])
        if not links:
            raise ValueError("robot.links 不可為空")
        return cls(
            name=str(data["name"]),
            base_pose=PoseSpec.from_dict(data["base_pose"]),
            base_size=base_size,
            base_color=_color(data.get("base_color", [0.2, 0.2, 0.2])),
            links=links,
            gripper=GripperSpec.from_dict(data["gripper"]),
            points_per_link=int(data.get("points_per_link", 1000)),
        )


@dataclass(frozen=True)
class SensorNoiseSpec:
    """RGB-D 深度與相機外參的簡化誤差模型。"""

    depth_quantization_m: float
    axial_noise_std_base_m: float
    axial_noise_std_per_m2: float
    dropout_probability: float
    extrinsic_translation_std_m: float
    extrinsic_rotation_std_deg: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SensorNoiseSpec:
        values = cls(
            depth_quantization_m=float(data.get("depth_quantization_m", 0.0)),
            axial_noise_std_base_m=float(data.get("axial_noise_std_base_m", 0.0)),
            axial_noise_std_per_m2=float(data.get("axial_noise_std_per_m2", 0.0)),
            dropout_probability=float(data.get("dropout_probability", 0.0)),
            extrinsic_translation_std_m=float(
                data.get("extrinsic_translation_std_m", 0.0)
            ),
            extrinsic_rotation_std_deg=float(
                data.get("extrinsic_rotation_std_deg", 0.0)
            ),
        )
        non_negative = (
            values.depth_quantization_m,
            values.axial_noise_std_base_m,
            values.axial_noise_std_per_m2,
            values.extrinsic_translation_std_m,
            values.extrinsic_rotation_std_deg,
        )
        if any(value < 0.0 for value in non_negative):
            raise ValueError("camera.noise 的標準差與量化間距不可小於 0")
        if not 0.0 <= values.dropout_probability <= 1.0:
            raise ValueError("camera.noise.dropout_probability 必須位於 0 到 1")
        return values


@dataclass(frozen=True)
class CameraSpec:
    """以 look-at 方式描述虛擬 RGB-D 相機。"""

    name: str
    position: Vector3
    look_at: Vector3
    up: Vector3
    vertical_fov_deg: float
    aspect_ratio: float
    width: int
    height: int
    near: float
    far: float
    noise: SensorNoiseSpec

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CameraSpec:
        near = float(data["near"])
        far = float(data["far"])
        if near <= 0.0 or far <= near:
            raise ValueError("camera 必須滿足 0 < near < far")
        width = int(data.get("width", 160))
        height = int(data.get("height", 120))
        if width <= 0 or height <= 0:
            raise ValueError("camera.width 與 camera.height 必須大於 0")
        vertical_fov_deg = float(data["vertical_fov_deg"])
        if not 0.0 < vertical_fov_deg < 180.0:
            raise ValueError("camera.vertical_fov_deg 必須位於 0 到 180 度")
        aspect_ratio = float(data.get("aspect_ratio", width / height))
        if abs(aspect_ratio - width / height) > 1e-3:
            raise ValueError("camera.aspect_ratio 必須與 width / height 一致")
        return cls(
            name=str(data["name"]),
            position=_vector3(data["position"], "camera.position"),
            look_at=_vector3(data["look_at"], "camera.look_at"),
            up=_vector3(data.get("up", [0.0, 0.0, 1.0]), "camera.up"),
            vertical_fov_deg=vertical_fov_deg,
            aspect_ratio=aspect_ratio,
            width=width,
            height=height,
            near=near,
            far=far,
            noise=SensorNoiseSpec.from_dict(data.get("noise", {})),
        )


@dataclass(frozen=True)
class SceneSpec:
    """完整模擬場景設定。"""

    name: str
    seed: int
    units: str
    table: TableSpec
    objects: tuple[ObjectSpec, ...]
    robot: RobotSpec
    camera: CameraSpec

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SceneSpec:
        units = str(data.get("units", "meter"))
        if units != "meter":
            raise ValueError("第一版只接受 meter，避免單位混用")
        objects = tuple(ObjectSpec.from_dict(item) for item in data["objects"])
        if not objects:
            raise ValueError("scene.objects 不可為空")
        return cls(
            name=str(data["name"]),
            seed=int(data.get("seed", 0)),
            units=units,
            table=TableSpec.from_dict(data["table"]),
            objects=objects,
            robot=RobotSpec.from_dict(data["robot"]),
            camera=CameraSpec.from_dict(data["camera"]),
        )

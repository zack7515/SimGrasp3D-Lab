"""由簡化 RobotSpec 匯出供規劃整合使用的 URDF 與 SRDF。"""

from __future__ import annotations

import re
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from simgrasp3d.geometry.transforms import align_z_axis, rpy_deg_from_matrix
from simgrasp3d.models.specs import RobotSpec


def _safe_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value).strip("_")
    return result or "robot"


def _values(values: tuple[float, ...] | np.ndarray) -> str:
    return " ".join(f"{float(value):.10g}" for value in np.asarray(values).ravel())


def _origin(parent: ET.Element, xyz: np.ndarray, rpy_rad: np.ndarray | None = None) -> None:
    attributes = {"xyz": _values(xyz)}
    if rpy_rad is not None:
        attributes["rpy"] = _values(rpy_rad)
    ET.SubElement(parent, "origin", attributes)


def _box_link(root: ET.Element, name: str, size: np.ndarray) -> None:
    link = ET.SubElement(root, "link", {"name": name})
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "box", {"size": _values(size)})


def _cylinder_link(
    root: ET.Element,
    name: str,
    translation: np.ndarray,
    radius: float,
) -> None:
    link = ET.SubElement(root, "link", {"name": name})
    length = float(np.linalg.norm(translation))
    rotation = align_z_axis(translation)
    rpy_rad = np.deg2rad(rpy_deg_from_matrix(rotation))
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind)
        _origin(element, translation / 2.0, rpy_rad)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(
            geometry,
            "cylinder",
            {"radius": f"{radius:.10g}", "length": f"{length:.10g}"},
        )


def _joint(
    root: ET.Element,
    name: str,
    joint_type: str,
    parent_name: str,
    child_name: str,
    xyz: np.ndarray,
    axis: tuple[float, float, float] | None = None,
    limits_deg: tuple[float, float] | None = None,
) -> None:
    joint = ET.SubElement(root, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent_name})
    ET.SubElement(joint, "child", {"link": child_name})
    _origin(joint, xyz)
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": _values(axis)})
    if limits_deg is not None:
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": f"{np.deg2rad(limits_deg[0]):.10g}",
                "upper": f"{np.deg2rad(limits_deg[1]):.10g}",
                "effort": "100",
                "velocity": "2.5",
            },
        )


def build_urdf(robot: RobotSpec) -> str:
    """建立保留關節鏈與碰撞尺寸的簡化 URDF。"""

    robot_name = _safe_name(robot.name)
    root = ET.Element("robot", {"name": robot_name})
    _box_link(root, "base_link", np.asarray(robot.base_size))
    link_names = [f"arm_link_{index + 1}" for index in range(len(robot.links))]
    for name, link_spec in zip(link_names, robot.links, strict=True):
        _cylinder_link(
            root,
            name,
            np.asarray(link_spec.translation, dtype=np.float64),
            link_spec.radius,
        )
    for index, (name, link_spec) in enumerate(
        zip(link_names, robot.links, strict=True)
    ):
        parent_name = "base_link" if index == 0 else link_names[index - 1]
        origin = (
            np.asarray([0.0, 0.0, robot.base_size[2]])
            if index == 0
            else np.asarray(robot.links[index - 1].translation)
        )
        axis = {
            "x": (1.0, 0.0, 0.0),
            "y": (0.0, 1.0, 0.0),
            "z": (0.0, 0.0, 1.0),
        }[link_spec.joint_axis]
        _joint(
            root,
            _safe_name(link_spec.name),
            "revolute",
            parent_name,
            name,
            origin,
            axis,
            link_spec.joint_limits_deg,
        )

    ET.SubElement(root, "link", {"name": "flange_link"})
    _joint(
        root,
        "flange_fixed_joint",
        "fixed",
        link_names[-1],
        "flange_link",
        np.asarray(robot.links[-1].translation),
    )
    gripper = robot.gripper
    _box_link(root, "gripper_palm", np.asarray(gripper.palm_size))
    palm_center = np.asarray([gripper.palm_size[0] / 2.0, 0.0, 0.0])
    _joint(
        root,
        "gripper_palm_joint",
        "fixed",
        "flange_link",
        "gripper_palm",
        palm_center,
    )
    finger_center_x = gripper.palm_size[0] + gripper.finger_size[0] / 2.0
    finger_y = gripper.opening / 2.0 + gripper.finger_size[1] / 2.0
    for side, y_offset in (("left", finger_y), ("right", -finger_y)):
        link_name = f"gripper_{side}_finger"
        _box_link(root, link_name, np.asarray(gripper.finger_size))
        _joint(
            root,
            f"{link_name}_joint",
            "fixed",
            "flange_link",
            link_name,
            np.asarray([finger_center_x, y_offset, 0.0]),
        )
    ET.SubElement(root, "link", {"name": "tool0"})
    _joint(
        root,
        "tool0_fixed_joint",
        "fixed",
        "flange_link",
        "tool0",
        np.asarray(gripper.tcp_offset),
    )
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=False) + "\n"


def build_srdf(robot: RobotSpec) -> str:
    """建立 MoveIt 可延伸的 arm、gripper 群組與相鄰碰撞排除。"""

    root = ET.Element("robot", {"name": _safe_name(robot.name)})
    arm_group = ET.SubElement(root, "group", {"name": "arm"})
    ET.SubElement(
        arm_group,
        "chain",
        {"base_link": "base_link", "tip_link": "tool0"},
    )
    gripper_group = ET.SubElement(root, "group", {"name": "gripper"})
    for name in ("gripper_palm", "gripper_left_finger", "gripper_right_finger"):
        ET.SubElement(gripper_group, "link", {"name": name})
    ET.SubElement(
        root,
        "end_effector",
        {"name": "parallel_gripper", "parent_link": "flange_link", "group": "gripper"},
    )
    adjacent = [("base_link", "arm_link_1")]
    adjacent.extend(
        (f"arm_link_{index}", f"arm_link_{index + 1}")
        for index in range(1, len(robot.links))
    )
    adjacent.extend(
        (
            (f"arm_link_{len(robot.links)}", "flange_link"),
            ("flange_link", "gripper_palm"),
            ("flange_link", "gripper_left_finger"),
            ("flange_link", "gripper_right_finger"),
            ("flange_link", "tool0"),
        )
    )
    for first, second in adjacent:
        ET.SubElement(
            root,
            "disable_collisions",
            {"link1": first, "link2": second, "reason": "Adjacent"},
        )
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=False) + "\n"


def export_robot_description(
    output_dir: str | Path,
    robot: RobotSpec,
) -> dict[str, Path]:
    """將規劃用 URDF 與 SRDF 寫入指定目錄。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    urdf_path = destination / "learning_arm.urdf"
    srdf_path = destination / "learning_arm.srdf"
    urdf_path.write_text(build_urdf(robot), encoding="utf-8")
    srdf_path.write_text(build_srdf(robot), encoding="utf-8")
    return {"urdf": urdf_path, "srdf": srdf_path}

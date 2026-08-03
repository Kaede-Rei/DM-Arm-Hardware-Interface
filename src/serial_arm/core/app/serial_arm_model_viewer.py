#!/usr/bin/env python3
"""
URDF 机械臂模型、关节坐标系与旋转轴可视化工具

特点：
1. 从 arm.yaml 的 model.urdf_path 读取机械臂 URDF
2. 解析 URDF 中的 STL visual / collision 网格并显示完整机械臂模型
3. 显示各关节的 RGB 坐标系、黑色关节轴及关节名称标签
4. 支持逐关节或全局开关坐标系、关节轴和标签
5. 支持显示固定关节坐标系、模型透明度调节和线框模式
6. 在窗口左侧显示坐标轴颜色说明和当前显示状态菜单
7. 支持通过命令行参数设置窗口尺寸、视角、缩放和显示选项

依赖：
    python3 -m pip install numpy vtk pyyaml

运行：
    python3 serial_arm_model_viewer.py --config /path/to/robot.yaml
    python3 serial_arm_model_viewer.py --fixed-joints
    python3 serial_arm_model_viewer.py --no-show-model
    python3 serial_arm_model_viewer.py --opacity 0.6
    python3 serial_arm_model_viewer.py --help

操作：
- 鼠标左键拖动：旋转视角
- 鼠标中键拖动：平移视角
- 鼠标滚轮：缩放视角
- 上 / 下：选择上一个或下一个关节
- 1：显示或隐藏当前关节坐标系
- 2：显示或隐藏当前关节轴
- 3：显示或隐藏当前关节轴标签
- M：显示或隐藏完整机械臂模型
- F：显示或隐藏全部关节坐标系
- A：显示或隐藏全部关节轴
- L：显示或隐藏全部关节轴标签
- X：显示或隐藏固定关节坐标系
- W：切换实体与线框显示
- + / -：增加或降低模型透明度
- R：复位视角
- H：显示或隐藏左侧菜单
- 关闭窗口：退出程序

坐标轴颜色：
- X：红色
- Y：绿色
- Z：蓝色
- URDF 关节轴：黑色
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import vtk
import yaml

PACKAGE_NAME = "serial_arm_core"
WINDOW_TITLE = "Model Viewer"


@dataclass
class Mesh:
    filename: Path
    origin: np.ndarray
    scale: np.ndarray


@dataclass
class Link:
    name: str
    meshes: list[Mesh] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass
class JointDisplay:
    joint: Joint
    frame_actor: vtk.vtkProp3D
    axis_actor: vtk.vtkProp3D | None = None
    label_actor: vtk.vtkProp3D | None = None
    show_frame: bool = True
    show_axis: bool = True
    show_label: bool = True


def parse_vec(text: str | None, default: tuple[float, ...]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_from_origin(origin: ET.Element | None) -> np.ndarray:
    xyz = parse_vec(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
    rpy = parse_vec(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
    tf = np.eye(4)
    tf[:3, :3] = rpy_to_matrix(rpy)
    tf[:3, 3] = xyz
    return tf


def load_urdf_path(config_path: Path) -> Path:
    with config_path.open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    urdf_path = Path(cfg["model"]["urdf_path"])
    if not urdf_path.is_absolute():
        urdf_path = (config_path.parent / urdf_path).resolve()
    return urdf_path


def resolve_mesh_path(filename: str, package_root: Path) -> Path:
    prefix = f"package://{PACKAGE_NAME}/"
    if filename.startswith(prefix):
        return package_root / filename[len(prefix) :]
    if filename.startswith("package://"):
        raise ValueError(f"unsupported package URI: {filename}")
    path = Path(filename)
    return path if path.is_absolute() else package_root / path


def parse_urdf(
    urdf_path: Path, package_root: Path
) -> tuple[dict[str, Link], list[Joint]]:
    root = ET.parse(urdf_path).getroot()
    links: dict[str, Link] = {}
    for link_node in root.findall("link"):
        link = Link(name=link_node.attrib["name"])
        for node_name in ("collision", "visual"):
            for geom_parent in link_node.findall(node_name):
                mesh_node = geom_parent.find("geometry/mesh")
                if mesh_node is None or "filename" not in mesh_node.attrib:
                    continue
                mesh_path = resolve_mesh_path(
                    mesh_node.attrib["filename"], package_root
                )
                if mesh_path.suffix.lower() != ".stl":
                    continue
                link.meshes.append(
                    Mesh(
                        filename=mesh_path,
                        origin=transform_from_origin(geom_parent.find("origin")),
                        scale=parse_vec(mesh_node.get("scale"), (1.0, 1.0, 1.0)),
                    )
                )
                break
        links[link.name] = link

    joints: list[Joint] = []
    for joint_node in root.findall("joint"):
        axis_node = joint_node.find("axis")
        axis = parse_vec(
            axis_node.get("xyz") if axis_node is not None else None, (0.0, 0.0, 1.0)
        )
        norm = np.linalg.norm(axis)
        if norm > 0.0:
            axis = axis / norm
        joints.append(
            Joint(
                name=joint_node.attrib["name"],
                joint_type=joint_node.attrib.get("type", "fixed"),
                parent=joint_node.find("parent").attrib["link"],
                child=joint_node.find("child").attrib["link"],
                origin=transform_from_origin(joint_node.find("origin")),
                axis=axis,
            )
        )
    return links, joints


def compute_poses(
    links: dict[str, Link], joints: list[Joint]
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    children = {joint.child for joint in joints}
    roots = [name for name in links if name not in children]
    if not roots:
        raise ValueError("URDF has no root link")

    by_parent: dict[str, list[Joint]] = {}
    for joint in joints:
        by_parent.setdefault(joint.parent, []).append(joint)

    link_poses = {roots[0]: np.eye(4)}
    joint_poses: dict[str, np.ndarray] = {}
    stack = [roots[0]]
    while stack:
        parent = stack.pop()
        for joint in by_parent.get(parent, []):
            joint_tf = link_poses[parent] @ joint.origin
            joint_poses[joint.name] = joint_tf
            link_poses[joint.child] = joint_tf
            stack.append(joint.child)
    return link_poses, joint_poses


def to_vtk_transform(tf: np.ndarray) -> vtk.vtkTransform:
    matrix = vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4):
            matrix.SetElement(row, col, float(tf[row, col]))
    transform = vtk.vtkTransform()
    transform.SetMatrix(matrix)
    return transform


def make_stl_actor(mesh: Mesh, link_tf: np.ndarray, opacity: float) -> vtk.vtkActor:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh.filename))

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetUserTransform(to_vtk_transform(link_tf @ mesh.origin))
    actor.SetScale(float(mesh.scale[0]), float(mesh.scale[1]), float(mesh.scale[2]))
    actor.GetProperty().SetColor(0.58, 0.64, 0.68)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetInterpolationToPhong()
    return actor


def make_joint_frame(tf: np.ndarray, length: float) -> vtk.vtkAxesActor:
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(length, length, length)
    axes.SetShaftTypeToCylinder()
    axes.SetCylinderRadius(0.035)
    axes.SetConeRadius(0.12)
    axes.SetSphereRadius(0.055)
    axes.SetUserTransform(to_vtk_transform(tf))
    axes.AxisLabelsOff()
    return axes


def make_joint_axis(tf: np.ndarray, axis: np.ndarray, length: float) -> vtk.vtkActor:
    origin = tf[:3, 3]
    direction = tf[:3, :3] @ axis
    end = origin + direction * length

    line = vtk.vtkLineSource()
    line.SetPoint1(float(origin[0]), float(origin[1]), float(origin[2]))
    line.SetPoint2(float(end[0]), float(end[1]), float(end[2]))

    tube = vtk.vtkTubeFilter()
    tube.SetInputConnection(line.GetOutputPort())
    tube.SetRadius(length * 0.035)
    tube.SetNumberOfSides(16)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(tube.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.02, 0.02, 0.02)
    return actor


def make_axis_label(
    tf: np.ndarray, axis: np.ndarray, name: str, length: float
) -> vtk.vtkBillboardTextActor3D:
    origin = tf[:3, 3]
    direction = tf[:3, :3] @ axis
    position = origin + direction * (length * 1.08)

    label = vtk.vtkBillboardTextActor3D()
    label.SetInput(f"{name} axis")
    label.SetPosition(float(position[0]), float(position[1]), float(position[2]))
    prop = label.GetTextProperty()
    prop.SetFontSize(18)
    prop.SetColor(0.02, 0.02, 0.02)
    prop.SetBackgroundColor(1.0, 1.0, 1.0)
    prop.SetBackgroundOpacity(0.45)
    return label


def make_text_actor(font_size: int, position: tuple[int, int]) -> vtk.vtkTextActor:
    text = vtk.vtkTextActor()
    text.SetPosition(*position)
    prop = text.GetTextProperty()
    prop.SetFontSize(font_size)
    prop.SetColor(0.05, 0.05, 0.05)
    prop.SetFontFamilyToCourier()
    return text


class ModelAxisViewer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.urdf_path = load_urdf_path(args.config.resolve())
        package_root = self.urdf_path.parents[2]
        self.links, self.joints = parse_urdf(self.urdf_path, package_root)
        self.link_poses, self.joint_poses = compute_poses(self.links, self.joints)

        self.show_model = args.show_model
        self.show_joint_frames = args.show_joint_frames
        self.show_joint_axes = args.show_joint_axes
        self.show_axis_labels = args.show_axis_labels
        self.show_fixed_joints = args.fixed_joints
        self.show_menu = True
        self.wireframe = False
        self.opacity = args.opacity
        self.selected_joint_index = 0

        self.mesh_actors: list[vtk.vtkActor] = []
        self.mesh_representations: dict[vtk.vtkActor, int] = {}
        self.joint_displays: list[JointDisplay] = []

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.96, 0.97, 0.98)
        self.legend_actor = make_text_actor(18, (16, 16))
        self.menu_actor = make_text_actor(15, (16, 48))
        self.renderer.AddActor2D(self.legend_actor)
        self.renderer.AddActor2D(self.menu_actor)

        self.window = vtk.vtkRenderWindow()
        self.window.SetWindowName(WINDOW_TITLE)
        self.window.SetSize(args.width, args.height)
        self.window.AddRenderer(self.renderer)
        self.window.SetMultiSamples(4)

        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.window)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.AddObserver("KeyPressEvent", self.on_key_press_event)
        self.interactor.AddObserver("TimerEvent", self.on_timer_event)

        self.build_scene()
        self.apply_display_config(render_now=False)

    def build_scene(self) -> None:
        for link_name, link in self.links.items():
            link_tf = self.link_poses.get(link_name)
            if link_tf is None:
                continue
            for mesh in link.meshes:
                if not mesh.filename.exists():
                    continue
                actor = make_stl_actor(mesh, link_tf, self.opacity)
                self.mesh_actors.append(actor)
                self.mesh_representations[actor] = (
                    actor.GetProperty().GetRepresentation()
                )
                self.renderer.AddActor(actor)

        for joint in self.joints:
            joint_tf = self.joint_poses.get(joint.name)
            if joint_tf is None:
                continue
            frame_actor = make_joint_frame(joint_tf, self.args.frame_length)
            self.renderer.AddActor(frame_actor)

            axis_actor = None
            label_actor = None
            if joint.joint_type != "fixed":
                axis_actor = make_joint_axis(
                    joint_tf, joint.axis, self.args.axis_length
                )
                self.renderer.AddActor(axis_actor)

                label_actor = make_axis_label(
                    joint_tf, joint.axis, joint.name, self.args.axis_length
                )
                self.renderer.AddActor(label_actor)

            is_fixed = joint.joint_type == "fixed"
            self.joint_displays.append(
                JointDisplay(
                    joint=joint,
                    frame_actor=frame_actor,
                    axis_actor=axis_actor,
                    label_actor=label_actor,
                    show_frame=self.show_joint_frames
                    and (not is_fixed or self.show_fixed_joints),
                    show_axis=self.show_joint_axes and not is_fixed,
                    show_label=self.show_axis_labels and not is_fixed,
                )
            )

    def apply_display_config(self, render_now: bool = True) -> None:
        for actor in self.mesh_actors:
            actor.SetVisibility(self.show_model)
            prop = actor.GetProperty()
            prop.SetOpacity(self.opacity)
            if self.wireframe:
                prop.SetRepresentationToWireframe()
                prop.EdgeVisibilityOff()
            else:
                prop.SetRepresentation(
                    self.mesh_representations.get(actor, vtk.VTK_SURFACE)
                )
                prop.EdgeVisibilityOff()
            prop.Modified()
            actor.Modified()

        for display in self.joint_displays:
            display.frame_actor.SetVisibility(display.show_frame)
            if display.axis_actor is not None:
                display.axis_actor.SetVisibility(display.show_axis)
            if display.label_actor is not None:
                display.label_actor.SetVisibility(
                    display.show_axis and display.show_label
                )

        self.legend_actor.SetInput("X red   Y green   Z blue   joint axis black")
        self.legend_actor.SetVisibility(self.show_menu)
        self.menu_actor.SetInput(self.menu_text())
        self.menu_actor.SetVisibility(self.show_menu)

        if render_now:
            self.window.Render()

    def menu_text(self) -> str:
        rows = [
            "[Up/Down] select joint   [1] frame   [2] axis   [3] label",
            "[M] model "
            + self.on_off(self.show_model)
            + "   [F/A/L] all frame/axis/label   [W] wire "
            + self.on_off(self.wireframe)
            + "   [+/-] opacity "
            + f"{self.opacity:.2f}",
            "[R] reset camera   [H] hide menu",
            "",
            "Joint                 Frame Axis Label",
        ]
        for index, display in enumerate(self.joint_displays):
            marker = ">" if index == self.selected_joint_index else " "
            axis_text = (
                self.check_text(display.show_axis)
                if display.axis_actor is not None
                else " -- "
            )
            label_text = (
                self.check_text(display.show_label)
                if display.label_actor is not None
                else " -- "
            )
            rows.append(
                f"{marker} {display.joint.name:<20} "
                f"{self.check_text(display.show_frame):<5} "
                f"{axis_text:<4} "
                f"{label_text:<5}"
            )
        return "\n".join(rows)

    @staticmethod
    def on_off(value: bool) -> str:
        return "on" if value else "off"

    @staticmethod
    def check_text(value: bool) -> str:
        return "[x]" if value else "[ ]"

    def selected_joint(self) -> JointDisplay | None:
        if not self.joint_displays:
            return None
        return self.joint_displays[self.selected_joint_index]

    def set_all_frames(self, value: bool) -> None:
        self.show_joint_frames = value
        for display in self.joint_displays:
            if display.joint.joint_type == "fixed" and not self.show_fixed_joints:
                display.show_frame = False
            else:
                display.show_frame = value

    def set_all_axes(self, value: bool) -> None:
        self.show_joint_axes = value
        for display in self.joint_displays:
            if display.axis_actor is not None:
                display.show_axis = value

    def set_all_labels(self, value: bool) -> None:
        self.show_axis_labels = value
        for display in self.joint_displays:
            if display.label_actor is not None:
                display.show_label = value

    def handle_key(self, key: str) -> bool:
        key = key.lower()
        if key == "m":
            self.show_model = not self.show_model
        elif key == "a":
            self.set_all_axes(not self.show_joint_axes)
        elif key == "l":
            self.set_all_labels(not self.show_axis_labels)
        elif key == "f":
            self.set_all_frames(not self.show_joint_frames)
        elif key == "x":
            self.show_fixed_joints = not self.show_fixed_joints
            for display in self.joint_displays:
                if display.joint.joint_type == "fixed":
                    display.show_frame = (
                        self.show_fixed_joints and self.show_joint_frames
                    )
        elif key == "w":
            self.wireframe = not self.wireframe
        elif key in ("up", "kp_up"):
            self.selected_joint_index = max(0, self.selected_joint_index - 1)
        elif key in ("down", "kp_down"):
            self.selected_joint_index = min(
                len(self.joint_displays) - 1, self.selected_joint_index + 1
            )
        elif key in ("1", "kp_1"):
            display = self.selected_joint()
            if display is not None:
                display.show_frame = not display.show_frame
        elif key in ("2", "kp_2"):
            display = self.selected_joint()
            if display is not None and display.axis_actor is not None:
                display.show_axis = not display.show_axis
        elif key in ("3", "kp_3"):
            display = self.selected_joint()
            if display is not None and display.label_actor is not None:
                display.show_label = not display.show_label
        elif key in ("plus", "equal", "kp_add"):
            self.opacity = min(1.0, self.opacity + 0.1)
        elif key in ("minus", "underscore", "kp_subtract"):
            self.opacity = max(0.1, self.opacity - 0.1)
        elif key == "h":
            self.show_menu = not self.show_menu
        elif key == "r":
            self.renderer.ResetCamera()
            self.renderer.GetActiveCamera().Zoom(self.args.zoom)
            self.renderer.ResetCameraClippingRange()
        else:
            return False
        self.apply_display_config()
        return True

    def on_key_press_event(self, obj, event) -> None:
        key = self.interactor.GetKeySym()
        if self.handle_key(key) and key.lower() == "w":
            self.interactor.CreateOneShotTimer(1)

    def on_timer_event(self, obj, event) -> None:
        self.apply_display_config()

    def start(self) -> None:
        self.renderer.ResetCamera()
        camera = self.renderer.GetActiveCamera()
        camera.Azimuth(self.args.azim)
        camera.Elevation(self.args.elev)
        camera.Zoom(self.args.zoom)
        self.renderer.ResetCameraClippingRange()

        self.window.Render()
        self.interactor.Initialize()
        self.interactor.Start()


def render(args: argparse.Namespace) -> None:
    ModelAxisViewer(args).start()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to arm.yaml"
    )
    parser.add_argument(
        "--show-model",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show the complete STL model",
    )
    parser.add_argument(
        "--show-joint-frames",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show RGB frames for movable joints",
    )
    parser.add_argument(
        "--show-joint-axes",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show black URDF joint axes",
    )
    parser.add_argument(
        "--show-axis-labels",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show labels for movable joint axes",
    )
    parser.add_argument(
        "--fixed-joints", action="store_true", help="Draw fixed joint frames too"
    )
    parser.add_argument(
        "--opacity", type=float, default=0.82, help="Initial model opacity"
    )
    parser.add_argument(
        "--frame-length",
        type=float,
        default=0.10,
        help="RGB frame axis length in meters",
    )
    parser.add_argument(
        "--axis-length",
        type=float,
        default=0.16,
        help="Black joint axis length in meters",
    )
    parser.add_argument("--width", type=int, default=1280, help="Window width")
    parser.add_argument("--height", type=int, default=900, help="Window height")
    parser.add_argument(
        "--elev", type=float, default=18.0, help="Initial camera elevation"
    )
    parser.add_argument(
        "--azim", type=float, default=-68.0, help="Initial camera azimuth"
    )
    parser.add_argument("--zoom", type=float, default=1.35, help="Initial camera zoom")
    return parser.parse_args()


def main() -> int:
    try:
        render(parse_args())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

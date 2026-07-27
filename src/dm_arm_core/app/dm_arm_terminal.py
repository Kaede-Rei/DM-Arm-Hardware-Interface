#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DM-Arm pybind11 Python 交互终端

使用流程
--------
1 创建虚拟环境并安装 Python 构建依赖

    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install numpy pybind11 scikit-build-core build

2 Pinocchio 位于 /opt/openrobots 时加载环境

    export PATH=/opt/openrobots/bin:$PATH
    export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH

3 使用 scikit-build-core 调用 CMake 编译 pybind11 扩展并生成 wheel

    cd python
    python -m build --wheel

    python -m pip install --force-reinstall dist/dm_arm-*.whl
    cd ..

4 验证 wheel 是否正确安装

    python -c "import dm_arm; print(dm_arm.__file__)"
    python -c "import dm_arm; print(getattr(dm_arm, '__version__', 'unknown'))"

输出路径应位于当前 .venv 的 site-packages 中，而不是仓库源码目录

5 先做无真机检查

    python dm_arm_python_terminal.py \
        --config config/dm_arm_white.yaml \
        --check-only

6 确认机械臂已支撑、急停可用、零位、方向、限位和电机型号正确后启动真机

    python dm_arm_python_terminal.py \
        --config config/dm_arm_white.yaml \
        --allow-hardware

依赖
----
Python >= 3.10
NumPy >= 1.24
pybind11 >= 2.11
scikit-build-core >= 0.10
build
python3-dev
CMake
C++17 编译器
yaml-cpp
Pinocchio

构建 wheel 时必须启用

    DM_ARM_BUILD_PYTHON=ON
    DM_ARM_ENABLE_DYNAMICS=ON

需要通过 Python 控制达妙真机时还必须启用

    DM_ARM_BUILD_DAMIAO=ON

安全说明
--------
真机启动必须同时满足

    命令行传入 --allow-hardware
    RobotSession 构造参数 allow_hardware=True
    runtime.write_enabled=true
    wheel 构建时启用 DM_ARM_BUILD_DAMIAO

模型前馈模式应在 INACTIVE 状态设置

RobotSession.stop() 当前会直接停止 C++ worker 并调用底层 deactivate；
它本身不包含 C++ 终端中的停放轨迹

本脚本的菜单 0 和菜单 3 会先执行

    move_to(park_pos)
    → 等待实测位置与速度判据
    → hold_current()
    → stop()

通信中断或 FAULT 时，软件不能保证所有关节仍有保持力矩；
应先使用机械支撑，再决定是否强制失能
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

DEFAULT_CONFIG = "config/dm_arm.yaml"


def enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    return str(value).rsplit(".", 1)[-1]


def vector(value: Any) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(value, dtype=np.float64).reshape(-1))


def print_vector(name: str, value: Any) -> None:
    values = vector(value)
    text = ", ".join(f"{item:.6f}" for item in values)
    print(f"{name:<24} [{text}]")


def print_matrix(name: str, value: Any) -> None:
    data = np.asarray(value, dtype=np.float64)
    print(f"{name} {data.shape}:")
    print(np.array2string(data, precision=6, suppress_small=False))


def read_int(prompt: str) -> Optional[int]:
    try:
        return int(input(prompt).strip())
    except EOFError:
        return None
    except ValueError:
        return -1


def read_float(prompt: str) -> Optional[float]:
    try:
        value = float(input(prompt).strip())
    except EOFError:
        return None
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def read_vector(prompt: str, size: int) -> Optional[np.ndarray]:
    try:
        parts = input(prompt).strip().split()
    except EOFError:
        return None
    if len(parts) != size:
        return np.array([], dtype=np.float64)
    try:
        values = np.asarray([float(item) for item in parts], dtype=np.float64)
    except ValueError:
        return np.array([], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        return np.array([], dtype=np.float64)
    return np.ascontiguousarray(values)


class Terminal:
    def __init__(self, dm_arm: Any, config: Path) -> None:
        self.dm_arm = dm_arm
        self.config_path = config
        self.cfg = dm_arm.load_robot_cfg(str(config))
        self.session = dm_arm.RobotSession(config, allow_hardware=True)
        self.joint_names = list(self.cfg.joint_names)
        self.dynamics = dm_arm.Dynamics()
        self.dynamics.configure(self.cfg.dynamics)
        self.quit = False

    def banner(self) -> None:
        print("\n==============================================")
        print(" DM-Arm Python Binding Terminal")
        print(f" dm_arm : {getattr(self.dm_arm, '__version__', 'unknown')}")
        print(f" config : {self.config_path}")
        print("==============================================")
        print("[危险] 必须确认机械臂已支撑、急停可用、零位、方向、限位和电机型号正确")
        print("[说明] 实时循环由 C++ RobotSession worker 维护")

    def menu(self) -> None:
        print("\n------------ 主菜单 ------------")
        print(" 1. 查看 RobotSession 状态")
        print(" 2. start() / activate()")
        print(" 3. 回到停放姿态并 stop()")
        print(" 4. clear_fault()")
        print(" 5. 切换阻抗模式")
        print(" 6. 切换模型前馈模式（仅 INACTIVE）")
        print(" 7. 梯形参考移动到绝对位置")
        print(" 8. 梯形参考执行相对移动")
        print(" 9. 取消目标并切换当前位置刚性保持")
        print("10. 查看 Joint / Actuator 周期状态")
        print("11. 查看动力学向量与末端位姿")
        print("12. 查看质量矩阵与末端 Jacobian")
        print("13. 查看执行器静态参数")
        print("14. 设置重力补偿比例")
        print("15. 查看配置摘要")
        print("16. 读取指定 Frame 位姿与 Jacobian")
        print("17. 执行 pybinding 只读自检")
        print("18. 连续监视状态")
        print("21. 立即 stop() 并失能（危险）")
        print("22. FAULT 进入受限柔性恢复")
        print("23. FAULT 返回刚性保持")
        print("24. 查看当前故障恢复模式")
        print(" 0. 回到停放姿态并安全退出")

    def run(self) -> int:
        self.banner()
        handlers: dict[int, Callable[[], Any]] = {
            0: self.safe_exit,
            1: self.summary,
            2: self.start,
            3: self.park_and_stop,
            4: self.clear_fault,
            5: self.set_impedance,
            6: self.set_model_mode,
            7: self.move_absolute,
            8: self.move_relative,
            9: self.hold,
            10: self.show_states,
            11: self.show_dynamics,
            12: self.show_matrices,
            13: self.show_actuators,
            14: self.set_gravity_scale,
            15: self.show_config,
            16: self.show_frame,
            17: self.self_check,
            18: self.monitor,
            21: self.force_stop,
            22: self.enter_fault_compliant_recovery,
            23: self.return_to_fault_rigid_hold,
            24: self.show_fault_hold_mode,
        }

        while not self.quit:
            self.menu()
            try:
                choice = read_int("请选择: ")
                if choice is None:
                    self.safe_exit()
                    continue
                handler = handlers.get(choice)
                if handler is None:
                    print("未知菜单编号")
                    continue
                handler()
            except KeyboardInterrupt:
                print("\n收到 KeyboardInterrupt，将尝试安全停放")
                self.safe_exit()
            except self.dm_arm.DmArmError as error:
                print(f"dm_arm 调用失败: {error}")
            except Exception as error:
                print(f"Python 终端异常: {type(error).__name__}: {error}")
        return 0

    def start(self) -> None:
        if self.session.running:
            print("RobotSession 已经运行")
            return
        self.session.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snapshot = self.session.snapshot
            if snapshot.valid:
                print("start() 成功，C++ worker 已获得合法周期快照")
                return
            if snapshot.last_error:
                print(f"后台错误: {snapshot.last_error}")
                return
            time.sleep(0.02)
        print("start() 已返回，但尚未获得合法周期快照")

    def park_and_stop(self) -> bool:
        if not self.session.running:
            print("RobotSession 已经停止")
            return True
        if self.session.state == self.dm_arm.RobotState.FAULT:
            print("Robot 处于 FAULT，不能执行正常停放轨迹")
            return False

        shutdown = self.cfg.shutdown
        if not shutdown.park_before_disable:
            print("park_before_disable=false；安全路径不会自动直接失能，请使用菜单 21")
            return False

        target = vector(shutdown.park_pos)
        print("开始回到配置的停放姿态")
        print_vector("park_pos", target)

        self.session.set_impedance_mode(self.dm_arm.JointImpedanceMode.RIGID_TRACKING)
        self.session.move_to(target, float(shutdown.speed_scale))

        started = time.monotonic()
        last_report = started
        settled: Optional[float] = None
        max_pos_error = math.inf
        max_vel = math.inf
        pos_index = 0
        vel_index = 0

        while self.session.running:
            now = time.monotonic()
            snapshot = self.session.snapshot
            if snapshot.last_error:
                print(f"停放过程中后台错误: {snapshot.last_error}")
                return False
            if not snapshot.valid:
                time.sleep(0.02)
                continue

            pos_error = np.abs(vector(snapshot.cycle.joint_state.pos) - target)
            vel = np.abs(vector(snapshot.cycle.joint_state.vel))
            pos_index = int(np.argmax(pos_error))
            vel_index = int(np.argmax(vel))
            max_pos_error = float(pos_error[pos_index])
            max_vel = float(vel[vel_index])

            reached = max_pos_error <= float(
                shutdown.position_tolerance
            ) and max_vel <= float(shutdown.velocity_tolerance)
            if reached:
                if settled is None:
                    settled = now
                if now - settled >= float(shutdown.settle_time_s):
                    break
            else:
                settled = None

            if now - last_report >= 1.0:
                print(
                    f"[停放] 最大位置误差={max_pos_error:.6f} rad ({self.joint_names[pos_index]})，"
                    f"最大速度={max_vel:.6f} rad/s ({self.joint_names[vel_index]})"
                )
                last_report = now

            if now - started > float(shutdown.timeout_s):
                ratio = float(getattr(shutdown, "relaxed_tolerance_ratio", 2.0))
                relaxed_pos = float(shutdown.position_tolerance) * ratio
                relaxed_vel = float(shutdown.velocity_tolerance) * ratio
                if max_pos_error <= relaxed_pos and max_vel <= relaxed_vel:
                    print("[停放] 严格判据超时，但满足宽松判据，将继续保持并失能")
                    break
                self.session.hold_current()
                print("停放超时，已请求 RIGID_HOLD 并取消 stop()")
                return False
            time.sleep(0.02)

        self.session.hold_current()
        time.sleep(0.2)
        self.session.stop()
        print("已到达停放姿态并 stop()")
        return True

    def safe_exit(self) -> bool:
        if self.session.running and not self.park_and_stop():
            print("安全退出取消，RobotSession 仍在运行")
            return False
        self.quit = True
        print("Python 终端退出")
        return True

    def force_stop(self) -> None:
        if not self.session.running:
            print("RobotSession 已经停止")
            return
        if input("机械臂可能立即下落；确认已支撑后输入 FORCE: ").strip() != "FORCE":
            print("已取消")
            return
        self.session.stop()
        print("已立即 stop() 并失能")

    def clear_fault(self) -> None:
        self.session.clear_fault()
        print(f"clear_fault() 完成，state={enum_name(self.session.state)}")

    def enter_fault_compliant_recovery(self) -> None:
        self.session.enter_fault_compliant_recovery()
        print("已进入 FAULT 受限柔性恢复")

    def return_to_fault_rigid_hold(self) -> None:
        self.session.return_to_fault_rigid_hold()
        print("已返回 FAULT 刚性保持")

    def show_fault_hold_mode(self) -> None:
        print(f"FaultHoldMode           : {enum_name(self.session.fault_hold_mode)}")

    def set_impedance(self) -> None:
        print(
            "\n1 RIGID_HOLD\n2 RIGID_TRACKING\n3 COMPLIANT_HOLD\n4 COMPLIANT_DRAG\n5 COMPLIANT_TRACKING"
        )
        modes = {
            1: self.dm_arm.JointImpedanceMode.RIGID_HOLD,
            2: self.dm_arm.JointImpedanceMode.RIGID_TRACKING,
            3: self.dm_arm.JointImpedanceMode.COMPLIANT_HOLD,
            4: self.dm_arm.JointImpedanceMode.COMPLIANT_DRAG,
            5: self.dm_arm.JointImpedanceMode.COMPLIANT_TRACKING,
        }
        mode = modes.get(read_int("模式: "))
        if mode is None:
            print("模式输入无效")
            return
        self.session.set_impedance_mode(mode)
        print(f"已请求切换为 {enum_name(mode)}")

    def set_model_mode(self) -> None:
        print("\n1 NONE\n2 GRAVITY\n3 FULL_INVERSE_DYNAMICS")
        modes = {
            1: self.dm_arm.ModelFeedforwardMode.NONE,
            2: self.dm_arm.ModelFeedforwardMode.GRAVITY,
            3: self.dm_arm.ModelFeedforwardMode.FULL_INVERSE_DYNAMICS,
        }
        mode = modes.get(read_int("模式: "))
        if mode is None:
            print("模式输入无效")
            return
        self.session.set_model_feedforward_mode(mode)
        print(f"模型前馈模式已切换为 {enum_name(mode)}")

    def move_absolute(self) -> None:
        target = read_vector(
            f"输入 {len(self.joint_names)} 个目标位置 rad: ", len(self.joint_names)
        )
        speed = read_float("速度比例 (0, 1]: ")
        if target is None or target.size != len(self.joint_names):
            print("目标输入无效")
            return
        if speed is None or not math.isfinite(speed) or speed <= 0.0 or speed > 1.0:
            print("速度比例无效")
            return
        self.session.move_to(target, speed)
        print("绝对目标已提交")

    def move_relative(self) -> None:
        snapshot = self.session.snapshot
        if not snapshot.valid:
            print("请先 start() 并等待合法周期快照")
            return
        delta = read_vector(
            f"输入 {len(self.joint_names)} 个相对位移 rad: ", len(self.joint_names)
        )
        speed = read_float("速度比例 (0, 1]: ")
        if delta is None or delta.size != len(self.joint_names):
            print("相对位移输入无效")
            return
        if speed is None or not math.isfinite(speed) or speed <= 0.0 or speed > 1.0:
            print("速度比例无效")
            return
        target = vector(snapshot.cycle.joint_state.pos) + delta
        self.session.move_to(target, speed)
        print_vector("relative target", target)

    def hold(self) -> None:
        self.session.hold_current()
        print("已取消目标并请求 RIGID_HOLD")

    def summary(self) -> None:
        snapshot = self.session.snapshot
        print(f"RobotState             : {enum_name(self.session.state)}")
        print(f"FaultHoldMode          : {enum_name(self.session.fault_hold_mode)}")
        print(f"configured             : {self.session.configured}")
        print(f"running                : {self.session.running}")
        print(f"snapshot.valid         : {snapshot.valid}")
        print(f"snapshot.last_error    : {snapshot.last_error or '<empty>'}")
        if snapshot.valid:
            print(f"cycle dt               : {snapshot.cycle.dt:.9f} s")
            print_vector("joint position", snapshot.cycle.joint_state.pos)

    def show_states(self) -> None:
        snapshot = self.session.snapshot
        if not snapshot.valid:
            print("尚无合法周期快照")
            return
        cycle = snapshot.cycle
        pos = vector(cycle.joint_state.pos)
        vel = vector(cycle.joint_state.vel)
        acc = vector(cycle.joint_acc)
        tor = vector(cycle.joint_state.tor)

        print("\nJoint feedback:")
        print(f"{'joint':<10}{'pos':<13}{'vel':<13}{'acc':<13}{'tor':<13}")
        for i, name in enumerate(self.joint_names):
            print(
                f"{name:<10}{pos[i]:<13.6f}{vel[i]:<13.6f}{acc[i]:<13.6f}{tor[i]:<13.6f}"
            )

        cmd = cycle.joint_cmd
        print("\nJoint command:")
        fields = [
            vector(cmd.pos),
            vector(cmd.vel),
            vector(cycle.joint_ref_acc),
            vector(cmd.tor),
            vector(cycle.model_feedforward),
            vector(cmd.kp),
            vector(cmd.kd),
        ]
        print(
            f"{'joint':<10}{'pos':<12}{'vel':<12}{'ref_acc':<12}{'tor':<12}{'model_ff':<12}{'kp':<10}{'kd':<10}"
        )
        for i, name in enumerate(self.joint_names):
            print(
                f"{name:<10}{fields[0][i]:<12.6f}{fields[1][i]:<12.6f}{fields[2][i]:<12.6f}"
                f"{fields[3][i]:<12.6f}{fields[4][i]:<12.6f}{fields[5][i]:<10.6f}{fields[6][i]:<10.6f}"
            )

        state = cycle.actuator_state
        act_cmd = cycle.actuator_cmd
        info = list(self.session.actuator_info)
        arrays = [
            vector(state.pos),
            vector(state.vel),
            vector(state.tor),
            np.asarray(state.online).reshape(-1),
            np.asarray(state.enabled).reshape(-1),
            np.asarray(state.err_code).reshape(-1),
            vector(act_cmd.pos),
            vector(act_cmd.vel),
            vector(act_cmd.tor),
            vector(act_cmd.kp),
            vector(act_cmd.kd),
        ]
        print("\nActuator feedback and MIT command:")
        print(
            f"{'actuator':<12}{'id':<6}{'pos':<11}{'vel':<11}{'tor':<11}{'on':<5}{'en':<5}{'err':<6}"
            f"{'cmd_pos':<11}{'cmd_vel':<11}{'cmd_tor':<11}{'kp':<9}{'kd':<9}"
        )
        for i, item in enumerate(info):
            print(
                f"{item.name:<12}{item.motor_id:<6}{arrays[0][i]:<11.5f}{arrays[1][i]:<11.5f}{arrays[2][i]:<11.5f}"
                f"{int(arrays[3][i]):<5}{int(arrays[4][i]):<5}{int(arrays[5][i]):<6}"
                f"{arrays[6][i]:<11.5f}{arrays[7][i]:<11.5f}{arrays[8][i]:<11.5f}"
                f"{arrays[9][i]:<9.5f}{arrays[10][i]:<9.5f}"
            )
        print(f"cycle dt: {cycle.dt:.9f} s")

    def show_dynamics(self) -> None:
        snapshot = self.session.snapshot
        if not snapshot.valid:
            print("尚无合法动力学快照")
            return
        info = self.session.dynamics_info
        state = snapshot.dynamics
        print(
            f"joints_count={info.joints_count}, nq={info.nq}, nv={info.nv}, total_mass={info.total_mass:.6f} kg"
        )
        for name, value in [
            ("q", state.pos),
            ("dq", state.vel),
            ("ddq_est", state.acc),
            ("tau_feedback", state.tor),
            ("ddq_ref", state.ref_acc),
            ("gravity", state.gravity),
            ("gravity_compensation", state.gravity_compensation),
            ("nonlinear", state.nonlinear),
            ("coriolis", state.coriolis),
            ("inverse_dynamics", state.inverse_dynamics),
            ("forward_dynamics", state.forward_dynamics),
        ]:
            print_vector(name, value)
        print_matrix("tool_pose", state.tool_pose)

    def show_matrices(self) -> None:
        snapshot = self.session.snapshot
        if not snapshot.valid:
            print("尚无合法动力学快照")
            return
        print_matrix("mass_matrix", snapshot.dynamics.mass_matrix)
        print_matrix("tool_jacobian", snapshot.dynamics.tool_jacobian)

    def show_actuators(self) -> None:
        print(
            f"{'actuator':<12}{'joint':<10}{'id':<7}{'master':<9}{'type':<16}{'q_max':<11}{'dq_max':<11}{'tau_max':<11}"
        )
        for item in self.session.actuator_info:
            print(
                f"{item.name:<12}{item.joint_name:<10}{item.motor_id:<7}{item.master_id:<9}{item.motor_type:<16}"
                f"{item.q_max:<11.5f}{item.dq_max:<11.5f}{item.tau_max:<11.5f}"
            )
        print_vector("pos_ratio", self.cfg.mapper.pos_ratio)
        print_vector("tor_ratio", self.cfg.mapper.tor_ratio)
        print_vector("direction", self.cfg.mapper.direction)
        print_vector("joint_zero_offset", self.cfg.mapper.joint_zero_offset)
        print_vector("actuator_zero_offset", self.cfg.mapper.actuator_zero_offset)

    def set_gravity_scale(self) -> None:
        values = read_vector(
            f"输入 {len(self.joint_names)} 个比例 [0, 1]: ", len(self.joint_names)
        )
        if (
            values is None
            or values.size != len(self.joint_names)
            or np.any(values < 0.0)
            or np.any(values > 1.0)
        ):
            print("输入无效")
            return
        self.session.set_gravity_scale(values)
        self.cfg.dynamics.gravity_scale = values
        print("gravity_scale 已提交；不会回写 YAML")

    def show_config(self) -> None:
        cfg = self.cfg
        print(f"ctrl_frequency_hz       : {cfg.runtime.ctrl_frequency_hz}")
        print(f"write_enabled           : {cfg.runtime.write_enabled}")
        print(
            f"model_feedforward_mode  : {enum_name(cfg.runtime.model_feedforward_mode)}"
        )
        print(f"park_before_disable     : {cfg.shutdown.park_before_disable}")
        print_vector("park_pos", cfg.shutdown.park_pos)
        print(f"park_speed_scale        : {cfg.shutdown.speed_scale}")
        print(f"park_position_tolerance : {cfg.shutdown.position_tolerance}")
        print(f"park_velocity_tolerance : {cfg.shutdown.velocity_tolerance}")
        print(f"park_settle_time_s      : {cfg.shutdown.settle_time_s}")
        print(
            f"park_relaxed_ratio      : {getattr(cfg.shutdown, 'relaxed_tolerance_ratio', 2.0)}"
        )
        print(f"park_timeout_s          : {cfg.shutdown.timeout_s}")
        print(f"cmd_timeout_s           : {cfg.safety.cmd_timeout_s}")
        print(f"state_timeout_s         : {cfg.safety.state_timeout_s}")
        print(f"max_dt_s                : {cfg.safety.max_dt_s}")
        print(f"serial_port             : {cfg.damiao.serial_port}")
        print(f"baudrate                : {cfg.damiao.baudrate}")
        print(f"feedback_timeout_s      : {cfg.damiao.feedback_timeout_s}")
        print(
            f"activation_retries      : {getattr(cfg.damiao, 'activation_retries', '<not bound>')}"
        )
        print(f"urdf_path               : {cfg.dynamics.urdf_path}")
        print(f"base_frame              : {cfg.dynamics.base_frame}")
        print(f"tool_frame              : {cfg.dynamics.tool_frame}")
        print_vector("gravity_scale", cfg.dynamics.gravity_scale)
        print_vector("min_pos", cfg.safety.limits.min_pos)
        print_vector("max_pos", cfg.safety.limits.max_pos)
        print_vector("max_vel", cfg.safety.limits.max_vel)
        print_vector("max_acc", cfg.safety.limits.max_acc)
        print_vector("max_effort", cfg.safety.limits.max_effort)

    def refresh_offline_dynamics(self) -> bool:
        snapshot = self.session.snapshot
        if not snapshot.valid:
            print("尚无合法周期快照")
            return False
        cycle = snapshot.cycle
        self.dynamics.update(
            vector(cycle.joint_state.pos),
            vector(cycle.joint_state.vel),
            vector(cycle.joint_acc),
            vector(cycle.joint_state.tor),
            vector(cycle.joint_ref_acc),
        )
        return True

    def show_frame(self) -> None:
        name = input("输入 Frame 名称: ").strip()
        if not name or not self.refresh_offline_dynamics():
            return
        print_matrix(f"{name} pose", self.dynamics.frame_pose(name))
        print_matrix(f"{name} Jacobian", self.dynamics.frame_jacobian(name))

    def self_check(self) -> None:
        required = [
            "RobotSession",
            "Dynamics",
            "JointCtrller",
            "JointActuatorMapper",
            "Safety",
            "load_robot_cfg",
            "RobotState",
            "JointImpedanceMode",
            "ModelFeedforwardMode",
        ]
        missing = [name for name in required if not hasattr(self.dm_arm, name)]
        if missing:
            print("缺少公开符号: " + ", ".join(missing))
            return

        zero = np.zeros(len(self.joint_names), dtype=np.float64)
        dynamics = self.dm_arm.Dynamics()
        dynamics.configure(self.cfg.dynamics)
        dynamics.update(zero, zero, zero, zero, zero)
        checks = {
            "gravity": np.asarray(dynamics.gravity).shape == (len(self.joint_names),),
            "mass_matrix": np.asarray(dynamics.mass_matrix).shape
            == (len(self.joint_names), len(self.joint_names)),
            "tool_pose": np.asarray(dynamics.tool_pose).shape == (4, 4),
            "tool_jacobian": np.asarray(dynamics.tool_jacobian).shape
            == (6, len(self.joint_names)),
        }
        print(
            f"dm_arm version          : {getattr(self.dm_arm, '__version__', 'unknown')}"
        )
        print(f"RobotSession configured : {self.session.configured}")
        print(f"Actuator count          : {len(self.session.actuator_info)}")
        for name, passed in checks.items():
            print(f"{name:<24}: {'PASS' if passed else 'FAIL'}")
        print("该检查只验证绑定和离线 Dynamics，不代表真机已经验证")

    def monitor(self) -> None:
        duration = read_float("监视时长 s，0 表示直到 Ctrl+C: ")
        hz = read_float("显示频率 Hz: ")
        if duration is None or hz is None or duration < 0.0 or hz <= 0.0 or hz > 50.0:
            print("参数无效")
            return
        started = time.monotonic()
        try:
            while duration == 0.0 or time.monotonic() - started < duration:
                snapshot = self.session.snapshot
                if snapshot.valid:
                    print(
                        f"state={enum_name(snapshot.robot_state):<10} dt={snapshot.cycle.dt:.6f} "
                        f"q={vector(snapshot.cycle.joint_state.pos)}"
                    )
                else:
                    print(
                        f"state={enum_name(snapshot.robot_state):<10} error={snapshot.last_error or '<empty>'}"
                    )
                time.sleep(1.0 / hz)
        except KeyboardInterrupt:
            print("\n停止监视")


def import_dm_arm() -> Any:
    try:
        import dm_arm
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "无法导入 dm_arm；请先构建并安装 python/dist/dm_arm-*.whl"
        ) from error
    return dm_arm


def check_only(dm_arm: Any, config: Path) -> int:
    cfg = dm_arm.load_robot_cfg(str(config))
    dynamics = dm_arm.Dynamics()
    dynamics.configure(cfg.dynamics)
    zero = np.zeros(len(cfg.joint_names), dtype=np.float64)
    dynamics.update(zero, zero, zero, zero, zero)
    print(f"dm_arm version       : {getattr(dm_arm, '__version__', 'unknown')}")
    print(f"joint_names          : {list(cfg.joint_names)}")
    print(f"gravity shape        : {np.asarray(dynamics.gravity).shape}")
    print(f"mass_matrix shape    : {np.asarray(dynamics.mass_matrix).shape}")
    print(f"tool_pose shape      : {np.asarray(dynamics.tool_pose).shape}")
    print(f"tool_jacobian shape  : {np.asarray(dynamics.tool_jacobian).shape}")
    print("check-only 完成；没有连接或写入真实硬件")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="DM-Arm pybind11 Python terminal")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()

    config = Path(args.config).expanduser().resolve()
    if not config.is_file():
        print(f"配置文件不存在: {config}", file=sys.stderr)
        return 2

    try:
        dm_arm = import_dm_arm()
        if args.check_only:
            return check_only(dm_arm, config)
        if not args.allow_hardware:
            print(
                "拒绝启动真机；请显式传入 --allow-hardware，或先使用 --check-only",
                file=sys.stderr,
            )
            return 2
        return Terminal(dm_arm, config).run()
    except Exception as error:
        print(f"启动失败: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

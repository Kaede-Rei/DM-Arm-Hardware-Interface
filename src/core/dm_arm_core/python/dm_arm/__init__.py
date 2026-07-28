"""DM-Arm Python 公共接口"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ._dm_arm import ActuatorInfo
from ._dm_arm import ActuatorMitCmd
from ._dm_arm import ActuatorState
from ._dm_arm import DamiaoActuatorCfg
from ._dm_arm import DamiaoBusCfg
from ._dm_arm import ConfigErr
from ._dm_arm import ConfigErrInfo
from ._dm_arm import DmArmError
from ._dm_arm import Dynamics
from ._dm_arm import DynamicsCfg
from ._dm_arm import DynamicsErr
from ._dm_arm import DynamicsInfo
from ._dm_arm import DynamicsState
from ._dm_arm import FaultCompliantRecoveryCfg
from ._dm_arm import FaultHoldMode
from ._dm_arm import FaultRecoveryCfg
from ._dm_arm import JointActuatorMapCfg
from ._dm_arm import JointActuatorMapper
from ._dm_arm import JointActuatorMapErr
from ._dm_arm import JointCtrlCmd
from ._dm_arm import JointCtrller
from ._dm_arm import JointCtrllerCfg
from ._dm_arm import JointCtrllerErr
from ._dm_arm import JointCtrllerState
from ._dm_arm import JointImpedanceGains
from ._dm_arm import JointImpedanceMode
from ._dm_arm import JointLimitCfg
from ._dm_arm import JointPosCmd
from ._dm_arm import JointPosVelCmd
from ._dm_arm import JointPosVelTorCmd
from ._dm_arm import JointState
from ._dm_arm import ModelFeedforwardErr
from ._dm_arm import ModelFeedforwardMode
from ._dm_arm import MotorBusErr
from ._dm_arm import RobotCfg
from ._dm_arm import RobotErr
from ._dm_arm import RobotFault
from ._dm_arm import RobotCycleOutput
from ._dm_arm import RobotSessionSnapshot
from ._dm_arm import RobotState
from ._dm_arm import RuntimeCfg
from ._dm_arm import ShutdownCfg
from ._dm_arm import Safety
from ._dm_arm import SafetyAction
from ._dm_arm import SafetyCfg
from ._dm_arm import SafetyErr
from ._dm_arm import SafetyFault
from ._dm_arm import _RobotSession
from ._dm_arm import __version__
from ._dm_arm import load_robot_cfg
from ._dm_arm import validate_robot_cfg
from ._dm_arm import validate_robot_core_cfg


class RobotSession:
    """使用 C++ 工作线程维护控制周期的会话

    Python 线程只提交阻抗模式、重力比例和位置目标并读取快照；200 Hz 控制周期始终在 C++ 工作线程中执行
    ``runtime.write_enabled=true`` 时使用真机 Damiao 后端；false 时使用离线 mock 后端
    """

    def __init__(self, config_file: str | Path) -> None:
        """加载配置并构建底层会话；该阶段不会激活机械臂"""
        self._cfg = load_robot_cfg(str(config_file))
        self._session = _RobotSession()
        self._session.configure(str(config_file))

    def start(self) -> None:
        """激活后端并启动 C++ 控制线程"""
        self._session.start()

    def stop(self) -> None:
        """停止 C++ 控制线程并在 ACTIVE 状态下安全失能"""
        self._session.stop()

    def reset_fault(self) -> None:
        """兼容旧接口，内部执行 clear_fault()"""
        self._session.reset_fault()

    def clear_fault(self) -> None:
        """清除 Robot FAULT 并进入 ACTIVE + RIGID_HOLD"""
        self._session.clear_fault()

    def enter_fault_compliant_recovery(self) -> None:
        """人工请求进入 FAULT 受限柔性恢复"""
        self._session.enter_fault_compliant_recovery()

    def return_to_fault_rigid_hold(self) -> None:
        """返回 FAULT 刚性保持"""
        self._session.return_to_fault_rigid_hold()

    def set_impedance_mode(self, mode: JointImpedanceMode) -> None:
        """提交阻抗模式切换请求；请求由 C++ 工作线程串行应用"""
        self._session.set_impedance_mode(mode)

    def set_model_feedforward_mode(self, mode: ModelFeedforwardMode) -> None:
        """在 INACTIVE 状态下设置模型前馈模式"""
        self._session.set_model_feedforward_mode(mode)

    def set_gravity_scale(self, gravity_scale: np.ndarray) -> None:
        """设置六轴重力补偿比例；运行期间由 C++ 工作线程应用"""
        self._session.set_gravity_scale(
            np.ascontiguousarray(gravity_scale, dtype=np.float64)
        )

    def move_to(self, pos: np.ndarray, speed_scale: float = 0.3) -> None:
        """提交六轴绝对位置目标和速度比例"""
        self._session.move_to(
            np.ascontiguousarray(pos, dtype=np.float64), float(speed_scale)
        )

    def hold_current(self) -> None:
        """取消位置目标并请求切换到当前位置刚性保持"""
        self._session.hold_current()

    @property
    def snapshot(self) -> RobotSessionSnapshot:
        """返回最近一次会话快照副本"""
        return self._session.snapshot

    @property
    def state(self) -> RobotState:
        """返回当前 Robot 生命周期状态"""
        return self._session.state

    @property
    def fault_hold_mode(self) -> FaultHoldMode:
        """返回当前 FAULT 保持模式"""
        return self._session.fault_hold_mode

    @property
    def configured(self) -> bool:
        """返回会话是否已经完成配置"""
        return self._session.configured

    @property
    def running(self) -> bool:
        """返回 C++ 控制线程是否正在运行"""
        return self._session.running

    @property
    def config(self) -> RobotCfg:
        """返回当前静态配置副本"""
        return self._session.config

    @property
    def dynamics_info(self) -> DynamicsInfo:
        """返回动力学模型静态信息副本"""
        return self._session.dynamics_info

    @property
    def actuator_info(self) -> list[ActuatorInfo]:
        """返回达妙执行器静态信息副本"""
        return self._session.actuator_info

    def __enter__(self) -> "RobotSession":
        """启动会话并返回当前对象"""
        self.start()
        return self

    def __exit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc: Optional[BaseException],
        traceback: object,
    ) -> None:
        """退出上下文时停止会话并安全失能"""
        self.stop()


__all__ = [
    "ActuatorInfo",
    "ActuatorMitCmd",
    "ActuatorState",
    "DamiaoActuatorCfg",
    "DamiaoBusCfg",
    "ConfigErr",
    "ConfigErrInfo",
    "DmArmError",
    "Dynamics",
    "DynamicsCfg",
    "DynamicsErr",
    "DynamicsInfo",
    "DynamicsState",
    "FaultCompliantRecoveryCfg",
    "FaultHoldMode",
    "FaultRecoveryCfg",
    "JointActuatorMapCfg",
    "JointActuatorMapper",
    "JointActuatorMapErr",
    "JointCtrlCmd",
    "JointCtrller",
    "JointCtrllerCfg",
    "JointCtrllerErr",
    "JointCtrllerState",
    "JointImpedanceGains",
    "JointImpedanceMode",
    "JointLimitCfg",
    "JointPosCmd",
    "JointPosVelCmd",
    "JointPosVelTorCmd",
    "JointState",
    "ModelFeedforwardErr",
    "ModelFeedforwardMode",
    "MotorBusErr",
    "RobotCfg",
    "RobotErr",
    "RobotFault",
    "RobotCycleOutput",
    "RobotSession",
    "RobotSessionSnapshot",
    "RobotState",
    "RuntimeCfg",
    "ShutdownCfg",
    "Safety",
    "SafetyAction",
    "SafetyCfg",
    "SafetyErr",
    "SafetyFault",
    "load_robot_cfg",
    "validate_robot_cfg",
    "validate_robot_core_cfg",
    "__version__",
]

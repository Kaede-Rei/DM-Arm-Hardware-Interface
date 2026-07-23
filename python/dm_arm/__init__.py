"""DM-Arm Python API"""

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
    """C++ worker-thread based hardware session

    The session never runs the 200 Hz control loop in Python; hardware start
    requires an explicit ``allow_hardware=True`` constructor argument
    """

    def __init__(self, config_file: str | Path, *, allow_hardware: bool = False) -> None:
        self._allow_hardware = bool(allow_hardware)
        self._session = _RobotSession()
        self._session.configure(str(config_file))

    def start(self) -> None:
        self._session.start(self._allow_hardware)

    def stop(self) -> None:
        self._session.stop()

    def reset_fault(self) -> None:
        self._session.reset_fault()

    def set_impedance_mode(self, mode: JointImpedanceMode) -> None:
        self._session.set_impedance_mode(mode)

    def set_model_feedforward_mode(self, mode: ModelFeedforwardMode) -> None:
        self._session.set_model_feedforward_mode(mode)

    def set_gravity_scale(self, gravity_scale: np.ndarray) -> None:
        self._session.set_gravity_scale(np.ascontiguousarray(gravity_scale, dtype=np.float64))

    def move_to(self, pos: np.ndarray, speed_scale: float = 0.3) -> None:
        self._session.move_to(np.ascontiguousarray(pos, dtype=np.float64), float(speed_scale))

    def hold_current(self) -> None:
        self._session.hold_current()

    @property
    def snapshot(self) -> RobotSessionSnapshot:
        return self._session.snapshot

    @property
    def state(self) -> RobotState:
        return self._session.state

    @property
    def configured(self) -> bool:
        return self._session.configured

    @property
    def running(self) -> bool:
        return self._session.running

    @property
    def config(self) -> RobotCfg:
        return self._session.config

    @property
    def dynamics_info(self) -> DynamicsInfo:
        return self._session.dynamics_info

    @property
    def actuator_info(self) -> list[ActuatorInfo]:
        return self._session.actuator_info

    def __enter__(self) -> "RobotSession":
        self.start()
        return self

    def __exit__(self, exc_type: Optional[type[BaseException]], exc: Optional[BaseException], traceback: object) -> None:
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

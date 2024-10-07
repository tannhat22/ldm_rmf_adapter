# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

# import paho.mqtt.client as mqtt
# import time
import ruamel.yaml

# import json
import sys
import logging
from enum import IntEnum, Enum

# import glob
# import ssl
import threading

from abc import ABC, abstractmethod
from typing import Any, Callable

import ruamel.yaml.comments
import ruamel.yaml.scalarfloat
from ldm_fleet_msgs.msg import LiftState as LDMLiftState


class RobotStatus(IntEnum):
    # For elevator
    HAS_ENTERED = 1
    HAS_GOT_OFF = 2
    CANCEL_TO_ENTER = 3
    CANCEL_TO_GET_OFF = 4
    KEEP_DOOR_OPEN = 5

    # For door
    START_TO_PASS = 0
    HAS_PASSED = 1
    CANCEL_TO_PASS = 3
    # KEEP_DOOR_OPEN = 5


class ResultCode(IntEnum):
    SUCCESS = 1
    FAIL = 2
    ERROR = 3
    UNDER_EMERGENCY = 5


class DeviceType(Enum):
    ELEVATOR = "elevator"
    DOOR = "door"


class LdmFloorInfo:
    def __init__(
        self,
        floor_name: str,
        has_front_door: bool = True,
        has_rear_door: bool = False,
        # position_x: float = 0.0,
        # position_y: float = 0.0,
        # yaw: float = 0.0,
    ):
        self.floor_name = floor_name
        self.has_front_door = has_front_door
        self.has_rear_door = has_rear_door
        # self.position_x = position_x
        # self.position_y = position_y
        # self.yaw = yaw


class LdmContext(ABC):
    """
    Context class for LDM to support multiple elevators and doors with a single Robot Account.
    """

    _logger: Any

    _device_type: DeviceType

    _context_lock: threading.RLock

    # Event to wait sychronous API response
    # _response_event: threading.Event

    reset_callback: Callable[[], None]

    def __init__(self, device_type: DeviceType, logger=None) -> None:
        if logger is None:
            self._logger = logging.getLogger("ldm_client")
        else:
            self._logger = logger

        self._device_type = device_type

        self._context_lock = threading.RLock()

        # self._response_event = threading.Event()

        self.reset_callback = None

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        pass

    def get_device_type(self) -> DeviceType:
        return self._device_type

    @abstractmethod
    def _msg_callback(self, msg) -> None:
        pass

    def reset(self):
        self._reset()
        if self.reset_callback is not None:
            self.reset_callback()

    @abstractmethod
    def _reset(self):
        pass


class LdmElevatorContext(LdmContext):
    _elevator_id: str
    _floor_list: list[LdmFloorInfo]

    _is_available: bool
    _is_registered: bool
    _current_motion: int

    _current_floor: str
    _current_door: int

    _target_floor: str
    # _target_door: int

    def __init__(self, logger=None) -> None:
        super().__init__(DeviceType.ELEVATOR, logger)

    def initialize(self, config: dict) -> bool:
        self._elevator_id = config.get("ldm_elevator_id", None)
        floor_list_tmp = config.get("ldm_floor_list", None)

        ret = True

        if self._elevator_id == None:
            self._logger.error("[LDM] <ldm_elevator_id> is not specified. Use 0.")
            self._elevator_id = 0
        else:
            self._elevator_id = str(self._elevator_id)

        if type(floor_list_tmp) is not ruamel.yaml.comments.CommentedSeq:
            self._logger.error("[LDM] <ldm_floor_list> is not a list.")
            ret = False

        self._floor_list = []

        for f in floor_list_tmp:
            if (
                isinstance(f, list)
                and len(f) == 3
                and type(f[1]) == bool
                and type(f[2]) == bool
                # and type(f[3]) == ruamel.yaml.scalarfloat.ScalarFloat
                # and type(f[4]) == ruamel.yaml.scalarfloat.ScalarFloat
                # and type(f[5]) == ruamel.yaml.scalarfloat.ScalarFloat
            ):
                self._floor_list.append(LdmFloorInfo(str(f[0]), f[1], f[2]))
            else:
                self._logger.error(
                    "[LDM] ldm_floor_list has format: [level, has_FD, has_BD]."
                )
                return False

        if len(self._floor_list) > 63:
            self._logger.error("[LDM] LDM supports no more than 63 levels of floors.")
            ret = False

        if ret == False:
            return False

        self._is_available = True
        self._is_registered = False
        self._current_motion = 3

        self._current_floor = self._floor_list[0].floor_name
        self._current_door = 0
        self._target_floor = ""
        # self._target_door = 0

        return True

    def _reset(self):
        with self._context_lock:
            self._is_registered = False
            self._current_motion = 3
            self._target_floor = ""
            # self._target_door = 0

    def _msg_callback(self, msg: LDMLiftState) -> None:
        # result = payload.get("result", ResultCode.ERROR.value)
        current_mode = msg.current_mode
        with self._context_lock:
            match current_mode:
                case LDMLiftState.MODE_EMERGENCY:
                    self._is_available = False
                    self.reset()

                case _:
                    self._is_available = True
                    if msg.register_state == LDMLiftState.REGISTER_SIGNED:
                        self._is_registered = True
                    elif msg.register_state == LDMLiftState.REGISTER_RELEASED:
                        self._is_registered = False

                    self._current_floor = msg.current_floor
                    self._current_door = msg.door_state
                    self._current_motion = msg.motion_state


class LdmDoorContext(LdmContext):
    _floor_id: str
    _door_id: str
    _door_type: str

    _is_registered: bool

    _current_door: int
    _current_lock: int

    def __init__(self, logger=None) -> None:
        super().__init__(DeviceType.DOOR, logger)

    def initialize(self, config: dict) -> bool:
        self._floor_id = config.get("ldm_floor_id", None)
        self._door_id = config.get("ldm_door_id", None)
        self._door_type = config.get("ldm_door_type", None)

        ret = True

        if self._floor_id == None:
            self._logger.error("[LDM] <ldm_floor_id> is not specified.")
            ret = False

        if self._door_id == None:
            self._logger.error("[LDM] <ldm_door_id> is not specified")
            ret = False

        if self._door_type == None:
            self._logger.error("[LDM] <ldm_door_type> is not specified")
            ret = False

        if ret == False:
            return False

        self._is_registered = False

        self._current_door = 0
        self._current_lock = 1

        return True

    def _reset(self):
        self._is_registered = False
        self._current_door = 0
        self._current_lock = 1

    def _msg_callback(self, msg) -> None:
        pass
        # result = payload.get("result", ResultCode.ERROR.value)

        # with self._context_lock:
        #     if result == ResultCode.SUCCESS.value:
        #         match api:
        #             case "RegistrationResult":
        #                 if not payload.get("dry_run", False):
        #                     self._is_registered = True
        #             case "ReleaseResult":
        #                 self.reset()
        #             case "DoorStatus":
        #                 if self._door_type == "lock":
        #                     self._current_lock = payload.get("lock", 1)
        #                 else:
        #                     self._current_door = payload.get("door", 0)


class LdmClient:
    _logger: Any

    _context_dict: dict[str, LdmContext]

    # _publish_lock: threading.Lock

    def __init__(self, logger=None) -> None:
        if logger is None:
            self._logger = logging.getLogger("ldm_client")
        else:
            self._logger = logger

        self._context_dict = {}

        # self._publish_lock = threading.Lock()

    def initialize(self, config_file_path: str) -> bool:
        yaml = ruamel.yaml.YAML()
        with open(config_file_path) as file:
            config = yaml.load(file.read())

        elevator_config = config.get("elevators", None)
        if type(elevator_config) is ruamel.yaml.comments.CommentedSeq:
            for ec in elevator_config:
                context = LdmElevatorContext(self._logger)
                if context.initialize(ec):
                    self._context_dict.update({context._elevator_id: context})

        door_config = config.get("doors", None)
        if type(door_config) is ruamel.yaml.comments.CommentedSeq:
            for dc in door_config:
                context = LdmDoorContext(self._logger)
                if context.initialize(dc):
                    self._context_dict.update({context._door_id: context})

        return True

    def get_contexts(self) -> dict[str, LdmContext]:
        return self._context_dict


# Main routine
if __name__ == "__main__":

    # if len(sys.argv) < 2:
    #     logging.error(
    #         "Please specify yaml config file as the 1 st command line parameter."
    #     )
    #     sys.exit(1)

    ldm_context = LdmClient()

    if not ldm_context.initialize(
        config_file_path="/home/tannhat/rmf_ws/src/ldm_rmf_adapter/config.yaml"
    ):
        logging.error("LdmClient initialization error")
        sys.exit(1)

    context_dict = ldm_context.get_contexts()
    context = None
    for c in context_dict.values():
        if c.get_device_type() is DeviceType.ELEVATOR:
            context = c
            print(f"elevator_id: {c._elevator_id}")
            # break

    if context is None:
        logging.error("No Elevator Context in the config file")
        sys.exit(1)

    ldm_context._logger.addHandler(logging.StreamHandler(sys.stdout))
    ldm_context._logger.setLevel(logging.DEBUG)

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
        self, floor_name: str, has_front_door: bool = True, has_rear_door: bool = False
    ):
        self.floor_name = floor_name
        self.has_front_door = has_front_door
        self.has_rear_door = has_rear_door


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
    _motion_state: int

    _current_floor: str
    _current_door: int

    _target_floor: str
    _target_door: int

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
            ):
                self._floor_list.append(LdmFloorInfo(str(f[0]), f[1], f[2]))
            else:
                self._floor_list.append(LdmFloorInfo(str(f), True, False))

        if len(self._floor_list) > 63:
            self._logger.error("[LDM] LDM supports no more than 63 levels of floors.")
            ret = False

        if ret == False:
            return False

        self._is_available = True
        self._is_registered = False
        self._motion_state = 0

        self._current_floor = self._floor_list[0].floor_name
        self._current_door = 0
        self._target_floor = ""
        self._target_door = 0

        return True

    def _reset(self):
        with self._context_lock:
            self._is_registered = False
            self._motion_state = 0
            self._target_floor = ""
            self._target_door = 0

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

    # def start(self):
    #     self._mqtt_client.loop_start()

    # def stop(self):
    #     self._mqtt_client.loop_stop()

    # def _on_connect(
    #     self, client: mqtt.Client, userdata: "LdmClient", flags: dict, rc: int
    # ) -> None:
    #     self._logger.info(
    #         f"[LDM] Connected to {self._mqtt_server} with client_id {self._robot_id}, result code {rc}"
    #     )

    #     self._mqtt_client.subscribe(
    #         [
    #             (f"/ldm/{self._bldg_id}/+/+/RegistrationResult/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/CallElevatorResult/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/ElevatorStatus/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/OpenDoorResult/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/DoorStatus/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/RobotStatusResult/{self._robot_id}", 1),
    #             (f"/ldm/{self._bldg_id}/+/+/ReleaseResult/{self._robot_id}", 1),
    #         ]
    #     )

    # def _on_disconnect(
    #     self, client: mqtt.Client, userdata: "LdmClient", rc: int
    # ) -> None:
    #     self._logger.info(
    #         f"[LDM] Disconnected from {self._mqtt_server} with client_id {self._robot_id}, result code "
    #         + str(rc)
    #     )

    # def _on_message(
    #     self, client: mqtt.Client, userdata: "LdmClient", msg: mqtt.MQTTMessage
    # ):

    #     try:
    #         payload_kv = json.loads(msg.payload)
    #     except json.JSONDecodeError as e:
    #         self._logger.debug(f"[LDM] Skip message with non-JSON payload: {e}")
    #         return

    #     if type(payload_kv) != dict:
    #         self._logger.error(
    #             f"[LDM] MQTT Payload format error: {msg.topic}, {msg.payload}"
    #         )
    #         return

    #     self._msg_callback(msg.topic, payload_kv)

    # def _msg_callback(self, topic: str, payload_kv: dict) -> None:
    #     token = topic.split("/")
    #     topic_prefix = "/".join(token[0:5])
    #     api = token[-2]

    #     context = self._context_dict.get(topic_prefix, None)
    #     if context is not None:
    #         context._msg_callback(api, payload_kv)
    #         self._logger.debug(f"[LDM] Received: {topic}, {payload_kv}")

    #         if not api in ["ElevatorStatus", "DoorStatus"]:
    #             context._response_event.set()

    #     else:
    #         self._logger.warning(f"[LDM] No relevant context for {topic}, {payload_kv}")

    # def _publish(
    #     self,
    #     context: LdmContext,
    #     api: str,
    #     payload: dict,
    #     timeout_sec: float = 0,
    #     wait_response: bool = True,
    # ) -> bool:
    #     topic = f"{context._topic_prefix}/{api}/{self._robot_id}"
    #     payload.update(
    #         {
    #             "robot_id": self._robot_id,
    #             "timestamp": int(time.time() * 1000) / 1000,
    #         }
    #     )
    #     json_payload = json.dumps(payload)

    #     need_to_sync = (0 < timeout_sec) and (
    #         not api in ["RequestElevatorStatus", "RequestDoorStatus"]
    #     )

    #     if need_to_sync:
    #         context._response_event.clear()
    #         qos = 1
    #     else:
    #         qos = 0

    #     with self._publish_lock:
    #         pub_info = self._mqtt_client.publish(topic, json_payload, qos=qos)

    #     if need_to_sync:
    #         # Wait for completion of publish()
    #         start_time = time.time()
    #         while True:
    #             try:
    #                 # When the state is disconnection, wait_for_publish() will return soon with Exception
    #                 pub_info.wait_for_publish(timeout=0.2)

    #             except Exception:
    #                 time.sleep(0.2)
    #                 continue

    #             if pub_info.is_published():
    #                 # Elapsed time between PUB and PUBACK
    #                 self._logger.debug(
    #                     f"[LDM] Published ({time.time()-start_time:.03f}): {topic}, {json_payload}"
    #                 )  # noqa

    #                 if wait_response:
    #                     # Wait for receiving response from LDM
    #                     start_time = time.time()
    #                     ret = context._response_event.wait(timeout_sec)
    #                     if ret:
    #                         # Elapsed time between LDM's request and response
    #                         self._logger.debug(
    #                             f"[LDM] Response received ({time.time()-start_time:.03f}): {api}"
    #                         )  # noqa
    #                     else:
    #                         self._logger.debug(
    #                             f"[LDM] Response timeout ({time.time()-start_time:.03f}): {api}"
    #                         )  # noqa
    #                     return ret
    #                 else:
    #                     return True

    #             elif start_time + timeout_sec < time.time():
    #                 self._logger.debug(
    #                     f"[LDM] Publish timeout ({time.time()-start_time:.03f}): {topic}, {json_payload}"
    #                 )  # noqa
    #                 return False

    #     return True


# def do_registration(self, context: LdmContext, dry_run: bool = False) -> bool:
#     if dry_run:
#         payload = {"dry_run": True}
#     else:
#         payload = {}

#     return self._publish(context, "Registration", payload, 180)

# def do_release(self, context: LdmContext, wait_response: bool = True) -> bool:
#     return self._publish(context, "Release", {}, 20, wait_response)

# def do_robot_status(
#     self, context: LdmContext, robot_status: RobotStatus, wait_response: bool = True
# ) -> bool:
#     return self._publish(
#         context, "RobotStatus", {"state": robot_status.value}, 20, wait_response
#     )

# def do_call_elevator(
#     self,
#     context: LdmElevatorContext,
#     origination: str,
#     destination: str,
#     origination_door: int = 0,
#     destination_door: int = 0,
#     direction: int = 0,
# ) -> bool:

#     if context._target_floor == "":
#         context._target_floor = origination
#         context._target_door = origination_door
#     else:
#         context._target_floor = destination
#         context._target_door = destination_door

#     payload = {"direction": direction}

#     if origination is not None:
#         payload.update(
#             {"origination": origination, "origination_door": origination_door}
#         )

#     if destination is not None:
#         payload.update(
#             {"destination": destination, "destination_door": destination_door}
#         )

#     return self._publish(context, "CallElevator", payload, 180)

# def do_request_elevator_status(self, context: LdmElevatorContext) -> bool:
#     return self._publish(context, "RequestElevatorStatus", {})

# def do_open_door(self, context: LdmDoorContext, direction: int = None) -> bool:
#     if context._device_type == "flap" and direction is not None:
#         return self._publish(context, "OpenDoor", {"direciton": direction}, 20)
#     else:
#         return self._publish(context, "OpenDoor", {}, 20)

# def do_request_door_status(self, context: LdmDoorContext) -> bool:
#     return self._publish(context, "RequestDoorStatus", {})

# Main routine
if __name__ == "__main__":

    # if len(sys.argv) < 2:
    #     logging.error(
    #         "Please specify yaml config file as the 1 st command line parameter."
    #     )
    #     sys.exit(1)

    ldm_context = LdmClient()

    if not ldm_context.initialize(
        config_file_path="/home/tannhat/rmf_ws/src/ldm_rmf_adapter/ldm_config/server_config_simulator.yaml"
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

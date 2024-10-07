# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy.wait_for_message
from rmf_lift_msgs.msg import LiftState, LiftRequest
from rmf_door_msgs.msg import DoorState, DoorRequest, DoorMode
from ldm_fleet_msgs.msg import (
    FleetLiftState,
    LiftRequest as LDMLiftRequest,
    RegisterRequest,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.publisher import Publisher

from . import ldm_context
import threading
from typing import Any
from abc import ABC, abstractmethod
import time


class RmfContext(ABC):
    _logger: Any

    _ldm_context: ldm_context.LdmContext

    _is_occupied: bool
    _occupant_id: str
    _lock: threading.Lock

    # buffer to neglect mutiple same LiftRequest
    _destination_floor: str

    def __init__(self, ldm_context: ldm_context.LdmContext, logger=None) -> None:
        self._logger = logger

        self._ldm_context = ldm_context
        self._ldm_context.reset_callback = self.reset

        self._is_occupied = False
        self._occupant_id = ""
        self._lock = threading.Lock()

        self._destination_floor = ""

    def set_occupant(self, occupant_id: str) -> bool:
        with self._lock:
            if self._is_occupied and self._occupant_id != occupant_id:
                return False

            self._is_occupied = True
            self._occupant_id = occupant_id
            return True

    def reset(self) -> None:
        with self._lock:
            self._is_occupied = False
            self._occupant_id = ""
            self._destination_floor = ""

    def get_occupant(self) -> str:
        return self._occupant_id

    def set_destination_floor(self, destination_floor: str) -> None:
        with self._lock:
            self._destination_floor = destination_floor

    def get_destination_floor(self) -> str:
        return self._destination_floor

    @abstractmethod
    def get_status(self) -> LiftState | DoorState:
        pass


class RmfLiftContext(RmfContext):
    _ldm_context: ldm_context.LdmElevatorContext
    _rmf_floor_list: list[str]

    def __init__(
        self, ldm_context: ldm_context.LdmElevatorContext, logger=None
    ) -> None:
        super().__init__(ldm_context, logger)

        # convolve door direction into floor_name
        self._rmf_floor_list = []
        for f in self._ldm_context._floor_list:
            if f.has_front_door:
                self._rmf_floor_list.append(f.floor_name)
            if f.has_rear_door:
                self._rmf_floor_list.append(f"{f.floor_name}_r")

    def get_status(self) -> LiftState:
        lift_state = LiftState()
        lift_state.lift_name = self._ldm_context._elevator_id
        lift_state.available_floors = [
            x.floor_name for x in self._ldm_context._floor_list
        ]

        # encode door direction to floor_name
        match self._ldm_context._current_door:
            case 0:
                lift_state.door_state = LiftState.DOOR_CLOSED
            case 2:
                lift_state.door_state = LiftState.DOOR_OPEN
            case _:
                lift_state.door_state = LiftState.DOOR_MOVING

        # if self._ldm_context._target_door == 2:
        #     lift_state.destination_floor = (
        #         f"{self._ldm_context._target_floor}_r"  # noqa
        #     )
        # else:
        #     lift_state.destination_floor = self._ldm_context._target_floor
        lift_state.current_floor = self._ldm_context._current_floor
        lift_state.destination_floor = self._ldm_context._target_floor
        lift_state.motion_state = self._ldm_context._current_motion

        # if (
        #     lift_state.current_floor == lift_state.destination_floor
        #     and lift_state.door_state == LiftState.DOOR_OPEN
        # ):
        #     lift_state.motion_state = LiftState.MOTION_STOPPED
        # else:
        #     lift_state.motion_state = LiftState.MOTION_UNKNOWN

        lift_state.available_modes = [
            LiftState.MODE_AGV,
            LiftState.MODE_OFFLINE,
            LiftState.MODE_EMERGENCY,
        ]

        if self._ldm_context._is_available:
            lift_state.current_mode = LiftState.MODE_AGV
        else:
            lift_state.current_mode = LiftState.MODE_EMERGENCY

        # lift_state.current_mode will be overwritten with LiftState.MODE_OFFLINE if _ldm_client is not connected.
        # lift_state.lift_time shall be set with rclpy.Time

        lift_state.session_id = self._occupant_id

        return lift_state


class RmfDoorContext(RmfContext):
    _ldm_context: ldm_context.LdmDoorContext

    def __init__(self, ldm_context: ldm_context.LdmDoorContext, logger=None) -> None:
        super().__init__(ldm_context, logger)

    def get_status(self) -> DoorState:
        door_state = DoorState()
        door_state.door_name = self._ldm_context._door_id

        if self._ldm_context._current_door == 0:
            door_state.current_mode = DoorMode(value=DoorMode.MODE_CLOSED)
        else:
            door_state.current_mode = DoorMode(value=DoorMode.MODE_OPEN)

        # door_state.current_mode will be overwritten with DoorMode.MODE_OFFLINE if _ldm_client is not connected.
        # door_state.door_time shall be set with rclpy.Time

        return door_state


class LdmRmfAdapter(Node):
    _ldm_client: ldm_context.LdmClient

    _lift_context_dict: dict[str, RmfLiftContext]
    _door_context_dict: dict[str, RmfDoorContext]

    _lift_state_pub: Publisher
    _door_state_pub: Publisher

    def __init__(self) -> None:
        super().__init__("LdmRmfAdapter")

        fleet_cb_group = MutuallyExclusiveCallbackGroup()

        # Parameters used in this Node
        self.declare_parameter(
            "ldm_server_config",
            "",
            ParameterDescriptor(
                description="Path to server_config.yaml, which includes a single elevator setting."
            ),
        )
        self.declare_parameter(
            "timeout",
            10.0,
            ParameterDescriptor(description="Time out for register request"),
        )

        # Initializing LdmClient
        # ros2 run ldm_rmf_lift_adapter ldm_rmf_lift_adapter --ros-args -p "ldm_server_config:=<path to server_config.yaml>" -p "ldm_cert_dir:=<path to cert dir including pem files>"
        server_config_file = self.get_parameter("ldm_server_config").value
        self.timeout = self.get_parameter("timeout").value

        self.get_logger().info(f'Config_file: "{server_config_file}"')
        self.get_logger().info(f"timeout: {self.timeout}")

        self._ldm_client = ldm_context.LdmClient(self.get_logger())
        self._ldm_client.initialize(server_config_file)

        ldm_context_dict = self._ldm_client.get_contexts()

        self._lift_context_dict = {}
        self._door_context_dict = {}
        self.current_cmd_id = 0

        for name, l_context in ldm_context_dict.items():
            match l_context.get_device_type():
                case ldm_context.DeviceType.ELEVATOR:
                    self._lift_context_dict.update(
                        {name: RmfLiftContext(l_context, self.get_logger())}
                    )
                case ldm_context.DeviceType.DOOR:
                    self._door_context_dict.update(
                        {name: RmfDoorContext(l_context, self.get_logger())}
                    )

        # self._ldm_client.start()

        # Initializing ROS2 messaging
        state_qos_profile = QoSProfile(
            depth=15,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
        )

        request_qos_profile = QoSProfile(
            depth=0,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # To publish lift status to RMF
        self._lift_state_pub = self.create_publisher(
            LiftState, "lift_states", qos_profile=state_qos_profile
        )

        # To publish door status to RMF
        # self._door_state_pub = self.create_publisher(
        #     DoorState, "door_states", qos_profile=state_qos_profile
        # )

        # To publish lift request to Fleet Lift Server
        self._ldm_lift_request_pub = self.create_publisher(
            LDMLiftRequest, "lift_ldm_requests", qos_profile=state_qos_profile
        )

        # To publish register request to Fleet Lift Server
        self._ldm_register_request_pub = self.create_publisher(
            RegisterRequest, "register_ldm_requests", qos_profile=state_qos_profile
        )

        # Subscribe lift requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_lifts.html
        self._lift_request_sub = self.create_subscription(
            LiftRequest,
            "adapter_lift_requests",
            self._lift_request_callback,
            qos_profile=request_qos_profile,
        )

        # Subscribe door requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_doors.html
        # self._door_request_sub = self.create_subscription(
        #     DoorRequest,
        #     "adapter_door_requests",
        #     self._door_request_callback,
        #     qos_profile=request_qos_profile,
        # )

        # Subscribe fleet lift state from Fleet Lift Server
        self._fleet_lift_state_sub = self.create_subscription(
            FleetLiftState,
            "fleet_ldm_state",
            self._fleet_lift_state_callback,
            100,
            callback_group=fleet_cb_group,
        )

        # To publish all LiftState and DoorState every second.
        self._pub_rmf_state_timer = self.create_timer(1.0, self._publish_rmf_states)

    def next_cmd_id(self):
        self.current_cmd_id = self.current_cmd_id + 1
        return self.current_cmd_id

    def robot_name_analyst(self, requester_id: str) -> None | list[str]:
        requester_analyst = requester_id.split("/")
        if (
            len(requester_analyst) != 2
            or requester_analyst[0] == ""
            or requester_analyst[1] == ""
        ):
            self.get_logger().error(
                f"format of requester_id: '{requester_id}' incorrect!"
            )
            return None
        return requester_analyst

    def _publish_rmf_states(self):
        current_time = self.get_clock().now().to_msg()
        # is_connected = self._ldm_client._mqtt_client.is_connected()

        for rl_context in self._lift_context_dict.values():
            lift_state = rl_context.get_status()

            # if not is_connected:
            #     lift_state.current_mode = LiftState.MODE_OFFLINE
            lift_state.lift_time = current_time

            self._lift_state_pub.publish(lift_state)

        # for rd_context in self._door_context_dict.values():
        #     door_state = rd_context.get_status()

        #     # if not is_connected:
        #     #     door_state.current_mode = DoorMode(value=DoorMode.MODE_OFFLINE)
        #     door_state.door_time = current_time

        #     self._door_state_pub.publish(door_state)

    def _lift_request_callback(self, msg: LiftRequest) -> None:
        rl_context = self._lift_context_dict.get(msg.lift_name, None)
        if rl_context is None:
            return

        # session_id means the sender Node name
        if rl_context.get_occupant() == "":
            rl_context.set_occupant(msg.session_id)

        elif rl_context.get_occupant() != msg.session_id:
            self.get_logger().warning(
                f"[{msg.lift_name}] session_id mismatch: session is owned by {rl_context.get_occupant()} but requested from {msg.session_id}"
            )
            return

        match msg.request_type:
            case LiftRequest.REQUEST_END_SESSION:
                self.reset_lift(rl_context)
                self.get_logger().info(f"[{msg.lift_name}] Release")

            case LiftRequest.REQUEST_AGV_MODE:
                """
                When the robot is at the lift hall, LiftRequest.destination_floor shall be set in '<origination>' or '<origination>:<destination>'.
                Especially when the backend lift API behind LCI requires <destination> information, '<origination>:<destination>' format is mandatory for the 1st CallElevator.

                After the robot entered the cage, LiftRequest.destination_floor shall be set in '<destination>' or '<destination>:<destination>'.

                Example 1:
                - At the lift hall: '1F'
                - In the cage: '3F'

                Example 2:
                - At the lift hall: '1F:3F'
                - In the cage: '3F'

                Example 3:
                - At the lift hall: '1F:3F'
                - In the cage: '3F:3F'
                """
                target_floor_list = msg.destination_floor.split(":")

                if len(target_floor_list) not in [1, 2]:
                    self.get_logger().error(
                        f"[{msg.lift_name}] Format error of destination_floor ({msg.destination_floor}). It must be <origination> for the 1st request, <destination> for the 2nd request or <origination>:<destination>"
                    )
                    return

                for tf in target_floor_list:
                    if tf not in rl_context._rmf_floor_list:
                        self.get_logger().error(
                            f"[{msg.lift_name}] Invalid floor name {tf}. It is not in floor_list)"
                        )
                        return

                if (
                    rl_context.get_destination_floor() == msg.destination_floor
                    and rl_context.get_occupant() != "rmf_api_server"
                ):
                    # Because RMF sends same LiftRequest in 1 Hz, ldm-rmf-adapter has to neglect those redundant requests by checking whether LiftRequest.destination_floor changes.
                    return
                rl_context.set_destination_floor(msg.destination_floor)

                if not rl_context._ldm_context._is_registered:
                    res = self.do_registration(rl_context._ldm_context)
                    if not res or not rl_context._ldm_context._is_registered:
                        self.get_logger().warning(
                            f"[{msg.lift_name}] Registration failed: {res}"
                        )
                        return
                    self.get_logger().info(f"[{msg.lift_name}] Registration")

                origination: str = None
                destination: str = None

                if rl_context._ldm_context._target_floor == "":
                    # 1st CallElevator when the robot may be out of the cage.
                    if len(target_floor_list) == 2:
                        origination = target_floor_list[0]
                        destination = target_floor_list[1]
                        self.get_logger().info(
                            f"[{msg.lift_name}] 1st CallElevator: {origination} to {destination}"
                        )

                    else:
                        origination = target_floor_list[0]
                        self.get_logger().info(
                            f"[{msg.lift_name}] 1st CallElevator: {origination}"
                        )

                else:
                    # 2nd CallElevator when the robot may be in the cage.
                    if len(target_floor_list) == 2:
                        destination = target_floor_list[1]
                    else:
                        destination = target_floor_list[0]

                    self.get_logger().info(
                        f"[{msg.lift_name}] 2nd CallElevator: {destination}"
                    )

                # decode door direction from floor_name
                destination_door = msg.door_state
                # if origination is not None and origination.endswith("_r"):
                #     origination = origination[0:-2]

                # if destination is not None and destination.endswith("_r"):
                #     destination = destination[0:-2]

                res = self.do_call_elevator(
                    rl_context._ldm_context,
                    origination,
                    destination,
                    destination_door,
                )
                if not res:
                    self.get_logger().error(f"[{msg.lift_name}] CallElevator failed")
                    self.reset_lift(rl_context)
                    self.get_logger().info(f"[{msg.lift_name}] Release")

            case _:
                self.get_logger().warning(
                    f"[{msg.lift_name}] request_type {msg.request_type} is not supported."
                )

    def reset_lift(self, rl_context: RmfLiftContext):
        # if rl_context._ldm_context._is_registered:
        #     # RMF Lift API does not have information where the robot is. Then, HAS_GOT_OFF used for LCI to reset.
        #     # For resetting, no need to receive the corresponding response from LCI
        #     self._ldm_client.do_robot_status(
        #         rl_context._ldm_context, ldm_context.RobotStatus.HAS_GOT_OFF, False
        #     )
        #     time.sleep(1)

        # For resetting, no need to receive the corresponding response from LCI
        self.do_release(rl_context._ldm_context)
        rl_context._ldm_context.reset()

    def _door_request_callback(self, msg: DoorRequest) -> None:
        rd_context = self._door_context_dict.get(msg.door_name, None)
        if rd_context is None:
            return

        # requester_id means the sender Node name
        if rd_context.get_occupant() == "":
            rd_context.set_occupant(msg.requester_id)

        elif rd_context.get_occupant() != msg.requester_id:
            self.get_logger().warning(
                f"[{msg.door_name}] requester_id mismatch: session is owned by {rd_context.get_occupant()} but requested from {msg.requester_id}"
            )
            return

        match msg.requested_mode.value:
            case DoorMode.MODE_CLOSED:
                self.reset_door(rd_context)
                self.get_logger().info(f"[{msg.door_name}] Release")

            case DoorMode.MODE_OPEN:
                if not rd_context._ldm_context._is_registered:
                    res = self.do_registration(rd_context._ldm_context)
                    if not res or not rd_context._ldm_context._is_registered:
                        self.get_logger().warning(
                            f"[{msg.door_name}] Registration failed: {res}"
                        )
                        return
                    self.get_logger().info(f"[{msg.door_name}] Registration")

                res = self.do_open_door(rd_context._ldm_context)
                if not res:
                    self.get_logger().error(f"[{msg.door_name}] OpenDoor failed")
                    self.reset_door(rd_context)
                    return
                self.get_logger().info(f"[{msg.door_name}] OpenDoor")

    def reset_door(self, rd_context: RmfDoorContext):
        # For resetting, no need to receive the corresponding response from LCI
        self.do_release(rd_context._ldm_context)
        rd_context.reset()

    def _fleet_lift_state_callback(self, lift_states: FleetLiftState):
        lifts = lift_states.lifts
        for l in lifts:
            rl_context = self._lift_context_dict.get(l.lift_name, None)
            if rl_context is not None:
                rl_context._ldm_context._msg_callback(msg=l)

    def do_registration(
        self, context: ldm_context.LdmElevatorContext | ldm_context.LdmDoorContext
    ):
        msg = RegisterRequest()
        match context.get_device_type():
            case ldm_context.DeviceType.ELEVATOR:
                msg.device_name = context._elevator_id
                msg.device_type = RegisterRequest.DEVICE_LIFT
                msg.register_mode = RegisterRequest.REGISTER_SIGNED
                msg.request_id = str(self.next_cmd_id())
                self._ldm_register_request_pub.publish(msg)

                startTime = self.get_clock().now()
                while True:
                    durationTime = (self.get_clock().now() - startTime).nanoseconds * (
                        10 ** (-9)
                    )
                    if durationTime < self.timeout:
                        with context._context_lock:
                            if context._is_registered:
                                return True
                        time.sleep(0.5)
                        # is_success, lift_states = (
                        #     rclpy.wait_for_message.wait_for_message(
                        #         FleetLiftState,
                        #         self,
                        #         "/fleet_lift_state",
                        #         time_to_wait=5.0,
                        #     )
                        # )
                        # if is_success:
                        #     lifts = lift_states.lifts
                        #     for l in lifts:
                        #         if l.lift_name == context._elevator_id:
                        #             if (
                        #                 l.register_state
                        #                 == LDMLiftState.REGISTER_RELEASED
                        #             ):
                        #                 return True
                        #             break
                        # else:
                        #     self.get_logger().error(
                        #         "No receive messages from /fleet_lift_state topic!"
                        #     )
                        #     return False
                    else:
                        self.get_logger().error(f"Timeout wait for register request!")
                        return False

            case ldm_context.DeviceType.DOOR:
                msg.device_name = context._door_id
                msg.device_type = RegisterRequest.DEVICE_DOOR
                msg.register_mode = RegisterRequest.REGISTER_SIGNED
                msg.request_id = str(self.next_cmd_id())
                self._ldm_register_request_pub.publish(msg)
                return True

    def do_release(self, context: ldm_context.LdmContext):
        msg = RegisterRequest()
        match context.get_device_type():
            case ldm_context.DeviceType.ELEVATOR:
                msg.device_name = context._elevator_id
                msg.device_type = RegisterRequest.DEVICE_LIFT
            case ldm_context.DeviceType.DOOR:
                msg.device_name = context._door_id
                msg.device_type = RegisterRequest.DEVICE_DOOR
        msg.register_mode = RegisterRequest.REGISTER_RELEASED
        msg.request_id = str(self.next_cmd_id())
        self._ldm_register_request_pub.publish(msg)

    def do_call_elevator(
        self,
        context: ldm_context.LdmElevatorContext,
        origination: str,
        destination: str,
        destination_door: int = 0,
    ) -> bool:
        if context._target_floor == "":
            context._target_floor = origination
        else:
            context._target_floor = destination

        msg = LDMLiftRequest()
        msg.lift_name = context._elevator_id
        msg.request_type = LDMLiftRequest.REQUEST_AGV_MODE
        msg.destination_floor = context._target_floor
        msg.door_state = destination_door
        msg.request_id = str(self.next_cmd_id())
        self._ldm_lift_request_pub.publish(msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    ldm_rmf_lift_adapter = LdmRmfAdapter()
    executor = MultiThreadedExecutor()
    executor.add_node(node=ldm_rmf_lift_adapter)
    # rclpy.spin(ldm_rmf_lift_adapter)
    # ldm_rmf_lift_adapter.destroy_node()
    # rclpy.shutdown()

    try:
        ldm_rmf_lift_adapter.get_logger().info(
            "Beginning client, shut down with CTRL-C"
        )
        executor.spin()
    except KeyboardInterrupt:
        ldm_rmf_lift_adapter.get_logger().info("Keyboard interrupt, shutting down.\n")
    ldm_rmf_lift_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

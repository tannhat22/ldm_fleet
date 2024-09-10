import time
import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node
from .mcprotocol.type1e import Type1E

from ldm_fleet_msgs.srv import Lift


class LiftService(Node):
    def __init__(self, config_yaml):
        super().__init__("lift_service")
        self.config_yaml = config_yaml

        # Params:
        # Cấu hình các thông số quan trọng:
        # self.declare_parameter("PLC_IP_address", "192.168.1.1")
        # self.declare_parameter("PLC_Port_address", 8501)
        # self.declare_parameter("timeout", 10.0)
        # self.declare_parameter("frequency", 5.0)

        # self.IP_addres_PLC = self.get_parameter("PLC_IP_address").value
        # self.port_addres_PLC = self.get_parameter("PLC_Port_address").value
        # self.timeout = self.get_parameter("timeout").value
        # self.frequency = self.get_parameter("frequency").value

        self.IP_addres_PLC = self.config_yaml["ip"]
        self.port_addres_PLC = self.config_yaml["port"]
        self.timeout = self.config_yaml["time_out"]
        self.frequency = self.config_yaml["frequency"]

        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"timeout: {self.timeout}")
        self.get_logger().info(f"frequency: {self.frequency}")

        self.pyPLC = Type1E("F")
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        # ------ Address all device -------:
        # Bits:
        self.door_trigger_bit = self.config_yaml["bit"]["door_trigger"]
        self.lift_trigger_bit = self.config_yaml["bit"]["lift_trigger"]

        ## Registers:
        # Lift data
        # Door_state: 0 (1: closed, 2: open, 3: moving)
        # Current_Floor: 1 (1: level 1, 2: level 2...)
        # Motion_state: 2 (1: STOP, 2: UP, 3: DOWN, 4: UNKNOWN)
        # Operation_mode: 3 (1: Manual, 2: Auto)
        # Current_state: 4 (1: Available, 2: Error)
        # Cabin_state: 5 (1: Full, 2: Empty)
        self.lift_data_reg = self.config_yaml["register"]["lift_data"]

        # 1: Open, 2: Closed
        self.door_control_reg = self.config_yaml["register"]["door_control"]
        # 1: level 1, 2: level 2 ...
        self.destination_floor_reg = self.config_yaml["register"]["destination_floor"]
        self.request_type_reg = self.config_yaml["register"]["request_type"]

        # Services server:
        self.srv = self.create_service(Lift, "lift_server", self.lift_callback)

        # Publishers:
        self.get_logger().info("is running!!!!!!!!!!")

    def lift_callback(self, request: Lift.Request, response: Lift.Response):
        try:
            self.get_logger().info(
                f"Get request LIFT:\n"
                f"  request_types: {request.request_types}\n"
                f"  des_floor: {request.destination_floor}\n"
                f"  door_state: {request.door_state}"
            )
            try:
                numFloor = self.config_yaml["floors"][request.destination_floor]
            except Exception as e:
                print(e)
                self.get_logger().error(
                    "Error may be Floor is not match with all floors of lift, pleas check!"
                )
                response.message = "Destination Floor is not match!"
                response.success = False
                return response

            liftData = self.pyPLC.batchread_wordunits(self.lift_data_reg, 6)
            # Xử lý tầng trước:
            currentFloor = liftData[1]
            if currentFloor != numFloor:
                self.pyPLC.batchwrite_wordunits(self.destination_floor_reg, [numFloor])
                self.pyPLC.batchwrite_bitunits(self.lift_trigger_bit, [1])
                process_state = 1
                startTime = self.get_clock().now()
                while process_state:
                    process_state = self.pyPLC.batchread_bitunits(
                        self.lift_trigger_bit, 1
                    )[0]
                    durationTime = (self.get_clock().now() - startTime).nanoseconds * (
                        10 ** (-9)
                    )
                    if durationTime >= self.timeout:
                        self.get_logger().error(f"Timeout lift reaches!")
                        response.success = False
                        response.message = "Lift timeout error!"
                        return response
                    time.sleep(0.5)
                    continue
                liftData = self.pyPLC.batchread_wordunits(self.lift_data_reg, 6)

            # Xử lý cửa:
            currentDoorState = liftData[0]
            door_control = 0
            if (request.door_state == Lift.Request.DOOR_OPEN) and (
                currentDoorState == 1
            ):
                door_control = 1
            elif (request.door_state == Lift.Request.DOOR_CLOSED) and (
                currentDoorState == 2
            ):
                door_control = 2

            if door_control != 0:
                self.pyPLC.batchwrite_wordunits(self.door_control_reg, [door_control])
                self.pyPLC.batchwrite_bitunits(self.door_trigger_bit, [1])
                process_state = 1
                startTime = self.get_clock().now()
                while process_state:
                    process_state = self.pyPLC.batchread_bitunits(
                        self.door_trigger_bit, 1
                    )[0]
                    durationTime = (self.get_clock().now() - startTime).nanoseconds * (
                        10 ** (-9)
                    )
                    if durationTime >= self.timeout:
                        self.get_logger().error(f"Timeout door reaches!")
                        response.success = False
                        response.message = "Door timeout error!"
                        return response
                    time.sleep(0.5)
                    continue
            self.get_logger().info(f"Request client is done!")
            response.success = True
            response.message = "Process success!"
            return response

        except Exception as e:
            self.get_logger().error(e)
            response.success = False
            response.message = "error undefined!"
            return response


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="lift_server", description="Configure and spin up the lift_server"
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    lift_service = LiftService(config_yaml=config_yaml["lift_info"])
    rclpy.spin(lift_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lift_service.pyPLC.close()
    lift_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

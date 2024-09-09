import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node
from .mcprotocol.type1e import Type1E

from ldm_fleet_msgs.msg import LiftState


class LiftStateUpdate(Node):
    def __init__(self, config_yaml):
        super().__init__("lift_state_update")
        self.config_yaml = config_yaml

        # Params:
        # Cấu hình các thông số quan trọng:
        self.IP_addres_PLC = self.config_yaml["ip"]
        self.port_addres_PLC = self.config_yaml["port_lift_state"]
        self.frequency = self.config_yaml["frequency"]

        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"frequency: {self.frequency}")

        self.pyPLC = Type1E("F")
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        # ------ Address all device -------:
        # Bits:

        ## Registers:
        # Lift data
        # Door_state: 0 (1: closed, 2: open, 3: moving)
        # Current_Floor: 1 (1: level 1, 2: level 2...)
        # Motion_state: 2 (1: STOP, 2: UP, 3: DOWN, 4: UNKNOWN)
        # Operation_mode: 3 (1: Manual, 2: Auto)
        # Current_state: 4 (1: Available, 2: Error)
        # Cabin_state: 5 (1: Full, 2: Empty)
        self.lift_data_reg = self.config_yaml["register"]["lift_data"]

        # Publishers:
        self.liftStatePub = self.create_publisher(LiftState, "/lift_state", 10)

        timer_period = 1 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("is running!!!!!!!!!!")

    def timer_callback(self):
        liftStateMsg = LiftState()
        liftData = self.pyPLC.batchread_wordunits(self.lift_data_reg, 6)

        # 0
        if liftData[0] == 1:
            liftStateMsg.door_state = LiftState.DOOR_CLOSED
        elif liftData[0] == 2:
            liftStateMsg.door_state = LiftState.DOOR_OPEN
        else:
            liftStateMsg.door_state = LiftState.DOOR_MOVING
        # 1
        for floor in self.config_yaml["floors"]:
            if self.config_yaml["floors"][floor] == liftData[1]:
                liftStateMsg.current_floor = floor
                break
        # 2
        liftStateMsg.motion_state = liftData[2] - 1
        # 4
        if liftData[4] == 2:
            liftStateMsg.current_mode = LiftState.MODE_EMERGENCY
        else:
            liftStateMsg.current_mode = LiftState.MODE_UNKNOWN

        self.liftStatePub.publish(liftStateMsg)


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

    lift_state_update = LiftStateUpdate(config_yaml=config_yaml["lift_info"])
    rclpy.spin(lift_state_update)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lift_state_update.pyPLC.close()
    lift_state_update.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

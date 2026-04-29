import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # ✅ Correct QoS for PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ✅ Publishers with correct QoS
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.counter = 0
        self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("Takeoff node started")

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.ts()
        self.cmd_pub.publish(msg)

    def timer_cb(self):
        # ✅ Offboard heartbeat
        offboard = OffboardControlMode()
        offboard.position = True
        offboard.timestamp = self.ts()
        self.offboard_pub.publish(offboard)

        # ✅ Trajectory setpoint
        sp = TrajectorySetpoint()

        # 🚀 Smooth takeoff logic
        if self.counter < 50:
            z = 0.0
        elif self.counter < 150:
            z = -5.0 * (self.counter - 50) / 100.0
        else:
            z = -5.0

        sp.position = [0.0, 0.0, z]
        sp.yaw = 0.0
        sp.timestamp = self.ts()
        self.traj_pub.publish(sp)

        # ✅ Switch to OFFBOARD
        if self.counter == 50:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info("Offboard mode set")

        # ✅ ARM
        if self.counter == 80:
            self.send_cmd(400, 1.0)
            self.get_logger().info("ARM sent")

        # Debug
        if self.counter % 20 == 0:
            self.get_logger().info(f"Target height: {z:.2f} m")

        self.counter += 1


def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
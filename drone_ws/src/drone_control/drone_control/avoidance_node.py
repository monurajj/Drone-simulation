import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class SafeStopNode(Node):
    D_STOP   = 2.0   # stop when front < this (m)
    D_RESUME = 2.5   # resume only when front > this (m)
    V_FORWARD = 0.5  # forward speed when clear (m/s)

    def __init__(self):
        super().__init__('safe_stop_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.front_distance = float('inf')
        self.armed = False
        self.offboard_active = False
        self.counter = 0
        self.moving = True   # state: moving forward or stopped

        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info('Safe Stop Node: move forward, stop if wall < 2.0m')

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.ts()
        self.cmd_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        # Compute front distance (center ±30°), ignore <0.5m (drone body)
        angle_min = -2.3562   # -135 deg
        angle_inc = 0.004367
        front_vals = []
        for i, r in enumerate(msg.ranges):
            if r < 0.5 or not math.isfinite(r):
                continue
            angle = angle_min + i * angle_inc
            if abs(angle) < math.radians(30):
                front_vals.append(r)
        self.front_distance = min(front_vals) if front_vals else float('inf')

    def control_loop(self):
        ts = self.ts()

        # Offboard heartbeat (must be published at > 10 Hz)
        om = OffboardControlMode()
        om.velocity = True
        om.timestamp = ts
        self.offboard_pub.publish(om)

        # Arming and offboard mode (2s arm, 4s offboard)
        self.counter += 1
        if self.counter == 40 and not self.armed:
            self.send_command(400, 1.0)
            self.armed = True
            self.get_logger().info('ARM command sent')
        if self.counter == 80 and not self.offboard_active:
            self.send_command(176, 1.0, 6.0)
            self.offboard_active = True
            self.get_logger().info('Offboard mode engaged')

        if not self.offboard_active:
            sp = TrajectorySetpoint()
            sp.velocity = [0.0, 0.0, 0.0]
            sp.yaw = float('nan')
            sp.timestamp = ts
            self.traj_pub.publish(sp)
            return

        # Hysteresis state machine
        if self.moving:
            if self.front_distance < self.D_STOP:
                self.moving = False
                self.get_logger().info(f'⚠️ AVOIDANCE: Wall at {self.front_distance:.2f}m → stopping')
        else:
            if self.front_distance > self.D_RESUME:
                self.moving = True
                self.get_logger().info(f'✅ AVOIDANCE: Wall cleared at {self.front_distance:.2f}m → resuming')

        vx = self.V_FORWARD if self.moving else 0.0
        sp = TrajectorySetpoint()
        sp.velocity = [vx, 0.0, 0.0]   # no lateral movement
        sp.yaw = float('nan')
        sp.timestamp = ts
        self.traj_pub.publish(sp)

        # Log every ~0.5 seconds (10 cycles at 20 Hz)
        if self.counter % 10 == 0:
            status = "MOVING" if self.moving else "STOPPED (wall within 2m)"
            self.get_logger().info(f'Front: {self.front_distance:.2f}m | {status}')

def main():
    rclpy.init()
    node = SafeStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
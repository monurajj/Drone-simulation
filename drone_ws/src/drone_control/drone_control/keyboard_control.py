import rclpy
import sys
import termios
import tty
import select
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

KEY_UP    = "\x1b[A"
KEY_DOWN  = "\x1b[B"
KEY_RIGHT = "\x1b[C"
KEY_LEFT  = "\x1b[D"

class KeyboardControl(Node):
    STEP_XY  = 0.4
    STEP_Z   = 0.3
    STEP_YAW = 0.15

    # Safety braking parameters
    BRAKE_START = 2.5   # start braking when front < this (m)
    BRAKE_STOP  = 2.0   # full stop when front < this (m)

    def __init__(self):
        super().__init__("keyboard_control")
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.traj_pub     = self.create_publisher(TrajectorySetpoint,  "/fmu/in/trajectory_setpoint",   10)
        self.cmd_pub      = self.create_publisher(VehicleCommand,      "/fmu/in/vehicle_command",        10)

        # LiDAR subscriber for front distance
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.z = -2.5
        self.yaw = 0.0
        self.armed = False
        self.offboard_active = False
        self.counter = 0
        self.last_key = ""

        self.front_distance = float('inf')

        self.create_timer(0.05, self.heartbeat_cb)
        self.create_timer(0.05, self.key_cb)
        self.create_timer(0.5,  self.display_cb)
        self.print_menu()

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        vc = VehicleCommand()
        vc.command          = command
        vc.param1           = float(p1)
        vc.param2           = float(p2)
        vc.target_system    = 1
        vc.target_component = 1
        vc.source_system    = 1
        vc.source_component = 1
        vc.from_external    = True
        vc.timestamp        = self.ts()
        self.cmd_pub.publish(vc)

    def scan_cb(self, msg):
        # Compute front distance (center ±30°), ignore <0.5m (drone body)
        angle_min = -2.3562
        angle_inc = 0.004367
        front_vals = []
        for i, r in enumerate(msg.ranges):
            if r < 0.5 or not math.isfinite(r):
                continue
            angle = angle_min + i * angle_inc
            if abs(angle) < math.radians(30):
                front_vals.append(r)
        self.front_distance = min(front_vals) if front_vals else float('inf')

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            return ""
        key = sys.stdin.read(1)
        if key == "\x1b":
            extra = sys.stdin.read(2)
            return key + extra
        return key

    def heartbeat_cb(self):
        om = OffboardControlMode()
        om.position  = True
        om.timestamp = self.ts()
        self.offboard_pub.publish(om)

        sp = TrajectorySetpoint()
        sp.position  = [self.x, self.y, self.z]
        sp.yaw       = self.yaw
        sp.timestamp = self.ts()
        self.traj_pub.publish(sp)

        self.counter += 1
        if self.counter == 40 and not self.offboard_active:
            self.send_command(176, 1.0, 6.0)  # offboard mode
            self.offboard_active = True
            self.get_logger().info("Offboard mode requested")
        if self.counter == 50 and not self.armed:
            self.send_command(400, 1.0)
            self.armed = True
            self.get_logger().info("ARM command sent!")

    def key_cb(self):
        key = self.get_key()
        if not key:
            return
        self.last_key = key
        self.handle_key(key)

    def handle_key(self, key):
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        # Default movement increments
        dx = 0.0
        dy = 0.0
        dz = 0.0
        dyaw = 0.0

        # Arrow keys (forward/back/left/right)
        if key == KEY_UP:
            dx = self.STEP_XY * cos_y
            dy = self.STEP_XY * sin_y
        elif key == KEY_DOWN:
            dx = -self.STEP_XY * cos_y
            dy = -self.STEP_XY * sin_y
        elif key == KEY_LEFT:
            dx = self.STEP_XY * (-sin_y)
            dy = self.STEP_XY * cos_y
        elif key == KEY_RIGHT:
            dx = -self.STEP_XY * (-sin_y)
            dy = -self.STEP_XY * cos_y
        elif key == "w":
            dz = -self.STEP_Z
        elif key == "s":
            dz = self.STEP_Z
        elif key == "a":
            dyaw = self.STEP_YAW
        elif key == "d":
            dyaw = -self.STEP_YAW
        elif key == "h":
            self.get_logger().info("HOLD — hovering in place")
        elif key == "l":
            self.get_logger().info("LAND command sent")
            self.send_command(21)
        elif key == "r":
            self.get_logger().info("RTL command sent")
            self.send_command(20)
        elif key == "k":
            self.get_logger().warn("DISARM command sent!")
            self.send_command(400, 0.0)
            self.armed = False
        elif key == "t":
            self.get_logger().info("Re-ARM command sent")
            self.send_command(400, 1.0)
            self.armed = True
        elif key in ("x", "\x03"):
            self.get_logger().info("Exiting...")
            rclpy.shutdown()

        # Apply safety braking for forward/backward movement
        # If front wall is too close, reduce or zero dx (forward component)
        if dx > 0:  # only forward movement (positive dx)
            if self.front_distance < self.BRAKE_STOP:
                dx = 0.0
                self.get_logger().warn(f"🛑 BRAKE: wall at {self.front_distance:.2f}m → forward blocked")
            elif self.front_distance < self.BRAKE_START:
                # Scale down linearly from 1 at BRAKE_START to 0 at BRAKE_STOP
                ratio = (self.front_distance - self.BRAKE_STOP) / (self.BRAKE_START - self.BRAKE_STOP)
                dx = dx * max(0.0, ratio)
                self.get_logger().info(f"⚠️ Braking: front {self.front_distance:.2f}m → speed reduced to {dx:.2f}")
            # else no braking

        # Update position / yaw
        self.x += dx
        self.y += dy
        self.z = max(min(self.z + dz, -0.3), -30.0)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

    def display_cb(self):
        alt = abs(self.z)
        armed_str = "ARMED" if self.armed else "DISARMED"
        sys.stdout.write(
            f"\r  [{armed_str}] x:{self.x:+6.1f}  y:{self.y:+6.1f}  "
            f"alt:{alt:5.1f}m  yaw:{math.degrees(self.yaw):+6.1f}deg  "
            f"front:{self.front_distance:5.1f}m  key:{repr(self.last_key):<10}"
        )
        sys.stdout.flush()

    def print_menu(self):
        print("""
+--------------------------------------------------+
|     DRONE KEYBOARD CONTROLLER (WITH SAFETY)      |
+--------------------------------------------------+
|  Arrow Keys   Move Forward/Back/Left/Right       |
|  W / S        Altitude Up / Down                 |
|  A / D        Yaw Left / Right                   |
+--------------------------------------------------+
|  H            Hold (hover in place)              |
|  L            Land                               |
|  R            Return to Launch (RTL)             |
|  T            Re-Arm                             |
|  K            KILL / Disarm                      |
|  X            Exit                               |
+--------------------------------------------------+
|  !!! Auto-brakes when wall closer than 2.5m !!!  |
+--------------------------------------------------+
""")

def main():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    rclpy.init()
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nTerminal restored.")
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

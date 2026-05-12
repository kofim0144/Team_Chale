#!/usr/bin/env python3
"""
Robot Coordinator (greenhouse harvest FSM) -- closed-loop drive.

Pose source: a worker thread that runs `gz topic -e -t /world/<world>/pose/info
continuously. We parse each message and extract the X position of the
chaleBOT model. This is the robot's TRUE world pose (no odom drift).

Drive control: P-controller. Velocity proportional to remaining distance,
with min/max clamping. Stops when both pos AND vel are within tolerance.

Sequence of events:
  INIT          -- wait for first world pose
  STARTUP_HOME  -- send arm to HOME, wait for it to finish
  CAMERA_SETTLE -- command camera RIGHT, wait camera_settle_s seconds
  DRIVING       -- drive forward drive_step_m via P-controller
  SCANNING      -- stop, wait scan_duration_s for detection
  POST_SCAN     -- pause post_scan_pause_s for the viewer
  PICKING       -- arm runs the harvest cycle; we wait
  DONE          -- terminal state
"""

import math
import re
import subprocess
import threading
import rclpy
from rclpy.node import Node
from enum import Enum

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String, Float64MultiArray


class State(Enum):
    INIT           = 0
    STARTUP_HOME   = 1   # send arm home, wait for completion
    CAMERA_SETTLE  = 2   # camera commanded, waiting to settle
    DRIVING        = 3
    SCANNING       = 4
    POST_SCAN      = 5   # short pause after scan completes
    PICKING        = 6
    DONE           = 7


CAMERA_RIGHT = +0.0
ROBOT_MODEL_NAME = 'chaleBOT'
WORLD_NAME = 'greenhouse_world'


class Coordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')

        # ----- Drive parameters ------------------------------------------
        self.declare_parameter('drive_step_m',        0.30)
        self.declare_parameter('total_drive_m',      6.0)
        self.declare_parameter('scan_duration_s',     5)
        self.declare_parameter('detection_timeout_s', 7)
        self.declare_parameter('max_reach_m',         1.20)

        # ----- P-controller parameters -----------------------------------
        self.declare_parameter('drive_kp',           0.8)
        self.declare_parameter('drive_max_speed',    0.30)
        self.declare_parameter('drive_min_speed',    0.02)
        self.declare_parameter('pos_tolerance_m',    0.01)
        self.declare_parameter('vel_tolerance_mps',  0.005)

        # ----- Pause durations -------------------------------------------
        self.declare_parameter('camera_settle_s',    5)   # after camera cmd
        self.declare_parameter('post_scan_pause_s',  5.0)   # after scan

        self.step_m         = self.get_parameter('drive_step_m').value
        self.total_drive_m  = self.get_parameter('total_drive_m').value
        self.scan_duration  = self.get_parameter('scan_duration_s').value
        self.det_timeout    = self.get_parameter('detection_timeout_s').value
        self.max_reach      = self.get_parameter('max_reach_m').value
        self.drive_kp       = self.get_parameter('drive_kp').value
        self.drive_max      = self.get_parameter('drive_max_speed').value
        self.drive_min      = self.get_parameter('drive_min_speed').value
        self.pos_tol        = self.get_parameter('pos_tolerance_m').value
        self.vel_tol        = self.get_parameter('vel_tolerance_mps').value
        self.camera_settle  = self.get_parameter('camera_settle_s').value
        self.post_scan_pause = self.get_parameter('post_scan_pause_s').value

        # ----- FSM state -------------------------------------------------
        self.state = State.INIT
        self.scan_start_time     = None
        self.camera_settle_start = None
        self.post_scan_start     = None
        self.drive_start_x       = None
        self.drive_target_x      = None
        self.total_distance      = 0.0
        self.fruits_picked       = 0
        self.pending_pick_after_post_scan = False

        # ----- Latest fruit detection ------------------------------------
        self.last_pose: PoseStamped | None = None
        self.last_pose_time = self.get_clock().now()
        self.last_diam: float = 0.0

        # ----- Robot world pose state -----------------------------------
        self.world_x: float | None = None
        self.world_x_prev: float | None = None
        self.world_x_time_prev = None
        self.world_vx: float = 0.0
        self._pose_lock = threading.Lock()

        # ----- ROS pubs / subs -------------------------------------------
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_pub   = self.create_publisher(String, '/arm/task', 10)
        self.camera_pub = self.create_publisher(Float64MultiArray,
                                                 '/camera_controller/commands',
                                                 10)
        self.world_x_pub = self.create_publisher(Float32, '/coordinator/world_x', 10)

        self.create_subscription(PoseStamped, '/fruit/pose',     self.pose_cb, 10)
        self.create_subscription(Float32,     '/fruit/diameter', self.size_cb, 10)
        self.create_subscription(String,      '/arm/status',     self.arm_cb,  10)

        # FSM tick at 10 Hz.
        self.create_timer(0.1, self.tick)

        # gz CLI streaming thread.
        self._stream_thread = threading.Thread(
            target=self._gz_pose_stream_loop, daemon=True)
        self._stream_thread.start()

        self.get_logger().info(
            f'Coordinator started. Streaming gz pose for {ROBOT_MODEL_NAME}. '
            f'Kp={self.drive_kp}, vmax={self.drive_max}, '
            f'pos_tol={self.pos_tol}, vel_tol={self.vel_tol}, '
            f'camera_settle={self.camera_settle}s, '
            f'post_scan_pause={self.post_scan_pause}s.')

    # ====================================================================
    # gz pose stream worker thread
    # ====================================================================
    def _gz_pose_stream_loop(self):
        cmd = ['gz', 'topic', '-e', '-t',
               f'/world/{WORLD_NAME}/pose/info']
        while rclpy.ok():
            try:
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True, bufsize=1)
            except Exception as e:
                self.get_logger().warn(f'gz topic spawn failed: {e}')
                return

            current_name = None
            for line in proc.stdout:
                if not rclpy.ok():
                    break
                line = line.strip()
                m = re.match(r'name:\s*"([^"]+)"', line)
                if m:
                    current_name = m.group(1)
                    continue
                if current_name == ROBOT_MODEL_NAME:
                    m2 = re.match(r'x:\s*(-?\d+\.?\d*(?:e-?\d+)?)', line)
                    if m2:
                        self._record_world_x(float(m2.group(1)))
                        current_name = None
            proc.wait()
            self.get_logger().warn('gz pose stream ended; restarting in 1s')
            import time
            time.sleep(1.0)

    def _record_world_x(self, x: float):
        now = self.get_clock().now()
        with self._pose_lock:
            if (self.world_x_time_prev is not None and
                    self.world_x_prev is not None):
                dt = (now - self.world_x_time_prev).nanoseconds / 1e9
                if dt > 1e-3:
                    self.world_vx = (x - self.world_x_prev) / dt
            self.world_x_prev      = x
            self.world_x_time_prev = now
            self.world_x           = x
        try:
            self.world_x_pub.publish(Float32(data=float(x)))
        except Exception:
            pass

    # ====================================================================
    # Other subscribers
    # ====================================================================
    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg
        self.last_pose_time = self.get_clock().now()

    def size_cb(self, msg: Float32):
        self.last_diam = msg.data

    def arm_cb(self, msg: String):
        s = msg.data

        # Startup home completion: command camera & start the demo.
        if self.state == State.STARTUP_HOME:
            if s == 'home':
                self.get_logger().info('Arm at HOME. Commanding camera RIGHT.')
                self._command_camera(CAMERA_RIGHT)
                self.camera_settle_start = self.get_clock().now()
                self.state = State.CAMERA_SETTLE
            elif s == 'failed':
                self.get_logger().warn(
                    'Startup home failed. Commanding camera anyway.')
                self._command_camera(CAMERA_RIGHT)
                self.camera_settle_start = self.get_clock().now()
                self.state = State.CAMERA_SETTLE
            return

        # Pick completion / failure: resume driving.
        if self.state == State.PICKING:
            if s == 'picked':
                self.fruits_picked += 1
                self.get_logger().info(
                    f'Fruit #{self.fruits_picked} harvested. Continuing.')
                self._begin_driving()
            elif s == 'failed':
                self.get_logger().warn('Arm task failed. Resuming drive.')
                self._begin_driving()

    # ====================================================================
    # Main FSM tick
    # ====================================================================
    def tick(self):
        # ---- INIT: send arm home before anything else ------------------
        if self.state == State.INIT:
            if self.world_x is None:
                return
            self.get_logger().info(
                f'Sending arm to HOME at startup. '
                f'World X={self.world_x:+.3f}.')
            self.task_pub.publish(String(data='home'))
            self.state = State.STARTUP_HOME
            return

        # ---- STARTUP_HOME: handled by arm_cb (no work in tick) ---------

        # ---- CAMERA_SETTLE: wait camera_settle_s seconds ---------------
        elif self.state == State.CAMERA_SETTLE:
            elapsed = (self.get_clock().now() - self.camera_settle_start).nanoseconds / 1e9
            if elapsed < self.camera_settle:
                return
            self.get_logger().info(
                f'Camera settled ({self.camera_settle:.1f}s). Beginning drive.')
            self._begin_driving()

        # ---- DRIVING ---------------------------------------------------
        elif self.state == State.DRIVING:
            if self.world_x is None:
                return
            self._step_drive_controller()

        # ---- SCANNING --------------------------------------------------
        elif self.state == State.SCANNING:
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed < self.scan_duration:
                return

            if self._have_fresh_reachable_target():
                self.get_logger().info(
                    'Reachable fruit detected during scan.')
                self.pending_pick_after_post_scan = True
            else:
                self.pending_pick_after_post_scan = False

            self.get_logger().info(
                f'Scan window ended. Pausing {self.post_scan_pause:.1f}s.')
            self.post_scan_start = self.get_clock().now()
            self.state = State.POST_SCAN

        # ---- POST_SCAN: wait post_scan_pause_s seconds ----------------
        elif self.state == State.POST_SCAN:
            elapsed = (self.get_clock().now() - self.post_scan_start).nanoseconds / 1e9
            if elapsed < self.post_scan_pause:
                return

            if self.pending_pick_after_post_scan and self.last_pose is not None:
                p = self.last_pose.pose.position
                d = self.last_diam
                self.get_logger().info(
                    f'Reachable fruit at ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f}) '
                    f'(d={d*100:.1f}cm). Picking.')
                self.task_pub.publish(String(
                    data=f'pick_with_size:{p.x:.4f},{p.y:.4f},{p.z:.4f},{d:.4f}'))
                self.pending_pick_after_post_scan = False
                self.state = State.PICKING
            else:
                if self.total_distance >= self.total_drive_m:
                    self.get_logger().info(
                        'No fruit and total_drive_m reached. Done.')
                    self._finish_harvesting()
                    return
                self.get_logger().info(
                    'No reachable fruit. Driving forward another step.')
                self._begin_driving()

        # PICKING: handled by arm_cb.
        # DONE: terminal.

    # ====================================================================
    # Drive controller (P-controller, runs at 10 Hz)
    # ====================================================================
    def _step_drive_controller(self):
        error = self.drive_target_x - self.world_x
        v_cmd = self.drive_kp * error

        if abs(error) > self.pos_tol:
            sign = 1.0 if v_cmd >= 0 else -1.0
            v_mag = max(min(abs(v_cmd), self.drive_max), self.drive_min)
            v_cmd = sign * v_mag
        else:
            v_cmd = 0.0

        t = Twist()
        t.linear.x = v_cmd
        self.cmd_pub.publish(t)

        in_pos = abs(error) < self.pos_tol
        in_vel = abs(self.world_vx) < self.vel_tol
        if in_pos and in_vel:
            travelled = abs(self.world_x - self.drive_start_x)
            self.total_distance += travelled
            self.get_logger().info(
                f'Drove {travelled:.3f} m '
                f'(target {self.step_m:.3f}, error {error*100:+.1f} cm). '
                f'Total {self.total_distance:.3f} m. Stopping to scan.')
            if self.total_distance >= self.total_drive_m - 1e-3:
                self.get_logger().info(
                    f'Reached total_drive_m={self.total_drive_m}. Stopping.')
                self._finish_harvesting()
                return
            self._begin_scanning()
            return

        travelled_so_far = abs(self.world_x - self.drive_start_x)
        if (self.total_distance + travelled_so_far) >= self.total_drive_m:
            self.get_logger().info(f'Reached total_drive_m mid-step.')
            self.total_distance += travelled_so_far
            self._finish_harvesting()
            return

    # ====================================================================
    # State transitions
    # ====================================================================
    def _begin_driving(self):
        if self.world_x is None:
            self.get_logger().warn('World X not yet available; deferring.')
            return
        self.drive_start_x  = self.world_x
        self.drive_target_x = self.world_x + self.step_m
        self.last_pose = None
        self.state = State.DRIVING
        self.get_logger().info(
            f'Drive segment: {self.world_x:+.3f} -> {self.drive_target_x:+.3f}')

    def _begin_scanning(self):
        self._stop_base()
        self.scan_start_time = self.get_clock().now()
        self.last_pose = None
        self.state = State.SCANNING

    def _finish_harvesting(self):
        self._stop_base()
        self.task_pub.publish(String(data='home'))
        self.get_logger().info(
            f'=== Harvest complete with {self.fruits_picked} fruit(s) picked. ===')
        self.state = State.DONE

    # ====================================================================
    # Helpers
    # ====================================================================
    def _have_fresh_reachable_target(self) -> bool:
        if self.last_pose is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if dt > self.det_timeout:
            return False
        p = self.last_pose.pose.position
        return (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5 <= self.max_reach

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    def _command_camera(self, angle_rad: float):
        msg = Float64MultiArray()
        msg.data = [angle_rad]
        self.camera_pub.publish(msg)


def main():
    rclpy.init()
    n = Coordinator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
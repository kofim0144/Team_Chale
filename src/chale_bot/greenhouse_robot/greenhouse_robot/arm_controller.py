#!/usr/bin/env python3
"""
Arm Controller (MoveIt2 + Gazebo)
=================================
Greenhouse harvest cycle with PRE-GRASP APPROACH, continuous
carried-fruit tracking using GRIPPER-LOCAL OFFSET, and grasp
verification.

Fruit-registry lookup uses Gazebo's TRUE robot world pose (via
gz CLI) instead of TF, because the TF-based odom frame suffers
from wheel-slip drift in this simulation.
"""

import math
import os
import subprocess
import threading
import time
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                              PositionConstraint, OrientationConstraint,
                              WorkspaceParameters)
from shape_msgs.msg import SolidPrimitive


# ============================================================
# CONFIG
# ============================================================
PLANNING_GROUP    = 'arm_group'
END_EFFECTOR_LINK = 'gripper_stator_1'   # used by MoveIt planning and verification
CARRIED_FRUIT_LINK = 'gripper_clamp_1'    # used by carried-fruit tracking (visible jaws)
WORLD_NAME        = 'greenhouse_world'
ROBOT_MODEL_NAME  = 'chaleBOT'

X_COMPENSATION = 0.0
Y_COMPENSATION = 0.05
Z_COMPENSATION = 0.0

GRASP_VERIFY_TOLERANCE_M = 0.10

CARRY_OFFSET_LOCAL = (0.0, 0.0, 0.05)
# Carried-fruit offset in CARRIED_FRUIT_LINK's local frame (gripper_clamp).
# Since gripper_clamp is at the visible clamping point, (0, 0, 0) is a
# reasonable starting value. Tweak if the fruit appears slightly off.

DROP_OFFSET_BELOW_GRIPPER_M = 0.10

PRE_GRASP_OFFSET_M = 0.10

GRASP_QX = 0.7071
GRASP_QY = 0.0
GRASP_QZ = 0.0
GRASP_QW = 0.7071

ORIENT_TOL_X = 0.30
ORIENT_TOL_Y = 0.10
ORIENT_TOL_Z = 0.10

BIG_BIN_OFFSET_X    = -0.70
BIG_BIN_OFFSET_Y    = +0.10
BIG_BIN_HEIGHT_Z    = 0.525
SMALL_BIN_OFFSET_X  = -0.70
SMALL_BIN_OFFSET_Y  = -0.10
SMALL_BIN_HEIGHT_Z  = 0.525

JOINT_NAMES = ['Revolute_14', 'Revolute_15', 'Revolute_16',
               'Revolute_17', 'Revolute_18']

HOME      = [ 0.00, -0.96,  1.92,  0.61, 0.0]
BIN_SMALL = [-1.6134, 1.3618, 0.5985, 0.3036, 0.0]
BIN_LARGE = [-1.9603, 1.2577, 0.6679, 0.4944, 0.0]

APPROACH_DZ = -0.02

DWELL_AT_FRUIT_S       = 5
DWELL_AT_BIN_S         = 5
DWELL_AFTER_DROP_S     = 5.0

SIZE_THRESHOLD_M = 0.06
DROP_FRUIT_MASS_KG = 1.0
DROP_FRUIT_RADIUS_M = 0.03   # radius of the fruit spawned at bin to fall
PLAN_POSITION_TOL_M = 0.03

GOAL_ACK_TIMEOUT_S      = 5.0
EXECUTION_TIMEOUT_S     = 120.0
ACTION_POLL_INTERVAL_S  = 0.05

TRACK_HZ = 20.0


def quat_rotate_vector(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = qy * vz - qz * vy
    ty = qz * vx - qx * vz
    tz = qx * vy - qy * vx
    ux = qw * vx + tx
    uy = qw * vy + ty
    uz = qw * vz + tz
    cx = qy * uz - qz * uy
    cy = qz * ux - qx * uz
    cz = qx * uy - qy * ux
    return (vx + 2.0 * cx, vy + 2.0 * cy, vz + 2.0 * cz)


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')
        self.last_carried_fruit_name = None
        self.last_carried_fruit_radius = 0.025

        self._track_stop_event = threading.Event()
        self._track_thread = None

        self.declare_parameter('fruit_registry_path', '')
        registry_path = self.get_parameter('fruit_registry_path').value
        if not registry_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('greenhouse_robot')
                registry_path = os.path.join(pkg, 'config', 'fruit_registry.yaml')
            except Exception:
                registry_path = ''

        self.fruit_registry: list[dict] = []
        self.lookup_tolerance_m = 0.30
        self._load_registry(registry_path)

        self.cbg = ReentrantCallbackGroup()
        self.move_action = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cbg)
        self.status_pub = self.create_publisher(String, '/arm/status', 10)
        self.create_subscription(
            String, '/arm/task', self.task_cb, 10, callback_group=self.cbg)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.get_logger().info('Waiting for MoveIt /move_action server...')
        self.move_action.wait_for_server()
        self.get_logger().info(
            f'MoveIt connected. Registry has {len(self.fruit_registry)} '
            f'fruit(s). Pre-grasp offset: {PRE_GRASP_OFFSET_M*100:.0f} cm. '
            f'Compensations (X,Y,Z): ({X_COMPENSATION}, '
            f'{Y_COMPENSATION}, {Z_COMPENSATION}). '
            f'Grasp verify tolerance: {GRASP_VERIFY_TOLERANCE_M*100:.1f} cm. '
            f'Carry offset (local in {CARRIED_FRUIT_LINK}): {CARRY_OFFSET_LOCAL}. '
            f'Tracking rate: {TRACK_HZ} Hz. Ready for tasks.')

    def _load_registry(self, path: str):
        if not path or not Path(path).is_file():
            self.get_logger().warn(f'Registry not found: {path or "<unset>"}')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.fruit_registry = data.get('fruits', [])
            self.lookup_tolerance_m = float(
                data.get('lookup_tolerance_m', 0.30))
            self.get_logger().info(
                f'Loaded {len(self.fruit_registry)} fruit(s) from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load registry: {e}')

    def task_cb(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'Task received: {cmd}')
        try:
            if cmd.startswith('pick_with_size:'):
                parts = cmd[len('pick_with_size:'):].split(',')
                xyz = [float(v) for v in parts[:3]]
                diameter = float(parts[3])
                self._do_pick(xyz, diameter_m=diameter)
                self._status('picked')
            elif cmd.startswith('pick:'):
                xyz = [float(v) for v in cmd[len('pick:'):].split(',')]
                self._do_pick(xyz, diameter_m=None)
                self._status('picked')
            elif cmd.startswith('goto:'):
                xyz = [float(v) for v in cmd[len('goto:'):].split(',')]
                self._do_goto(xyz)
                self._status('goto_done')
            elif cmd == 'place:small':
                self._goto_joints(BIN_SMALL); self._status('placed')
            elif cmd == 'place:large':
                self._goto_joints(BIN_LARGE); self._status('placed')
            elif cmd == 'home':
                self._goto_joints(HOME); self._status('home')
            elif cmd == 'open':
                self._set_gripper(0.0); self._status('gripper_done')
            elif cmd == 'close':
                self._set_gripper(0.04); self._status('gripper_done')
            else:
                self.get_logger().warn(f'Unknown task: {cmd}')
                self._status('failed')
        except Exception as e:
            self.get_logger().error(f'Task failed: {e}')
            self._stop_tracking()
            self._status('failed')

    def _do_pick(self, xyz_in_base_link: list, diameter_m=None):
        x, y, z = xyz_in_base_link
        x_planned = x + X_COMPENSATION
        y_planned = y + Y_COMPENSATION
        z_planned = z + APPROACH_DZ + Z_COMPENSATION

        pre_grasp_pose = Pose()
        pre_grasp_pose.position = Point(
            x=float(x_planned),
            y=float(y_planned + PRE_GRASP_OFFSET_M),
            z=float(z_planned))
        pre_grasp_pose.orientation = Quaternion(
            x=GRASP_QX, y=GRASP_QY, z=GRASP_QZ, w=GRASP_QW)

        grasp_pose = Pose()
        grasp_pose.position = Point(
            x=float(x_planned),
            y=float(y_planned),
            z=float(z_planned))
        grasp_pose.orientation = Quaternion(
            x=GRASP_QX, y=GRASP_QY, z=GRASP_QZ, w=GRASP_QW)

        self.get_logger().info(
            f'  Pick step 1/5: PRE-GRASP at ({x_planned:+.2f}, '
            f'{y_planned + PRE_GRASP_OFFSET_M:+.2f}, {z_planned:+.2f})')
        if not self._plan_to_pose_with_fallback(pre_grasp_pose):
            raise RuntimeError('Plan to pre-grasp pose failed')

        self.get_logger().info(
            f'  Pick step 2/5: APPROACH to fruit at ({x_planned:+.2f}, '
            f'{y_planned:+.2f}, {z_planned:+.2f})')
        if not self._plan_to_pose_with_fallback(grasp_pose):
            self.get_logger().warn(
                '  Approach failed; retreating to pre-grasp.')
            self._plan_to_pose_with_fallback(pre_grasp_pose)
            raise RuntimeError('Approach to grasp pose failed')

        if not self._verify_grasp_pose(
                target_xyz_base_link=(x_planned, y_planned, z_planned)):
            self.get_logger().warn(
                '  Grasp verification FAILED. Retreating to pre-grasp '
                'and aborting pick (no fruit deleted).')
            self._plan_to_pose_with_fallback(pre_grasp_pose)
            self._goto_joints(HOME)
            raise RuntimeError('Grasp verification failed (gripper not at fruit)')

        self.get_logger().info(
            f'  Verified. Waiting {DWELL_AT_FRUIT_S}s before grasping...')
        time.sleep(DWELL_AT_FRUIT_S)

        registry_size = self._delete_fruit_at(xyz_in_base_link)
        if registry_size is None:
            self.get_logger().warn(
                '  No matching fruit found in registry. '
                'Lifting and returning home.')
            self._plan_to_pose_with_fallback(pre_grasp_pose)
            self._goto_joints(HOME)
            return

        if diameter_m is not None:
            is_big = (diameter_m >= SIZE_THRESHOLD_M)
            size_str = 'big' if is_big else 'small'
            self.get_logger().info(
                f'  Diameter={diameter_m*100:.1f}cm -> {size_str}')
        else:
            size_str = registry_size
            is_big = (size_str == 'big')

        radius = 0.05 if is_big else 0.025
        self.last_carried_fruit_radius = radius

        self._spawn_carried_fruit_at_gripper(radius=radius)
        self._start_tracking()

        self.get_logger().info('  Pick step 3/5: LIFT to pre-grasp')
        if not self._plan_to_pose_with_fallback(pre_grasp_pose):
            self.get_logger().warn('  Lift failed; continuing anyway.')

        self.get_logger().info('  Pick step 4/5: HOME')
        if not self._goto_joints(HOME):
            self._stop_tracking()
            raise RuntimeError('Move to HOME failed')

        if is_big:
            self.get_logger().info('  Pick step 5/5: BIG bin')
            self._goto_joints(BIN_LARGE)
        else:
            self.get_logger().info('  Pick step 5/5: SMALL bin')
            self._goto_joints(BIN_SMALL)

        self.get_logger().info(
            f'  At bin. Waiting {DWELL_AT_BIN_S}s before releasing...')
        time.sleep(DWELL_AT_BIN_S)

        self._stop_tracking()
        if self.last_carried_fruit_name:
            self._delete_entity_by_name(self.last_carried_fruit_name)
            self.last_carried_fruit_name = None

        self._spawn_dropping_fruit(radius=DROP_FRUIT_RADIUS_M, is_big=is_big)

        self.get_logger().info(
            f'  Dropped. Waiting {DWELL_AFTER_DROP_S}s for fruit to settle...')
        time.sleep(DWELL_AFTER_DROP_S)

        self.get_logger().info('  Returning to HOME')
        self._goto_joints(HOME)
        self.get_logger().info('  Pick sequence complete')

    def _do_goto(self, xyz_in_base_link: list):
        x, y, z = xyz_in_base_link
        target_pose = Pose()
        target_pose.position = Point(
            x=float(x + X_COMPENSATION),
            y=float(y + Y_COMPENSATION),
            z=float(z + Z_COMPENSATION))
        target_pose.orientation = Quaternion(
            x=GRASP_QX, y=GRASP_QY, z=GRASP_QZ, w=GRASP_QW)
        self.get_logger().info(
            f'  Goto: planning to '
            f'({x + X_COMPENSATION:+.2f}, '
            f'{y + Y_COMPENSATION:+.2f}, '
            f'{z + Z_COMPENSATION:+.2f}) in base_link')
        if not self._plan_to_pose_with_fallback(target_pose):
            raise RuntimeError('Plan to goto pose failed')
        self.get_logger().info('  Goto: motion completed.')

    def _verify_grasp_pose(self, target_xyz_base_link) -> bool:
        tx, ty, tz = target_xyz_base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', END_EFFECTOR_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(
                f'  Verify: TF lookup base_link->{END_EFFECTOR_LINK} '
                f'failed: {e}')
            return True

        gx = tf.transform.translation.x
        gy = tf.transform.translation.y
        gz = tf.transform.translation.z

        dx = gx - tx
        dy = gy - ty
        dz = gz - tz
        err = math.sqrt(dx*dx + dy*dy + dz*dz)

        self.get_logger().info(
            f'  Grasp verify: gripper=({gx:+.3f},{gy:+.3f},{gz:+.3f}) '
            f'target=({tx:+.3f},{ty:+.3f},{tz:+.3f}) '
            f'err=({dx*100:+.1f},{dy*100:+.1f},{dz*100:+.1f})cm '
            f'|err|={err*100:.1f}cm '
            f'tol={GRASP_VERIFY_TOLERANCE_M*100:.1f}cm')

        return err < GRASP_VERIFY_TOLERANCE_M

    def _plan_to_pose_with_fallback(self, pose: Pose) -> bool:
        if self._plan_to_pose(pose, frame_id='base_link', use_orientation=True):
            return True
        self.get_logger().warn(
            '  Plan with orientation failed; retrying position-only')
        return self._plan_to_pose(pose, frame_id='base_link',
                                   use_orientation=False)

    def _delete_fruit_at(self, xyz_in_base_link: list):
        """
        Match the perceived fruit to a registered fruit and delete it.

        Uses Gazebo's TRUE robot world pose (via gz CLI) to convert the
        perception-reported base_link position to a world coordinate.
        This bypasses odom drift, which would otherwise cause wrong-
        fruit matches.

        Assumes the robot's yaw is 0 in the world frame (drives straight
        along world +X). If the robot ever rotates, this needs to apply
        the robot's yaw rotation to the base_link offset.
        """
        if not self.fruit_registry:
            self.get_logger().warn('  Registry empty')
            return None

        robot_pose = self._get_robot_world_pose()
        if robot_pose is None:
            self.get_logger().warn(
                '  Could not read robot world pose from gz; '
                'falling back to TF.')
            return self._delete_fruit_at_tf(xyz_in_base_link)

        rx, ry, rz = robot_pose
        bx, by, bz = xyz_in_base_link
        # Robot drives along world +X with yaw=0, so base_link axes
        # align with world axes. Simple addition is correct.
        wx = rx + bx
        wy = ry + by
        wz = rz + bz

        self.get_logger().info(
            f'  Robot world pose (gz): ({rx:+.2f}, {ry:+.2f}, {rz:+.2f})')
        self.get_logger().info(
            f'  Looking up fruit near world ({wx:+.2f}, {wy:+.2f}, {wz:+.2f})')

        best = None
        best_dist = float('inf')
        for entry in self.fruit_registry:
            d = math.sqrt((entry['x']-wx)**2 + (entry['y']-wy)**2 +
                          (entry['z']-wz)**2)
            if d < best_dist:
                best_dist = d
                best = entry
        if best is None or best_dist > self.lookup_tolerance_m:
            self.get_logger().warn(
                f'  No registry fruit within {self.lookup_tolerance_m} m '
                f'(closest: {best["name"] if best else "none"} @ '
                f'{best_dist:.2f} m)')
            return None
        if self._gz_remove_entity(best['name']):
            self.get_logger().info(
                f'  Deleted fruit "{best["name"]}" '
                f'(size={best.get("size", "?")}, dist={best_dist:.3f} m)')
            self.fruit_registry = [
                e for e in self.fruit_registry if e['name'] != best['name']]
            return best.get('size', 'small')
        return None

    def _delete_fruit_at_tf(self, xyz_in_base_link: list):
        """Fallback: original TF-based lookup if gz pose unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().warn(f'  TF lookup failed: {e}')
            return None
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.point.x, pt.point.y, pt.point.z = xyz_in_base_link
        pt_world = do_transform_point(pt, tf)
        wx, wy, wz = pt_world.point.x, pt_world.point.y, pt_world.point.z
        self.get_logger().info(
            f'  [TF fallback] Looking up fruit near world '
            f'({wx:+.2f}, {wy:+.2f}, {wz:+.2f})')
        best = None
        best_dist = float('inf')
        for entry in self.fruit_registry:
            d = math.sqrt((entry['x']-wx)**2 + (entry['y']-wy)**2 +
                          (entry['z']-wz)**2)
            if d < best_dist:
                best_dist = d
                best = entry
        if best is None or best_dist > self.lookup_tolerance_m:
            self.get_logger().warn(
                f'  No registry fruit within {self.lookup_tolerance_m} m '
                f'(closest: {best["name"] if best else "none"} @ '
                f'{best_dist:.2f} m)')
            return None
        if self._gz_remove_entity(best['name']):
            self.get_logger().info(
                f'  Deleted fruit "{best["name"]}" '
                f'(size={best.get("size", "?")}, dist={best_dist:.3f} m)')
            self.fruit_registry = [
                e for e in self.fruit_registry if e['name'] != best['name']]
            return best.get('size', 'small')
        return None

    # =====================================================================
    # Carried-fruit lifecycle
    # =====================================================================
    def _spawn_carried_fruit_at_gripper(self, radius: float):
        if self.last_carried_fruit_name:
            self._gz_remove_entity(self.last_carried_fruit_name)
            self.last_carried_fruit_name = None

        gx, gy, gz = self._compute_carried_fruit_world_pos()
        if gx is None:
            self.get_logger().warn('  Could not read gripper TF; skip spawn.')
            return

        spawn_name = f'carried_fruit_{int(time.time() * 1000) % 1000000}'
        sdf = (
            f'<sdf version="1.7">'
            f'<model name="{spawn_name}">'
            f'<static>true</static>'
            f'<pose>{gx} {gy} {gz} 0 0 0</pose>'
            f'<link name="link">'
            f'<visual name="visual">'
            f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>'
            f'<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>'
            f'</visual>'
            f'</link>'
            f'</model>'
            f'</sdf>'
        )
        if self._gz_create_entity(sdf):
            self.last_carried_fruit_name = spawn_name
            self.get_logger().info(
                f'  Spawned carried fruit "{spawn_name}" at world '
                f'({gx:+.2f}, {gy:+.2f}, {gz:+.2f})')

    def _compute_carried_fruit_world_pos(self):
        """
        Compute the carried fruit's world-frame position WITHOUT going
        through odom (which has wheel-slip drift in this simulation).

        Strategy:
          1. Get robot's TRUE world position from gz CLI (drift-free).
          2. Get gripper_clamp_1 pose in base_link from TF (drift-free
             since this subtree doesn't depend on odom).
          3. Add them (assumes robot yaw = 0, which holds in this demo).
          4. Rotate CARRY_OFFSET_LOCAL by the gripper's orientation.
          5. Add the rotated offset to step 3's result.
        """
        # Step 1: robot world pose (bypasses odom).
        robot_pose = self._get_robot_world_pose()
        if robot_pose is None:
            return (None, None, None)
        rx, ry, rz = robot_pose

        # Step 2: gripper pose in base_link (drift-free subtree).
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', CARRIED_FRUIT_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception:
            return (None, None, None)

        # Gripper position relative to base_link.
        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        bz = tf.transform.translation.z

        # Step 3: gripper world position = robot world + gripper in base_link.
        # (Assumes robot yaw = 0 in this demo. If the robot ever rotates,
        # the gripper-in-base-link offset must first be rotated by the
        # robot's world yaw.)
        gx = rx + bx
        gy = ry + by
        gz = rz + bz

        # Step 4: rotate the LOCAL carry offset by the gripper orientation.
        # The gripper's orientation in base_link is the same as in world
        # (since robot yaw = 0), so we can use the TF rotation directly.
        q = (tf.transform.rotation.x,
             tf.transform.rotation.y,
             tf.transform.rotation.z,
             tf.transform.rotation.w)
        ox, oy, oz = quat_rotate_vector(q, CARRY_OFFSET_LOCAL)

        # Step 5: combine.
        return (gx + ox, gy + oy, gz + oz)

    def _start_tracking(self):
        self._stop_tracking()
        self._track_stop_event = threading.Event()
        self._track_thread = threading.Thread(
            target=self._tracking_loop, daemon=True)
        self._track_thread.start()
        self.get_logger().info(
            f'  Tracking carried fruit at {TRACK_HZ} Hz '
            f'(local offset {CARRY_OFFSET_LOCAL} in {CARRIED_FRUIT_LINK}).')

    def _stop_tracking(self):
        if self._track_thread is not None:
            self._track_stop_event.set()
            self._track_thread.join(timeout=1.0)
            self._track_thread = None
            self.get_logger().info('  Tracking stopped.')

    def _tracking_loop(self):
        period = 1.0 / TRACK_HZ
        while not self._track_stop_event.is_set():
            t0 = time.time()
            if (self.last_carried_fruit_name is not None and rclpy.ok()):
                gx, gy, gz = self._compute_carried_fruit_world_pos()
                if gx is not None:
                    self._gz_set_pose(self.last_carried_fruit_name,
                                       gx, gy, gz)
            elapsed = time.time() - t0
            sleep_for = max(0.0, period - elapsed)
            time.sleep(sleep_for)

    def _gz_set_pose(self, entity_name: str, x: float, y: float, z: float):
        req_str = (
            f'name: "{entity_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        )
        cmd = ['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
               '--reqtype', 'gz.msgs.Pose',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '500', '--req', req_str]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
        except Exception:
            pass

    # =====================================================================
    # Drop fruit spawn
    # =====================================================================
    def _get_robot_world_pose(self):
        try:
            result = subprocess.run(
                ['gz', 'model', '-m', ROBOT_MODEL_NAME, '-p'],
                capture_output=True, text=True, timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f'  gz model exception: {e}')
            return None
        lines = result.stdout.splitlines()
        for i, line in enumerate(lines):
            if 'XYZ (m)' in line and i + 1 < len(lines):
                try:
                    parts = lines[i+1].strip().strip('[]').split()
                    return (float(parts[0]), float(parts[1]), float(parts[2]))
                except Exception:
                    pass
        return None

    def _spawn_dropping_fruit(self, radius: float, is_big: bool) -> bool:
        pose = self._get_robot_world_pose()
        if pose is None:
            self.get_logger().warn(
                '  Falling back to gripper position from TF.')
            gx, gy, gz = self._compute_carried_fruit_world_pos()
            if gx is None:
                return False
            gz -= DROP_OFFSET_BELOW_GRIPPER_M
        else:
            wx, wy, _ = pose
            if is_big:
                gx = wx + BIG_BIN_OFFSET_X
                gy = wy + BIG_BIN_OFFSET_Y
                gz = BIG_BIN_HEIGHT_Z
            else:
                gx = wx + SMALL_BIN_OFFSET_X
                gy = wy + SMALL_BIN_OFFSET_Y
                gz = SMALL_BIN_HEIGHT_Z

        spawn_name = f'dropped_fruit_{int(time.time() * 1000) % 1000000}'
        m = DROP_FRUIT_MASS_KG
        ii = (2.0 / 5.0) * m * (radius ** 2)
        sdf = (
            f'<sdf version="1.7">'
            f'<model name="{spawn_name}">'
            f'<pose>{gx} {gy} {gz} 0 0 0</pose>'
            f'<link name="link">'
            f'<inertial>'
            f'<mass>{m}</mass>'
            f'<inertia><ixx>{ii}</ixx><ixy>0</ixy><ixz>0</ixz>'
            f'<iyy>{ii}</iyy><iyz>0</iyz><izz>{ii}</izz></inertia>'
            f'</inertial>'
            f'<collision name="collision">'
            f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>'
            f'</collision>'
            f'<visual name="visual">'
            f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>'
            f'<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>'
            f'</visual>'
            f'</link>'
            f'</model>'
            f'</sdf>'
        )
        if self._gz_create_entity(sdf):
            bin_label = 'BIG' if is_big else 'SMALL'
            self.get_logger().info(
                f'  Dropped fruit "{spawn_name}" into {bin_label} bin '
                f'at world ({gx:+.2f}, {gy:+.2f}, {gz:+.2f})')
            return True
        return False

    def _delete_entity_by_name(self, entity_name: str):
        if self._gz_remove_entity(entity_name):
            self.get_logger().info(f'  Deleted entity "{entity_name}"')
        else:
            self.get_logger().warn(f'  Delete failed for "{entity_name}"')

    def _gz_remove_entity(self, entity_name: str) -> bool:
        cmd = ['gz', 'service', '-s', f'/world/{WORLD_NAME}/remove',
               '--reqtype', 'gz.msgs.Entity',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '2000',
               '--req', f'name: "{entity_name}", type: 2']
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            return 'data: true' in r.stdout
        except Exception as e:
            self.get_logger().warn(f'  gz remove exception: {e}')
            return False

    def _gz_create_entity(self, sdf_string: str) -> bool:
        sdf_escaped = sdf_string.replace('"', '\\"')
        req_str = f'sdf: "{sdf_escaped}"'
        cmd = ['gz', 'service', '-s', f'/world/{WORLD_NAME}/create',
               '--reqtype', 'gz.msgs.EntityFactory',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '2000', '--req', req_str]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if 'data: true' in r.stdout:
                return True
            self.get_logger().warn(
                f'  gz create failed: stdout={r.stdout!r} stderr={r.stderr!r}')
            return False
        except Exception as e:
            self.get_logger().warn(f'  gz create exception: {e}')
            return False

    # =====================================================================
    # MoveIt helpers
    # =====================================================================
    def _plan_to_pose(self, pose: Pose, frame_id: str = 'base_link',
                       use_orientation: bool = True) -> bool:
        goal = MoveGroup.Goal()
        req = self._make_request()

        pos_con = PositionConstraint()
        pos_con.header.frame_id = frame_id
        pos_con.link_name = END_EFFECTOR_LINK
        pos_con.constraint_region.primitives.append(SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[PLAN_POSITION_TOL_M]))
        pos_con.constraint_region.primitive_poses.append(Pose(
            position=pose.position, orientation=Quaternion(w=1.0)))
        pos_con.weight = 1.0

        gc = Constraints()
        gc.position_constraints.append(pos_con)

        if use_orientation:
            ori_con = OrientationConstraint()
            ori_con.header.frame_id = frame_id
            ori_con.link_name = END_EFFECTOR_LINK
            ori_con.orientation = pose.orientation
            ori_con.absolute_x_axis_tolerance = ORIENT_TOL_X
            ori_con.absolute_y_axis_tolerance = ORIENT_TOL_Y
            ori_con.absolute_z_axis_tolerance = ORIENT_TOL_Z
            ori_con.weight = 0.5
            gc.orientation_constraints.append(ori_con)

        req.goal_constraints.append(gc)
        goal.request = req
        goal.planning_options.plan_only = False
        return self._send_and_wait(goal)

    def _goto_joints(self, joint_values: list) -> bool:
        goal = MoveGroup.Goal()
        req = self._make_request()
        gc = Constraints()
        for name, value in zip(JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            gc.joint_constraints.append(jc)
        req.goal_constraints.append(gc)
        goal.request = req
        goal.planning_options.plan_only = False
        ok = self._send_and_wait(goal)
        if not ok:
            raise RuntimeError('Joint-space motion failed')
        return ok

    def _set_gripper(self, position: float) -> bool:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'end_effector_group'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x = -2.0; ws.min_corner.y = -2.0; ws.min_corner.z = -1.0
        ws.max_corner.x = 2.0; ws.max_corner.y = 2.0; ws.max_corner.z = 3.0
        req.workspace_parameters = ws
        gc = Constraints()
        jc = JointConstraint()
        jc.joint_name = 'Slider_20'
        jc.position = float(position)
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        gc.joint_constraints.append(jc)
        req.goal_constraints.append(gc)
        goal.request = req
        goal.planning_options.plan_only = False
        ok = self._send_and_wait(goal)
        if not ok:
            raise RuntimeError(f'Gripper motion to {position} failed')
        return ok

    def _make_request(self) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x = -2.0; ws.min_corner.y = -2.0; ws.min_corner.z = -1.0
        ws.max_corner.x = 2.0; ws.max_corner.y = 2.0; ws.max_corner.z = 3.0
        req.workspace_parameters = ws
        return req

    def _send_and_wait(self, goal: MoveGroup.Goal) -> bool:
        send_future = self.move_action.send_goal_async(goal)
        deadline = time.time() + GOAL_ACK_TIMEOUT_S
        while not send_future.done() and time.time() < deadline:
            time.sleep(ACTION_POLL_INTERVAL_S)
        if not send_future.done():
            self.get_logger().warn('  MoveGroup goal send timeout')
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('  MoveGroup goal rejected')
            return False
        result_future = handle.get_result_async()
        deadline = time.time() + EXECUTION_TIMEOUT_S
        while not result_future.done() and time.time() < deadline:
            time.sleep(ACTION_POLL_INTERVAL_S)
        if not result_future.done():
            self.get_logger().warn('  MoveGroup did not return a result in time')
            return False
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info('  Motion succeeded.')
            return True
        else:
            self.get_logger().warn(f'  Motion failed with error code {ec}')
            return False

    def _status(self, s: str):
        self.status_pub.publish(String(data=s))


def main():
    rclpy.init()
    node = ArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_tracking()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
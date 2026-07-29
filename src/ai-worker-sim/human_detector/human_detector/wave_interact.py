import copy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_R_JOINT_NAMES = [
    'arm_r_joint1',
    'arm_r_joint2',
    'arm_r_joint3',
    'arm_r_joint4',
    'arm_r_joint5',
    'arm_r_joint6',
    'arm_r_joint7',
]

HOME_ARM_POSITIONS = [0.0] * 7


def radians(values_degrees):
    return [math.radians(value) for value in values_degrees]


# Existing collision-safe poses used by this project.
SIDE_ARM_POSITION = radians([0.0, -60.0, 0.0, -20.0, 0.0, 0.0, 0.0])
ELBOW_UP_POSITION = radians([0.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0])
WRIST_WAVE_DEGREES = 15.0

# Fast trajectory. The largest average segment speed remains below the
# declared 5.0 rad/s arm-joint limit.
WAVE_END_TIME = 2.80


class WaveInteraction(Node):
    STATE_IDLE = 'idle'
    STATE_SENDING = 'sending_goal'
    STATE_NAVIGATING = 'navigating'
    STATE_CANCELLING = 'cancelling_goal'
    STATE_WAVING = 'waving'
    STATE_RETURNING_HOME = 'returning_home'
    STATE_RESUMING = 'resuming_goal'
    STATE_SUCCEEDED = 'succeeded'
    STATE_ABORTED = 'aborted'
    STATE_REJECTED = 'rejected'
    STATE_FAILED = 'failed'

    def __init__(self):
        super().__init__('wave_interaction')

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()

        self.declare_parameter('goal_topic', '/interaction_goal_pose')
        self.declare_parameter('rviz_goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('person_target_max_age', 3.0)
        self.declare_parameter('base_linear_stop_threshold', 0.02)
        self.declare_parameter('base_angular_stop_threshold', 0.03)
        self.declare_parameter('base_stop_stable_duration', 0.40)
        self.declare_parameter('base_stop_timeout', 30.0)
        self.declare_parameter('arm_home_tolerance', 0.08)
        self.declare_parameter('arm_home_stable_duration', 0.30)
        self.declare_parameter('arm_home_timeout', 600.0)
        self.declare_parameter('arm_trajectory_timeout', 600.0)
        self.declare_parameter(
            'arm_controller_action',
            '/arm_r_controller/follow_joint_trajectory',
        )

        self.goal_topic = self.get_parameter('goal_topic').value
        self.rviz_goal_topic = self.get_parameter('rviz_goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.person_target_max_age = float(
            self.get_parameter('person_target_max_age').value
        )
        self.base_linear_stop_threshold = float(
            self.get_parameter('base_linear_stop_threshold').value
        )
        self.base_angular_stop_threshold = float(
            self.get_parameter('base_angular_stop_threshold').value
        )
        self.base_stop_stable_duration = float(
            self.get_parameter('base_stop_stable_duration').value
        )
        self.base_stop_timeout = float(
            self.get_parameter('base_stop_timeout').value
        )
        self.arm_home_tolerance = float(
            self.get_parameter('arm_home_tolerance').value
        )
        self.arm_home_stable_duration = float(
            self.get_parameter('arm_home_stable_duration').value
        )
        self.arm_home_timeout = float(
            self.get_parameter('arm_home_timeout').value
        )
        self.arm_trajectory_timeout = float(
            self.get_parameter('arm_trajectory_timeout').value
        )
        arm_controller_action = self.get_parameter(
            'arm_controller_action'
        ).value

        self.interaction_state = self.STATE_IDLE
        self.original_goal_pose = None
        self.active_goal_handle = None
        self.active_goal_sequence = 0
        self.interaction_done_for_goal = False

        self.person_present = False
        self.latest_person_distance = None
        self.latest_person_distance_time = None
        self.distance_value_logged = False

        self.latest_linear_speed = None
        self.latest_angular_speed = None
        self.latest_joint_positions = {}

        self.worker_thread = None

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )
        self.arm_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            arm_controller_action,
            callback_group=self.callback_group,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.interaction_active_pub = self.create_publisher(
            Bool,
            '/interaction_active',
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._external_goal_cb,
            10,
            callback_group=self.callback_group,
        )

        # This lets the RViz "2D Goal Pose" tool work without another relay.
        if self.rviz_goal_topic != self.goal_topic:
            self.create_subscription(
                PoseStamped,
                self.rviz_goal_topic,
                self._external_goal_cb,
                10,
                callback_group=self.callback_group,
            )

        self.create_subscription(
            Bool,
            '/person_detected',
            self._person_detected_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PointStamped,
            '/person_target',
            self._person_target_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            '/person_distance',
            self._person_distance_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            '/wave_command',
            self._wave_command_cb,
            10,
            callback_group=self.callback_group,
        )

        self._publish_interaction_active(False)
        self.get_logger().info(
            'Ready: receive goal -> navigate -> detect -> cancel and stop -> '
            'wave -> arm home -> resume the saved goal.'
        )
        self.get_logger().info(
            f'Goal topics: {self.goal_topic} and {self.rviz_goal_topic}'
        )

    # ------------------------------------------------------------------
    # State and sensor data
    # ------------------------------------------------------------------
    def _set_state(self, new_state):
        with self.state_lock:
            old_state = self.interaction_state
            self.interaction_state = new_state

        if old_state != new_state:
            self.get_logger().info(
                f'Interaction state: {old_state} -> {new_state}'
            )

    def _publish_interaction_active(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.interaction_active_pub.publish(msg)

    def _odom_cb(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        with self.state_lock:
            self.latest_linear_speed = math.sqrt(
                linear.x ** 2 + linear.y ** 2 + linear.z ** 2
            )
            self.latest_angular_speed = math.sqrt(
                angular.x ** 2 + angular.y ** 2 + angular.z ** 2
            )

    def _joint_state_cb(self, msg):
        with self.state_lock:
            for name, position in zip(msg.name, msg.position):
                self.latest_joint_positions[name] = float(position)

    def _store_person_distance(self, distance):
        if not math.isfinite(distance) or distance <= 0.0:
            return

        should_log = False
        with self.state_lock:
            self.latest_person_distance = float(distance)
            self.latest_person_distance_time = time.monotonic()
            if self.person_present and not self.distance_value_logged:
                self.distance_value_logged = True
                should_log = True

        if should_log:
            self.get_logger().info(
                f'Person detected at distance {distance:.2f} m'
            )

    def _person_distance_cb(self, msg):
        self._store_person_distance(float(msg.data))

    def _person_target_cb(self, msg):
        # Backup source in case /person_distance is not available.
        point = msg.point
        distance = math.sqrt(
            point.x * point.x + point.y * point.y + point.z * point.z
        )
        self._store_person_distance(distance)

    # ------------------------------------------------------------------
    # Navigation goal ownership
    # ------------------------------------------------------------------
    def _external_goal_cb(self, msg):
        if not msg.header.frame_id:
            self.get_logger().error('Ignored goal with an empty frame_id')
            return

        with self.state_lock:
            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RETURNING_HOME,
                self.STATE_RESUMING,
            }:
                self.get_logger().warning(
                    'Ignored a new goal because the current goal or '
                    'interaction is still active'
                )
                return

            self.original_goal_pose = copy.deepcopy(msg)
            self.interaction_done_for_goal = False
            self.person_present = False
            self.distance_value_logged = False

        self._send_saved_goal(is_resume=False)

    def _send_saved_goal(self, is_resume):
        with self.state_lock:
            pose = copy.deepcopy(self.original_goal_pose)

        if pose is None:
            self.get_logger().error('No saved navigation goal exists')
            self._set_state(self.STATE_FAILED)
            return False

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose server is unavailable')
            self._set_state(self.STATE_FAILED)
            return False

        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        with self.state_lock:
            self.active_goal_sequence += 1
            sequence = self.active_goal_sequence
            self.active_goal_handle = None
            self.interaction_state = (
                self.STATE_RESUMING if is_resume else self.STATE_SENDING
            )

        label = 'resumed original goal' if is_resume else 'original goal'
        self.get_logger().info(f'Sending {label}')

        try:
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed: self._goal_response_cb(
                    completed,
                    sequence,
                    is_resume,
                )
            )
        except Exception as error:
            self.get_logger().error(f'Failed to send Nav2 goal: {error}')
            self._set_state(self.STATE_FAILED)
            return False

        return True

    def _goal_response_cb(self, future, sequence, is_resume):
        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to receive Nav2 goal response: {error}'
            )
            self._set_state(self.STATE_FAILED)
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the navigation goal')
            self._set_state(self.STATE_REJECTED)
            return

        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return
            self.active_goal_handle = goal_handle
            self.interaction_state = self.STATE_NAVIGATING
            person_is_already_visible = self.person_present

        if is_resume:
            self.get_logger().info('Nav2 accepted the resumed original goal')
        else:
            self.get_logger().info('Nav2 accepted the original goal')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._nav_result_cb(completed, sequence)
        )

        # A person may have been detected while the goal was still being sent.
        if person_is_already_visible:
            self._try_cancel_for_person()

    def _nav_result_cb(self, future, sequence):
        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return
            state_at_result = self.interaction_state

        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as error:
            self.get_logger().error(f'Failed to read Nav2 result: {error}')
            self._set_state(self.STATE_FAILED)
            return

        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return
            self.active_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded')
            self._publish_interaction_active(False)
            self._set_state(self.STATE_SUCCEEDED)
            return

        if status == GoalStatus.STATUS_CANCELED:
            if state_at_result in {
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RETURNING_HOME,
            }:
                self.get_logger().info(
                    'Navigation goal reached terminal CANCELED state'
                )
                return
            self.get_logger().warning('Navigation goal was canceled')
            self._set_state(self.STATE_IDLE)
            return

        if status == GoalStatus.STATUS_ABORTED:
            if state_at_result in {
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RETURNING_HOME,
            }:
                self.get_logger().warning(
                    'Nav2 reported ABORTED during cancellation; continuing '
                    'the stop-wave-resume sequence'
                )
                return
            self.get_logger().error('Navigation goal was aborted')
            self._set_state(self.STATE_ABORTED)
            return

        self.get_logger().error(
            f'Navigation goal ended with unexpected status {status}'
        )
        self._set_state(self.STATE_FAILED)

    # ------------------------------------------------------------------
    # Person detection -> immediate cancellation
    # ------------------------------------------------------------------
    def _person_detected_cb(self, msg):
        detected = bool(msg.data)

        with self.state_lock:
            was_present = self.person_present
            self.person_present = detected

            if not detected:
                # Do not reset distance_value_logged here. The detector may
                # briefly lose the person while the interaction is running.
                return

            distance = None
            if (
                not self.distance_value_logged
                and self.latest_person_distance is not None
                and self.latest_person_distance_time is not None
                and time.monotonic() - self.latest_person_distance_time
                <= self.person_target_max_age
            ):
                distance = self.latest_person_distance
                self.distance_value_logged = True

        # Log only once. If distance has not arrived yet, the distance
        # callback logs it when the first valid measurement arrives.
        if distance is not None:
            self.get_logger().info(
                f'Person detected at distance {distance:.2f} m'
            )

        # Only the first TRUE edge needs to request cancellation. If the first
        # detection occurred while Nav2 was accepting the goal, the goal
        # response callback performs the same check once the goal is active.
        if not was_present:
            self._try_cancel_for_person()

    def _try_cancel_for_person(self):
        with self.state_lock:
            if not self.person_present:
                return
            if self.interaction_done_for_goal:
                return
            if self.interaction_state != self.STATE_NAVIGATING:
                return
            if self.active_goal_handle is None:
                return

            self.interaction_done_for_goal = True
            self.interaction_state = self.STATE_CANCELLING
            goal_handle = self.active_goal_handle
            sequence = self.active_goal_sequence

        self._publish_interaction_active(True)
        self.get_logger().info(
            'Cancelling the active navigation goal immediately'
        )

        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._after_cancel_request(
                    completed,
                    sequence,
                )
            )
        except Exception as error:
            self.get_logger().error(
                f'Failed to request navigation cancellation: {error}'
            )
            with self.state_lock:
                if sequence == self.active_goal_sequence:
                    self.interaction_done_for_goal = False
                    self.interaction_state = self.STATE_NAVIGATING
            self._publish_interaction_active(False)

    def _after_cancel_request(self, future, sequence):
        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Navigation cancellation request failed: {error}'
            )
            self._restore_navigation_state_after_cancel_failure(sequence)
            return

        if response is None or not response.goals_canceling:
            self.get_logger().error(
                'Nav2 did not accept cancellation of the active goal'
            )
            self._restore_navigation_state_after_cancel_failure(sequence)
            return

        self.get_logger().info(
            'Nav2 accepted cancellation; stopping the base before waving'
        )
        self._start_worker(self._stop_wave_resume_worker, sequence)

    def _restore_navigation_state_after_cancel_failure(self, sequence):
        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return
            self.interaction_done_for_goal = False
            self.interaction_state = self.STATE_NAVIGATING
        self._publish_interaction_active(False)

    # ------------------------------------------------------------------
    # Stop -> wave -> home -> resume
    # ------------------------------------------------------------------
    def _start_worker(self, target, *args):
        with self.state_lock:
            if self.worker_thread is not None and self.worker_thread.is_alive():
                self.get_logger().warning('Interaction worker already running')
                return False
            self.worker_thread = threading.Thread(
                target=target,
                args=args,
                daemon=True,
            )
            self.worker_thread.start()
        return True

    def _stop_wave_resume_worker(self, sequence):
        if not self._force_and_confirm_base_stop():
            self.get_logger().error(
                'The base did not stop; skipping the wave and resuming the goal'
            )
            self._publish_interaction_active(False)
            self._resume_original_goal()
            return

        with self.state_lock:
            if sequence != self.active_goal_sequence:
                return
        self._set_state(self.STATE_WAVING)

        wave_success = self.do_wave()

        self._set_state(self.STATE_RETURNING_HOME)
        home_success = self._wait_until_arm_home(timeout=self.arm_home_timeout)
        if not home_success:
            self.get_logger().warning(
                'Wave did not finish at home; sending a separate home command'
            )
            home_success = self._command_arm_home()

        if not wave_success:
            self.get_logger().error('Wave trajectory did not complete normally')

        if not home_success:
            self.get_logger().error(
                'Arm is not home; navigation will not resume for safety'
            )
            self._publish_interaction_active(False)
            self._set_state(self.STATE_FAILED)
            return

        self.get_logger().info(
            'Wave finished and the right arm is home; continuing the saved goal'
        )
        self._publish_interaction_active(False)
        self._resume_original_goal()

    def _resume_original_goal(self):
        with self.state_lock:
            if self.original_goal_pose is None:
                self.get_logger().error('Cannot resume: saved goal is missing')
                self.interaction_state = self.STATE_FAILED
                return False
            self.interaction_state = self.STATE_RESUMING

        return self._send_saved_goal(is_resume=True)

    def _force_and_confirm_base_stop(self):
        zero = Twist()
        for _ in range(30):
            self.cmd_vel_pub.publish(zero)
            time.sleep(0.05)

        deadline = time.monotonic() + self.base_stop_timeout
        stable_since = None
        odom_was_seen = False

        while rclpy.ok() and time.monotonic() < deadline:
            with self.state_lock:
                linear = self.latest_linear_speed
                angular = self.latest_angular_speed

            if linear is None or angular is None:
                time.sleep(0.05)
                continue

            odom_was_seen = True
            stopped = (
                linear <= self.base_linear_stop_threshold
                and angular <= self.base_angular_stop_threshold
            )

            if stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self.base_stop_stable_duration:
                    self.get_logger().info('Base is confirmed stationary')
                    return True
            else:
                stable_since = None
                self.cmd_vel_pub.publish(zero)

            time.sleep(0.05)

        if not odom_was_seen:
            self.get_logger().warning(
                'No odometry was received; proceeding because Nav2 accepted '
                'cancellation and zero velocity was commanded'
            )
            return True

        return False

    # ------------------------------------------------------------------
    # Fast direct FollowJointTrajectory wave
    # ------------------------------------------------------------------
    def do_wave(self):
        current = self._wait_for_arm_positions(15.0)
        if current is None:
            self.get_logger().error(
                'Cannot wave because right-arm joint states are unavailable'
            )
            return False

        wave_left = list(ELBOW_UP_POSITION)
        wave_right = list(ELBOW_UP_POSITION)
        wave_left[5] = math.radians(-WRIST_WAVE_DEGREES)
        wave_right[5] = math.radians(WRIST_WAVE_DEGREES)

        # 2.80 seconds requested total time. The trajectory ends at zero/home.
        sequence = [
            (0.05, current),
            (0.45, SIDE_ARM_POSITION),
            (0.95, ELBOW_UP_POSITION),
            (1.15, wave_left),
            (1.35, wave_right),
            (1.55, wave_left),
            (1.75, wave_right),
            (1.95, ELBOW_UP_POSITION),
            (WAVE_END_TIME, HOME_ARM_POSITIONS),
        ]

        self.get_logger().info(
            'Sending maximum-speed wave trajectory: 2 cycles, 2.80 s, '
            'ending at arm home'
        )
        return self._execute_arm_trajectory(
            sequence=sequence,
            description='wave',
            expected_end_time=WAVE_END_TIME,
            timeout=self.arm_trajectory_timeout,
            require_movement=True,
        )

    def _command_arm_home(self):
        current = self._wait_for_arm_positions(15.0)
        if current is None:
            return False

        if self._arm_is_home_now():
            return self._wait_until_arm_home(timeout=2.0)

        self.get_logger().info('Sending right arm home recovery trajectory')
        return self._execute_arm_trajectory(
            sequence=[
                (0.05, current),
                (1.20, HOME_ARM_POSITIONS),
            ],
            description='arm-home recovery',
            expected_end_time=1.20,
            timeout=self.arm_home_timeout,
            require_movement=False,
        )

    def _execute_arm_trajectory(
        self,
        sequence,
        description,
        expected_end_time,
        timeout,
        require_movement,
    ):
        if not self.arm_trajectory_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'Right-arm FollowJointTrajectory server is unavailable'
            )
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_R_JOINT_NAMES)
        trajectory.points = [
            self._trajectory_point(positions, seconds)
            for seconds, positions in sequence
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = min(int(timeout), 2147483647)

        send_future = self.arm_trajectory_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 15.0):
            self.get_logger().error(
                f'Timed out sending {description} trajectory'
            )
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Arm controller rejected {description} trajectory'
            )
            return False

        self.get_logger().info(
            f'Arm controller accepted {description} trajectory'
        )

        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        deadline = start_time + timeout
        initial_positions = list(sequence[0][1])
        movement_seen = not require_movement
        home_stable_since = None

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            current = self._current_arm_positions()

            if current is not None and not movement_seen:
                movement_seen = any(
                    abs(current[index] - initial_positions[index]) > 0.08
                    for index in range(len(ARM_R_JOINT_NAMES))
                )

            if (
                now - start_time >= expected_end_time
                and movement_seen
                and self._arm_is_home_now()
            ):
                if home_stable_since is None:
                    home_stable_since = now
                elif now - home_stable_since >= self.arm_home_stable_duration:
                    self.get_logger().info(
                        f'{description} completed; arm reached home'
                    )
                    # Some simulation controllers delay the action result even
                    # after the final point is reached. Clear that stale goal.
                    if not result_future.done():
                        cancel_future = goal_handle.cancel_goal_async()
                        self._wait_for_future(cancel_future, 2.0)
                    return True
            else:
                home_stable_since = None

            if result_future.done():
                wrapped = result_future.result()
                if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error(
                        f'{description} action ended with status '
                        f'{wrapped.status}'
                    )
                    return False

                result = wrapped.result
                if (
                    result.error_code
                    != FollowJointTrajectory.Result.SUCCESSFUL
                ):
                    self.get_logger().error(
                        f'{description} controller error '
                        f'{result.error_code}: {result.error_string}'
                    )
                    return False

                self.get_logger().info(
                    f'Arm controller reported {description} success'
                )
                return self._wait_until_arm_home(
                    timeout=self.arm_home_timeout
                )

            time.sleep(0.02)

        self.get_logger().error(
            f'{description} exceeded the 10-minute timeout'
        )
        cancel_future = goal_handle.cancel_goal_async()
        self._wait_for_future(cancel_future, 2.0)
        return False

    def _current_arm_positions(self):
        with self.state_lock:
            if not all(
                name in self.latest_joint_positions
                for name in ARM_R_JOINT_NAMES
            ):
                return None
            return [
                self.latest_joint_positions[name]
                for name in ARM_R_JOINT_NAMES
            ]

    def _wait_for_arm_positions(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            positions = self._current_arm_positions()
            if positions is not None:
                return positions
            time.sleep(0.05)
        return None

    def _arm_is_home_now(self):
        positions = self._current_arm_positions()
        if positions is None:
            return False
        return all(
            abs(position) <= self.arm_home_tolerance
            for position in positions
        )

    def _wait_until_arm_home(self, timeout):
        deadline = time.monotonic() + timeout
        stable_since = None

        while rclpy.ok() and time.monotonic() < deadline:
            if self._arm_is_home_now():
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (
                    time.monotonic() - stable_since
                    >= self.arm_home_stable_duration
                ):
                    self.get_logger().info(
                        'Right arm is confirmed at home'
                    )
                    return True
            else:
                stable_since = None
            time.sleep(0.05)

        return False

    @staticmethod
    def _trajectory_point(positions, seconds):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        whole_seconds = int(seconds)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            round((seconds - whole_seconds) * 1.0e9)
        )
        if point.time_from_start.nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1_000_000_000
        return point

    @staticmethod
    def _wait_for_future(future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    # ------------------------------------------------------------------
    # Optional standalone wave test
    # ------------------------------------------------------------------
    def _wave_command_cb(self, msg):
        if not msg.data:
            return

        with self.state_lock:
            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RETURNING_HOME,
                self.STATE_RESUMING,
            }:
                self.get_logger().warning(
                    'Standalone wave ignored while navigation is active'
                )
                return

        self._start_worker(self._standalone_wave_worker)

    def _standalone_wave_worker(self):
        self._publish_interaction_active(True)
        if not self._force_and_confirm_base_stop():
            self._publish_interaction_active(False)
            return

        self._set_state(self.STATE_WAVING)
        success = self.do_wave()
        self._publish_interaction_active(False)
        self._set_state(self.STATE_IDLE if success else self.STATE_FAILED)


def main(args=None):
    rclpy.init(args=args)
    node = WaveInteraction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
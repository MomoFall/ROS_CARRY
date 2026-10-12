"""
Node follow_manager: state machine, control law, avoidance, way back, safety, beeps.

It is the only node that publishes on /cmd_vel. At every cycle (20 Hz) it
checks safety first (S2 to S5), updates the state machine, then computes the
velocity command of the current state: following the operator around the
obstacles (VFH), searching, or driving back to the start pose.
"""

import math

from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from . import markers
from .core.avoidance import avoid, AvoidanceParams
from .core.clustering import scan_to_points
from .core.controller import (align_command, ControllerParams, follow_command, goto_command,
                              limit_acceleration, saturate, search_command, steer)
from .core.geometry import (min_range_in_sector, normalize_angle, points_to_local,
                            points_to_world, polar, to_local, yaw_from_quaternion)
from .core.state_machine import (ACTIVE_STATES, Event, FollowStateMachine, Inputs,
                                 SafetyStatus, State)

try:
    from turtlebot3_msgs.srv import Sound
except ImportError:  # turtlebot3_msgs is not installed: beeps are only logged
    Sound = None

STATE_COLORS = {
    State.IDLE: markers.GREY,
    State.ACQUIRE: markers.YELLOW,
    State.FOLLOW: markers.GREEN,
    State.SEARCH: markers.ORANGE,
    State.STOPPED: markers.BLUE,
    State.RETURN: markers.PURPLE,
    State.HOME: markers.WHITE,
}


class FollowManagerNode(Node):
    """Decide what the robot does and send the only velocity command."""

    def __init__(self):
        super().__init__('follow_manager')
        self.ctrl = ControllerParams(
            follow_distance=self._param('follow_distance', 0.8),
            min_follow_distance=self._param('min_follow_distance', 0.5),
            k_v=self._param('k_v', 1.5),
            k_w=self._param('k_w', 1.5),
            max_linear_speed=self._param('max_linear_speed', 0.22),
            max_angular_speed=self._param('max_angular_speed', 1.5),
            turn_slowdown_angle=math.radians(self._param('turn_slowdown_angle_deg', 60.0)),
            distance_deadband=self._param('distance_deadband', 0.05),
            angle_deadband=math.radians(self._param('angle_deadband_deg', 3.0)),
            search_angular_speed=self._param('search_angular_speed', 0.8),
            return_speed=self._param('return_speed', 0.15))
        self.avoidance = self._param('avoidance', True)
        self.avoid_params = AvoidanceParams(
            window=self._param('avoid_window', 1.0),
            robot_radius=self._param('robot_radius', 0.11),
            clearance=self._param('avoid_clearance', 0.08),
            sector=math.radians(self._param('avoid_sector_deg', 5.0)),
            goal_clearance=self._param('avoid_goal_clearance', 0.35),
            max_detour=math.radians(self._param('avoid_max_detour_deg', 100.0)),
            hysteresis=self._param('avoid_hysteresis', 0.5))
        self.lidar_offset_x = self._param('lidar_offset_x', -0.032)
        return_home = self._param('return_home', True)
        return_delay = self._param('return_delay', 5.0)
        return_timeout = self._param('return_timeout', 120.0)
        self.home_tolerance = self._param('home_tolerance', 0.10)
        self.align_at_home = self._param('align_at_home', True)
        self.home_yaw_tolerance = math.radians(self._param('home_yaw_tolerance_deg', 5.0))
        self.max_linear_accel = self._param('max_linear_accel', 0.5)
        self.rate = self._param('control_rate', 20.0)
        self.target_timeout = self._param('target_timeout', 0.3)
        self.odom_timeout = self._param('odom_timeout', 0.5)
        self.safety_distance = self._param('safety_distance', 0.15)
        self.safety_half_angle = math.radians(self._param('safety_half_angle_deg', 35.0))
        self.scan_timeout = self._param('scan_timeout', 0.5)
        self.battery_check = self._param('battery_check', True)
        self.battery_critical_voltage = self._param('battery_critical_voltage', 11.0)
        self.battery_filter = self._param('battery_filter', 0.1)
        use_sound = self._param('use_sound', True)
        self.sound_beep = self._param('sound_beep', 4)
        self.sound_error = self._param('sound_error', 3)
        self.sound_low_battery = self._param('sound_low_battery', 2)
        self.beep_interval = self._param('beep_interval', 0.5)
        self.sm = FollowStateMachine(self._param('search_timeout', 15.0), self.now(),
                                     return_delay if return_home else None, return_timeout)

        self.last_scan_time = None
        self.scan_frame = 'base_scan'
        self.front_range = math.inf
        self.scan_checked = False
        self.scan_points = np.empty((0, 2))  # last scan, odom frame
        self.heading_odom = None  # current detour direction, odom frame
        self.avoid_view = None  # (heading, goal angle, sectors, blocked) for RViz
        self.robot_pose = None
        self.odom_time = None
        self.home_pose = None
        self.at_home_position = False
        self.target = None
        self.target_time = None
        self.battery_voltage = None
        self.emergency = False
        self.stop_requested = False
        self.last_v = 0.0
        self.last_cycle = None
        self.published_state = None
        self.state_pub_time = None
        self.danger_reasons = []
        self.beeps = []
        self.cycle = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'follow_state', state_qos)
        self.polar_pub = self.create_publisher(Vector3Stamped, 'target_polar', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'follow_markers', 10)

        self.create_subscription(PointStamped, 'target', self.on_target, 10)
        self.create_subscription(Empty, 'stop_gesture', self.on_stop_gesture, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery,
                                 qos_profile_sensor_data)

        self.create_service(Trigger, 'stop_follow', self.on_stop_follow)
        self.create_service(Trigger, 'emergency_stop', self.on_emergency_stop)
        self.create_service(Trigger, 'start_follow', self.on_start_follow)

        self.sound_client = None
        if use_sound and Sound is not None:
            self.sound_client = self.create_client(Sound, 'sound')
        elif use_sound:
            self.get_logger().warn('turtlebot3_msgs absent : les bips seront seulement affichés')

        self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info('follow_manager prêt, état IDLE')

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs

    def on_target(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.target_time = self.now()

    def on_stop_gesture(self, _msg):
        self.get_logger().info("Geste d'arrêt reçu")
        self.stop_requested = True

    def on_scan(self, scan):
        self.last_scan_time = self.now()
        self.scan_frame = scan.header.frame_id or self.scan_frame
        self.front_range = min_range_in_sector(scan.ranges, scan.angle_min,
                                               scan.angle_increment, scan.range_min,
                                               scan.range_max, self.safety_half_angle)
        if self.robot_pose is not None:
            # Kept in the odom frame: the robot moves (and turns) until the next scan.
            points, _ = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                       scan.range_min, scan.range_max)
            points[:, 0] += self.lidar_offset_x
            self.scan_points = points_to_world(points, self.robot_pose)
        if not self.scan_checked:
            self.scan_checked = True
            if self.safety_distance <= scan.range_min:
                self.get_logger().warn(
                    f'safety_distance ({self.safety_distance:.2f} m) <= portée minimale du '
                    f'LiDAR ({scan.range_min:.2f} m) : la règle S2 ne peut pas se '
                    f'déclencher, augmenter safety_distance')

    def on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        self.robot_pose = (pose.position.x, pose.position.y,
                           yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.odom_time = self.now()
        if self.home_pose is None:
            self.home_pose = self.robot_pose
            self.get_logger().info(
                f'Pose de départ enregistrée (F1.1) : x={pose.position.x:.2f} '
                f'y={pose.position.y:.2f}')

    def on_battery(self, msg):
        voltage = msg.voltage
        if not math.isfinite(voltage) or voltage <= 0.0:
            return
        if self.battery_voltage is None:
            self.battery_voltage = voltage
        else:
            self.battery_voltage += self.battery_filter * (voltage - self.battery_voltage)

    # ---------------------------------------------------------------- services

    def on_stop_follow(self, _request, response):
        self.stop_requested = True
        response.success = True
        response.message = 'Arrêt du suivi demandé'
        return response

    def on_emergency_stop(self, _request, response):
        self.emergency = True
        self.publish_cmd(0.0, 0.0)
        self.get_logger().warn("Arrêt d'urgence ! /start_follow pour relancer")
        response.success = True
        response.message = "Arrêt d'urgence activé"
        return response

    def on_start_follow(self, _request, response):
        self.emergency = False
        if self.sm.state in (State.STOPPED, State.HOME):
            self.sm.handle(Event.RESET, self.now())
            self.target = None
            self.target_time = None
            if self.robot_pose is not None:
                self.home_pose = self.robot_pose  # the new mission starts here (F1.1)
                self.get_logger().info(
                    f'Nouvelle pose de départ : x={self.home_pose[0]:.2f} '
                    f'y={self.home_pose[1]:.2f}')
        response.success = True
        response.message = 'Mission (re)lancée'
        return response

    # ------------------------------------------------------------- main loop

    def safety_status(self, now):
        """Evaluate the safety conditions S2 to S5."""
        scan_ok = (self.last_scan_time is not None
                   and now - self.last_scan_time <= self.scan_timeout)
        battery_low = (self.battery_check and self.battery_voltage is not None
                       and self.battery_voltage < self.battery_critical_voltage)
        return SafetyStatus(
            obstacle=scan_ok and self.front_range < self.safety_distance,
            scan_timeout=not scan_ok,
            battery_critical=battery_low,
            emergency_stop=self.emergency)

    def on_timer(self):
        now = self.now()
        dt = 1.0 / self.rate if self.last_cycle is None else max(now - self.last_cycle, 0.0)
        self.last_cycle = now

        safety = self.safety_status(now)
        target_visible = (self.target_time is not None
                          and now - self.target_time <= self.target_timeout)
        stop, self.stop_requested = self.stop_requested, False
        home_reached = self.sm.state == State.RETURN and self.home_reached()
        inputs = Inputs(now, safety, target_visible, stop, home_reached)
        for event, previous, state in self.sm.step(inputs):
            self.on_transition(event, previous, state, safety)
        self.report_danger(safety)

        v, w = self.command(now, safety)
        self.publish_cmd(limit_acceleration(v, self.last_v, self.max_linear_accel, dt), w)
        self.publish_state(now)
        self.play_due_beeps(now)
        self.cycle += 1
        if self.cycle % 4 == 0:
            self.publish_markers(safety)

    def command(self, now, safety):
        """Return (v, w) for the current state (zero when idle, stopped or home)."""
        state = self.sm.state
        self.avoid_view = None
        if safety.danger or state not in (State.FOLLOW, State.SEARCH, State.RETURN):
            return 0.0, 0.0
        odom_ok = self.odom_time is not None and now - self.odom_time <= self.odom_timeout
        if not odom_ok or (state != State.RETURN and self.target is None):
            self.get_logger().warn('Odométrie ou cible absente : arrêt',
                                   throttle_duration_sec=2.0)
            return 0.0, 0.0
        if state == State.RETURN:
            return self.return_command()
        goal = to_local(self.target, self.robot_pose)
        distance, angle = polar(goal)
        if state == State.SEARCH:
            return search_command(angle, self.ctrl)
        self.publish_polar(distance, angle)
        heading = None
        if self.avoidance and distance - self.ctrl.follow_distance > self.ctrl.distance_deadband:
            # The robot has to move toward the operator: go around what is in between.
            heading = self.avoid_heading(goal, ignore_goal=True)
            if heading is None:
                self.get_logger().warn("Passage bloqué vers l'opérateur : attente",
                                       throttle_duration_sec=2.0)
                return steer(0.0, angle, self.ctrl)
        return follow_command(distance, angle, self.ctrl, heading)

    def return_command(self):
        """Return (v, w) that drive back to the start pose, around the obstacles."""
        if self.home_pose is None:
            return 0.0, 0.0
        if self.at_home_position:
            if not self.align_at_home:
                return 0.0, 0.0
            return align_command(self.home_pose[2] - self.robot_pose[2], self.ctrl)
        goal = to_local(self.home_pose[:2], self.robot_pose)
        distance, angle = polar(goal)
        heading = None
        if self.avoidance:
            heading = self.avoid_heading(goal, ignore_goal=False)
            if heading is None:
                self.get_logger().warn('Chemin du retour bloqué : attente',
                                       throttle_duration_sec=2.0)
                return steer(0.0, angle, self.ctrl)
        return goto_command(distance, angle, self.ctrl, heading)

    def avoid_heading(self, goal, ignore_goal):
        """Return the heading toward goal (robot frame) around the obstacles, or None."""
        yaw = self.robot_pose[2]
        points = points_to_local(self.scan_points, self.robot_pose)
        previous = (None if self.heading_odom is None
                    else normalize_angle(self.heading_odom - yaw))
        heading, centres, blocked = avoid(points, goal, self.avoid_params, previous,
                                          ignore_goal)
        goal_angle = math.atan2(goal[1], goal[0])
        detour = heading is not None and heading != goal_angle
        self.heading_odom = heading + yaw if detour else None
        self.avoid_view = (heading, goal_angle, centres, blocked)
        return heading

    def home_reached(self):
        """Tell whether the robot is back on its start position (and heading)."""
        if self.home_pose is None or self.robot_pose is None:
            return False
        if math.dist(self.home_pose[:2], self.robot_pose[:2]) < self.home_tolerance:
            self.at_home_position = True
        if not self.at_home_position:
            return False
        yaw_error = normalize_angle(self.home_pose[2] - self.robot_pose[2])
        return not self.align_at_home or abs(yaw_error) < self.home_yaw_tolerance

    def on_transition(self, event, previous, state, safety):
        """Log the transition and play the matching beeps."""
        text = f'{previous.value} -> {state.value} ({event.value})'
        if event == Event.DANGER:
            self.get_logger().warn(f"{text} : {', '.join(safety.reasons())}")
        else:
            self.get_logger().info(text)
        if previous == State.ACQUIRE and state == State.FOLLOW:
            self.beep(self.sound_beep)  # F1.3 : opérateur verrouillé
        elif state == State.STOPPED and previous in ACTIVE_STATES:
            self.beep(self.sound_beep, count=2)  # F4.3 : mission terminée
            self.get_logger().info('Mission terminée : bagage livré')
        elif state == State.STOPPED:
            self.beep(self.sound_error)
            self.get_logger().warn('Retour au point de départ annulé ou impossible')
        elif state == State.RETURN:
            self.at_home_position = False
            self.heading_odom = None
            self.beep(self.sound_beep)
            self.get_logger().info('Retour au point de départ')
        elif state == State.HOME:
            self.beep(self.sound_beep, count=2)
            self.get_logger().info('Arrivé au point de départ')
        elif event == Event.SEARCH_TIMEOUT:
            self.beep(self.sound_error)  # F2.5 : opérateur introuvable
        elif event == Event.DANGER and safety.emergency_stop:
            self.beep(self.sound_error)
        elif event == Event.DANGER and safety.battery_critical:
            self.beep(self.sound_low_battery)

    def report_danger(self, safety):
        """Log once every change of the active safety conditions."""
        reasons = safety.reasons()
        if reasons != self.danger_reasons:
            if reasons:
                self.get_logger().warn(f"Sécurité : {', '.join(reasons)} -> vitesse nulle")
            else:
                self.get_logger().info('Sécurité : conditions normales')
            self.danger_reasons = reasons

    # ---------------------------------------------------------------- outputs

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(saturate(v, 0.0, self.ctrl.max_linear_speed))
        msg.angular.z = float(saturate(w, -self.ctrl.max_angular_speed,
                                       self.ctrl.max_angular_speed))
        self.cmd_pub.publish(msg)
        self.last_v = msg.linear.x

    def publish_state(self, now):
        state = self.sm.state.value
        if state != self.published_state or now - self.state_pub_time >= 1.0:
            self.state_pub.publish(String(data=state))
            self.published_state = state
            self.state_pub_time = now

    def publish_polar(self, distance, angle):
        """Publish the distance and bearing of the operator, for PlotJuggler."""
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.vector.x = float(distance)
        msg.vector.y = float(angle)
        msg.vector.z = float(distance - self.ctrl.follow_distance)
        self.polar_pub.publish(msg)

    def beep(self, value, count=1):
        now = self.now()
        self.beeps.extend((now + k * self.beep_interval, value) for k in range(count))

    def play_due_beeps(self, now):
        due = [value for when, value in self.beeps if when <= now]
        self.beeps = [(when, value) for when, value in self.beeps if when > now]
        for value in due:
            if self.sound_client is not None and self.sound_client.service_is_ready():
                request = Sound.Request()
                request.value = int(value)
                self.sound_client.call_async(request)
            else:
                self.get_logger().info(f'Bip ({value})')

    def avoidance_markers(self):
        """Show the blocked sectors (red dots) and the chosen heading of the avoidance."""
        if self.avoid_view is None:
            return []
        heading, goal_angle, centres, blocked = self.avoid_view
        radius = 0.4
        dots = [(radius * math.cos(a), radius * math.sin(a)) for a in centres[blocked]]
        result = [markers.points('base_link', 'avoidance', 0, dots, markers.RED, size=0.03,
                                 z=0.2)]
        if heading is not None:
            color = markers.GREEN if heading == goal_angle else markers.ORANGE
            tip = (0.6 * math.cos(heading), 0.6 * math.sin(heading))
            result.append(markers.line_strip('base_link', 'avoidance', 1, [(0.0, 0.0), tip],
                                             color, width=0.02, z=0.2))
        return result

    def publish_markers(self, safety):
        """Show the state, the safety zone, the avoidance and the start pose."""
        state = self.sm.state
        label = state.value
        if safety.danger:
            label += ' [' + ', '.join(safety.reasons()) + ']'
        array = MarkerArray()
        array.markers.append(markers.text('base_link', 'state', 0, 0.0, 0.0, 0.45, label,
                                          STATE_COLORS[state], size=0.1))
        zone = markers.sector(self.safety_distance, self.safety_half_angle)
        color = markers.RED if safety.obstacle else markers.GREY
        array.markers.append(markers.line_strip(self.scan_frame, 'safety', 0, zone, color,
                                                width=0.008))
        array.markers.extend(self.avoidance_markers())
        if self.home_pose is not None:
            array.markers.append(markers.cylinder('odom', 'home', 0, self.home_pose[0],
                                                  self.home_pose[1], 0.12, 0.02,
                                                  markers.WHITE, lifetime=1.0))
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FollowManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)  # never leave the robot moving
        except Exception:  # the context may already be shut down
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

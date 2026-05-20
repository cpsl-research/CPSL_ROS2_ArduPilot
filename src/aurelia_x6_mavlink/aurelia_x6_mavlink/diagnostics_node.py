import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

_GPS_FIX_LABELS = {-1: 'NO_FIX', 0: 'FIX', 1: 'SBAS_FIX', 2: 'GBAS_FIX'}

HEARTBEAT_TIMEOUT_S = 5.0


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        self._latest_state: State | None = None
        self._latest_odom: Odometry | None = None
        self._latest_gps: NavSatFix | None = None
        self._last_state_time: float = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, '/mavros/state', self._state_callback, qos)
        self.create_subscription(Odometry, 'odom', self._odom_callback, qos)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self._gps_callback, qos)

        self.create_timer(1.0, self._status_timer)
        self.create_timer(5.0, self._gps_timer)

    # ------------------------------------------------------------------ #

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg
        self._last_state_time = time.time()

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._latest_gps = msg

    # ------------------------------------------------------------------ #

    def _status_timer(self) -> None:
        if time.time() - self._last_state_time > HEARTBEAT_TIMEOUT_S:
            self.get_logger().warn(
                f'No MAVLink heartbeat received for >{HEARTBEAT_TIMEOUT_S:.0f} s'
            )

        if self._latest_state is not None:
            s = self._latest_state
            self.get_logger().info(
                f'FC state — connected: {s.connected}  armed: {s.armed}  '
                f'guided: {s.guided}  mode: {s.mode}'
            )

        if self._latest_odom is not None:
            p = self._latest_odom.pose.pose.position
            self.get_logger().info(
                f'Position — x: {p.x:.3f}  y: {p.y:.3f}  z: {p.z:.3f}'
            )

    def _gps_timer(self) -> None:
        if self._latest_gps is None:
            self.get_logger().info('GPS: no message received yet')
            return
        fix = self._latest_gps.status.status
        label = _GPS_FIX_LABELS.get(fix, str(fix))
        self.get_logger().info(
            f'GPS fix: {label} ({fix})  '
            f'lat: {self._latest_gps.latitude:.6f}  '
            f'lon: {self._latest_gps.longitude:.6f}'
        )


# --------------------------------------------------------------------------- #


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

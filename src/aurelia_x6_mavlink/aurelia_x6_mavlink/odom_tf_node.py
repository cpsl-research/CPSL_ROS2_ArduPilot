import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
import numpy as np


class OdomTfNode(Node):
    def __init__(self):
        super().__init__('odom_tf_node')

        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = ''
            self.tf_prefix = ''
        else:
            self.tf_prefix = self.namespace.lstrip('/') + '/'

        self._latest_odom: Odometry | None = None
        self._gps_fix = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self._odom_callback,
            qos,
        )
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            qos,
        )

        self.create_timer(0.1, self._tf_timer_callback)

    # ------------------------------------------------------------------ #

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.tf_prefix + 'odom'
        out.child_frame_id = self.tf_prefix + 'base_link'
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._gps_fix = msg.status.status >= NavSatStatus.STATUS_FIX

    # ------------------------------------------------------------------ #

    def _tf_timer_callback(self) -> None:
        if self._latest_odom is None:
            return

        now = self.get_clock().now().to_msg()
        odom = self._latest_odom
        pos = odom.pose.pose.position
        q = odom.pose.pose.orientation

        # odom → base_link (full 6DOF)
        t_bl = TransformStamped()
        t_bl.header.stamp = now
        t_bl.header.frame_id = self.tf_prefix + 'odom'
        t_bl.child_frame_id = self.tf_prefix + 'base_link'
        t_bl.transform.translation.x = pos.x
        t_bl.transform.translation.y = pos.y
        t_bl.transform.translation.z = pos.z
        t_bl.transform.rotation = q

        # odom → base_footprint (yaw-only, z=0)
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        qyaw = Rotation.from_euler('z', yaw).as_quat()
        t_bf = TransformStamped()
        t_bf.header.stamp = now
        t_bf.header.frame_id = self.tf_prefix + 'odom'
        t_bf.child_frame_id = self.tf_prefix + 'base_footprint'
        t_bf.transform.translation.x = pos.x
        t_bf.transform.translation.y = pos.y
        t_bf.transform.translation.z = 0.0
        t_bf.transform.rotation.x = qyaw[0]
        t_bf.transform.rotation.y = qyaw[1]
        t_bf.transform.rotation.z = qyaw[2]
        t_bf.transform.rotation.w = qyaw[3]

        transforms = [t_bl, t_bf]

        # map → odom (identity; only when GPS fix is available)
        if self._gps_fix:
            t_mo = TransformStamped()
            t_mo.header.stamp = now
            t_mo.header.frame_id = self.tf_prefix + 'map'
            t_mo.child_frame_id = self.tf_prefix + 'odom'
            t_mo.transform.rotation.w = 1.0
            transforms.append(t_mo)

        self.tf_broadcaster.sendTransform(transforms)


# --------------------------------------------------------------------------- #


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

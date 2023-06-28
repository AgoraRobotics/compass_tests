import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler

VIVE_WORLD = 'libsurvive_world'
MAP_FRAME = 'map'
TRACKER = 'LHR-97093BF8'
CALIB_DIST = 1      # move at least 1m to align frames


def vect3_dist(p1, p2):
    return math.dist((p1.x, p1.y), (p2.x, p2.y))

def vect3_angle(p1, p2):
    return math.atan2(p2.x - p1.x, p2.y - p1.y)

def rotate(p, origin=(0, 0), angle=0):
    # angle = np.deg2rad(degrees)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    o = np.atleast_2d(origin)
    p = np.atleast_2d(p)
    return np.squeeze((R @ (p.T-o.T) + o.T).T)


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `libsurvive_world` to map frame.
    The transforms are only published once at startup, after moving 1 meter, and are 
    constant for all time.
    """

    def __init__(self):
        super().__init__('vive_frame_sync_broadcaster')

        # Declare and acquire `target_frame` parameter
        self.tracker = self.declare_parameter(
          'tracker', TRACKER).get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.pos0_odom = None
        self.pos0_vive = None
        self.sync_finished = False

        self.timer = self.create_timer(1.0, self.on_timer)

        # Publish static transforms once at startup
        self.make_transforms([0, 0, 0, 0, 0, 0])

        self.publisher_odom = self.create_publisher(Odometry, 'odom_vive', 10)

    def on_timer(self):
        from_frame_rel = VIVE_WORLD
        to_frame_odom = 'base_link'
        to_frame_vive = self.tracker

        try:
            pos_odom = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                to_frame_odom,
                rclpy.time.Time()).transform.translation
            tf_vive = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_vive,
                rclpy.time.Time()).transform
            pos_vive = tf_vive.translation

            if not self.sync_finished:
                print(pos_odom, '\n\\', pos_vive, '\n')

            if self.pos0_odom is None:
                self.pos0_odom = pos_odom
                self.pos0_vive = pos_vive

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')
            return

        if self.pos0_odom:
            d = vect3_dist(self.pos0_odom, pos_odom)
            angle_odom = vect3_angle(self.pos0_odom, pos_odom)
            angle_vive = vect3_angle(self.pos0_vive, pos_vive)
            if d > CALIB_DIST and not self.sync_finished:
                print('=>', d, angle_odom, '|', angle_vive)
                self.angle_diff = angle_vive - angle_odom
                print('angle_diff', self.angle_diff)
                rot_vive = rotate([(pos_vive.x, pos_vive.y)], angle=self.angle_diff)
                self.tr_x, self.tr_y = rot_vive[0] - pos_odom.x, rot_vive[1] - pos_odom.y
                self.sync_finished = True
                print('pub', self.tr_x, self.tr_y, self.angle_diff)

            if self.sync_finished:
                self.make_transforms((-self.tr_x, -self.tr_y, 0, 0, 0, self.angle_diff))

                rot_vive = rotate([(pos_vive.x, pos_vive.y)], angle=self.angle_diff)
                rot_vive = [rot_vive[0] - self.tr_x, rot_vive[1] - self.tr_y]
                # print(tf_vive.rotation)
                self.publish_odometry(rot_vive[0], rot_vive[1], 0.0, 0.0, 0.0)


    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = VIVE_WORLD

        t.transform.translation.x = float(transformation[0])
        t.transform.translation.y = float(transformation[1])
        t.transform.translation.z = float(transformation[2])
        quat = quaternion_from_euler(
            float(transformation[3]), float(transformation[4]), float(transformation[5]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)

    def publish_odometry(self, x, y, theta, speed, ang_speed):
        """ Publish odometry message. """
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_VIVE'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = math.cos(theta / 2)
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.twist.twist.linear.x = speed
        msg.twist.twist.angular.z = ang_speed
        self.publisher_odom.publish(msg)

def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
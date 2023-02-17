import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# from tf_transformations import quaternion_from_euler

VIVE_WORLD = 'libsurvive_world'
MAP_FRAME = 'map'
TRACKER = 'LHR-28AFC235'
CALIB_DIST = 0.5    # move at least 1m to align frames

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

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
        super().__init__('static_turtle_tf2_broadcaster')

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

    def on_timer(self):
        from_frame_rel = VIVE_WORLD
        to_frame_odom = 'base_link'
        to_frame_vive = self.tracker

        if self.sync_finished:
            return

        try:
            pos_odom = self.tf_buffer.lookup_transform(
                'map',
                to_frame_odom,
                rclpy.time.Time()).transform.translation
            pos_vive = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_vive,
                rclpy.time.Time()).transform.translation
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
            print('=>', d, angle_odom, '|', angle_vive)
            if d > CALIB_DIST and not self.sync_finished:
                angle_diff = angle_odom - angle_vive
                print('angle_diff', angle_diff)
                rot_odom = rotate([(pos_odom.x, pos_odom.y)], angle=angle_diff)
                tr_x, tr_y = rot_odom[0] - pos_vive.x, rot_odom[1] - pos_vive.y
                print(tr_x, tr_y)
                self.make_transforms((-tr_x, -tr_y, 0, 0, 0, angle_diff))
                self.sync_finished = True

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = VIVE_WORLD
        t.child_frame_id = MAP_FRAME

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
import rclpy
from rclpy.node import Node
import json
import math
import time

from angle_headings import Angle
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from simple_pid import PID
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import euler_from_quaternion

TIMER_FREQ = 20
dt = 1 / TIMER_FREQ

MAX_ACCEL = 0.1


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def sign(a):
    return 1 if a > 0 else -1

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.mag_subscription = self.create_subscription(
        #     MagneticField,
        #     '/bno055/mag',
        #     self.mag_callback,
        #     10)

        self.target_angle = Angle(0, 'deg')     # 30s
        self.ang_vel = 0
        self.pid_angle = PID(1.5, 0.0, 0.0, setpoint=self.target_angle.convert('rad'))
        self.pid_line = PID(1.5, 0.0, 0.0, setpoint=0)

        def pi_clip(angle):
            if angle > 0:
                if angle > math.pi:
                    return angle - 2*math.pi
            else:
                if angle < -math.pi:
                    return angle + 2*math.pi
            return angle
        self.pid_angle.error_map = pi_clip

        self.heading = None
        # self.imu_subscription = self.create_subscription(
        #     Imu,
        #     '/bno055/imu',
        #     self.imu_callback,
        #     10)
        self.compass_subscription = self.create_subscription(
            Float32,
            '/compass',
            self.compass_callback,
            10)

        # self.calib = None
        # self.calib_sub = self.create_subscription(
        #     String,
        #     '/bno055/calib_status',
        #     self.calib_callback,
        #     10)

        self.odom_data = None
        self.odom_heading = None
        self.odom_count = 0
        self.diff_compass_odom = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        self.turning = False
        self.start_time = None
        self.start_position = None
        self.timer = self.create_timer(0.05, self.timer_callback_rotate)
        # self.timer = self.create_timer(0.05, self.timer_callback_square)
        # self.timer = self.create_timer(0.05, self.timer_callback_line)
        self.timer = self.create_timer(0.2, self.log_callback)

    # def mag_callback(self, msg):
    #     if not self.calib:
    #         return
    #     # if not (self.calib['sys'] == 3 and self.calib['mag'] == 3):
    #     #     print('\t'*6, self.calib)

    #     self.mag_field = mag_field = msg.magnetic_field
    #     mag = [axis*1e6 for axis in (mag_field.x, mag_field.y, mag_field.z)]
    #     mag_strength = math.dist([0]*3, mag)
    #     # print(f'{self.heading:.2f} \t{mag_strength:.2f} \t', mag)
    #     # print(f'{mag[0]:02f}\t{mag[1]:02f}\t{mag[2]:02f}\t{mag_strength}')

    # def imu_callback(self, msg):
    #     q = msg.orientation
    #     ex, ey, ez = euler_from_quaternion([q.x, q.y, q.z, q.w])
    #     self.heading = Angle(ez)
    #     # print("H", self.heading)

    def compass_callback(self, msg):
        self.heading = Angle(Angle(-msg.data, 'deg').convert('rad'))
        self.compass = Angle(msg.data, 'deg')
        # print(f"H: {float(self.heading):.2f}")

    # def calib_callback(self, msg):
    #     self.calib = json.loads(msg.data)
    #     if not (self.calib['sys'] == 3 and self.calib['mag'] == 3):
    #         # print('\t'*6, self.calib)
    #         pass

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = q = msg.pose.pose.orientation
        _, _, ez = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_heading = Angle(ez)  # TODO: negative?????
        self.odom_count += 1
        # print("OH", self.odom_heading.convert('deg'))


    def timer_callback_rotate(self):
        if self.odom_heading is None:
            print('no ODOM heading...')
            return

        if self.heading is None:
            print('no compass heading...')
            return

        if (self.diff_compass_odom is None and self.heading and self.odom_heading):
            self.diff_compass_odom = self.heading - self.odom_heading
            print(f"SAVE RAD {self.diff_compass_odom.measure:.2f} = {self.heading.measure:.2f} - {self.odom_heading.measure:.2f}")
            print(f"SAVE DEG {self.diff_compass_odom.convert('deg'):.2f} = {self.heading.convert('deg'):.2f} - {self.odom_heading.convert('deg'):.2f}")

        if self.diff_compass_odom is not None:
            odom_heading_abs = self.odom_heading + self.diff_compass_odom
            drift = self.heading - odom_heading_abs

            drift = drift.convert('deg')
            if drift > 180:
                drift = drift - 360
            # print('computed', odom_heading_abs)
        else:
            return  # do not continue until we have valid heading

        # print(f'{self.heading:.2f} \t{self.odom_heading:.2f} \t{odom_heading_abs:.2f} \t', self.odom_count * 10, 'Hz')
        # self.last_odom_count = self.odom_count

        # turn_speed = -1.0
        # if not self.turning:
        #     if time.monotonic() > self.finish_time + 3:
        #         print("turning...")
        #         self.start_angle = odom_heading_abs
        #         # self.start_time = time.monotonic()
        #         self.vel_msg.angular.z = turn_speed
        #         self.vel_pub.publish(self.vel_msg)
        #         self.turning = True

        # if self.turning:
        #     progress_angle = odom_heading_abs - self.start_angle

######### CIRCLE

        if self.start_time is None:
            self.start_time = time.monotonic()
        else:
            if time.monotonic() - self.start_time > 8:
                self.start_time = time.monotonic()
                self.target_angle += -90
                self.pid_angle.setpoint = self.target_angle.convert('rad')
                self.vel_msg.linear.x = 0.0

        heading = odom_heading_abs
        # heading = self.heading
        target_ang_vel = constrain(self.pid_angle(heading.convert('rad')), -0.5, 0.5)
        if abs(target_ang_vel) > abs(self.ang_vel):
            self.ang_vel += sign(target_ang_vel) * min(MAX_ACCEL*dt, abs(target_ang_vel-self.ang_vel))
        else:
            self.ang_vel = target_ang_vel

        print(f"{self.heading.convert('deg')} - {odom_heading_abs.convert('deg')} = {drift:.2f} " 
              f"| {float(heading.convert('deg')):.2f} ({heading.mod}) -> \t{float(self.target_angle):.2f}")
        # self.vel_msg.angular.z = constrain(kP * float(self.target_angle - heading), -0.3, 0.3)
        self.vel_msg.angular.z = self.ang_vel
        self.vel_pub.publish(self.vel_msg)



    def timer_callback_square(self):
        if self.odom_heading is None:
            print('no ODOM heading...')
            return

        if self.heading is None:
            print('no compass heading...')
            return

        if (self.diff_compass_odom is None and self.heading and self.odom_heading):
            self.diff_compass_odom = self.heading - self.odom_heading
            print(f"SAVE RAD {self.diff_compass_odom.measure:.2f} = {self.heading.measure:.2f} - {self.odom_heading.measure:.2f}")
            print(f"SAVE DEG {self.diff_compass_odom.convert('deg'):.2f} = {self.heading.convert('deg'):.2f} - {self.odom_heading.convert('deg'):.2f}")

        if self.diff_compass_odom is not None:
            odom_heading_abs = self.odom_heading + self.diff_compass_odom
            drift = self.heading - odom_heading_abs
            # print('computed', odom_heading_abs)
        else:
            return  # do not continue until we have valid heading

######### SQUARE

        if self.start_time is None:
            self.start_time = time.monotonic()
        else:
            if time.monotonic() - self.start_time > 13:
                self.start_time = time.monotonic()
                self.target_angle += -90
                self.pid_angle.setpoint = self.target_angle.convert('rad')
            elif time.monotonic() - self.start_time > 12:
                self.vel_msg.linear.x = constrain(self.vel_msg.linear.x - 0.02, 0.0, 0.3)
                print('--', self.vel_msg.linear.x)
            elif time.monotonic() - self.start_time > 7:
                self.vel_msg.linear.x = constrain(self.vel_msg.linear.x + 0.02, 0.0, 0.3)
                print('++', self.vel_msg.linear.x)
                pass

        heading = odom_heading_abs
        # heading = self.heading
        target_ang_vel = constrain(self.pid_angle(heading.convert('rad')), -0.5, 0.5)
        if abs(target_ang_vel) > abs(self.ang_vel):
            self.ang_vel += sign(target_ang_vel) * min(MAX_ACCEL*dt, abs(target_ang_vel-self.ang_vel))
        else:
            self.ang_vel = target_ang_vel

        # print(f"{self.heading.convert('deg')} - {odom_heading_abs.convert('deg')} = {drift.convert('deg'):.2f} " 
        #       f"| {float(heading.convert('deg')):.2f} ({heading.mod}) -> \t{float(self.target_angle):.2f}")
        # self.vel_msg.angular.z = constrain(kP * float(self.target_angle - heading), -0.3, 0.3)
        self.vel_msg.angular.z = self.ang_vel
        self.vel_pub.publish(self.vel_msg)

##########  LINE, drive straigth

    def timer_callback_line(self):
        if self.start_position is None:
            self.start_time = time.monotonic()
            self.start_position = self.position
            self.vel_msg.linear.x = 0.1
            self.vel_pub.publish(self.vel_msg)
        else:
            if time.monotonic() - self.start_time > 10:
                # self.pid_angle.setpoint = self.target_angle.convert('rad')
                self.vel_msg.linear.x = 0.0
                self.vel_pub.publish(self.vel_msg)

########### logging

    def log_callback(self):
        if not hasattr(self, 'mag_field'):
            return
        if not hasattr(self, 'print_header'):
            print('x,y,z,ow,ox,oy,oz,mx,my,mz')
            self.print_header = True
        p, o, m = self.position, self.orientation, self.mag_field
        print(','.join(str(e) for e in [p.x, p.y, p.z, o.w, o.x, o.y, o.z, m.x, m.y, m.z]))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

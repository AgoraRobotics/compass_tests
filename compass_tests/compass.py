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

MAX_ACCEL = 0.05
MAX_ANG_VEL = 0.3


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def sign(a):
    return 1 if a > 0 else -1

class MinimalSubscriber(Node):

    def __init__(self, timer_callback_name):
        super().__init__('minimal_subscriber')
        # self.mag_subscription = self.create_subscription(
        #     MagneticField,
        #     '/bno055/mag',
        #     self.mag_callback,
        #     10)

        self.target_angle = Angle(180, 'deg')     # 30s
        self.ang_vel = 0

        self.pid_angle = PID(2.5, 0.0, 0.25, setpoint=self.target_angle.convert('rad'))
        self.pid_angle.sample_time = dt
        # self.pid_angle.differential_on_measurement = False
        self.pid_angle.output_limits = (-MAX_ANG_VEL, MAX_ANG_VEL)

        self.pid_line = PID(1.5, 0.0, 0.0, setpoint=0)
        self.pid_side = PID(1.5, 0.0, 0.0, setpoint=0)

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
        timer_callback = getattr(self, timer_callback_name)
        self.timer = self.create_timer(0.05, timer_callback)
        # self.timer = self.create_timer(0.05, self.timer_callback_rotate_360)
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

### ROTATE 360 ###

    def timer_callback_rotate_360(self):
        if self.odom_heading is None:
            print('no ODOM heading...')
            return

        if self.heading is None:
            print('no compass heading...')
            return

        IDLE, STARTED, HALF, DONE = range(4)
        if not hasattr(self, 'state'):
            # save odom
            self.state = IDLE
        print('STATE', self.state)

        if self.state is IDLE:
            self.state = STARTED
            self.vel_msg.linear.x = 0.0 # 0.06 for turn on one wheel
            self.ang_vel = 0.0
            self.start_heading = self.heading
            self.start_odom = self.odom_heading
            self.last_progress = 0
            # TODO: clear wheel_distance, compass calib

        elif self.state is STARTED:
            self.ang_vel = constrain(self.ang_vel - MAX_ACCEL, -MAX_ANG_VEL, MAX_ANG_VEL)

            progress = self.heading - self.start_heading
            print('progress', progress)
            if abs(progress) > abs(self.last_progress):
                self.last_progress = progress
            elif abs(progress) < abs(self.last_progress) - math.pi/6:    # 30 deg past half
                self.state = HALF
                self.pid_angle.setpoint = self.start_heading.convert('rad')

        elif self.state is HALF:
            target_ang_vel = self.pid_angle(self.heading.convert('rad'))
            if abs(target_ang_vel) > abs(self.ang_vel):
                self.ang_vel += sign(target_ang_vel) * min(MAX_ACCEL*dt, abs(target_ang_vel-self.ang_vel))
            else:
                self.ang_vel = target_ang_vel
            print('heading', self.heading.convert('deg'), '->', self.start_heading.convert('deg'), target_ang_vel, self.pid_angle.components)

            if abs(self.heading - self.start_heading) < Angle('0.5', 'deg').convert('rad'):
                self.ang_vel = 0.0
                self.vel_msg.linear.x = 0.0
                self.state = DONE

        elif self.state is DONE:
            self.ang_vel = 0.0
            print('odom', self.start_odom.convert('deg'), '->', self.odom_heading.convert('deg'))
            print('odom correction', (2*math.pi - float(self.odom_heading-self.start_odom))/(math.pi*2))
            exit()

            self.state = DONE + 1
                
        # heading = odom_heading_abs

        # print(f"{self.heading.convert('deg')} - {odom_heading_abs.convert('deg')} = {drift:.2f} " 
        #       f"| {float(heading.convert('deg')):.2f} ({heading.mod}) -> \t{float(self.target_angle):.2f}")
        # self.vel_msg.angular.z = constrain(kP * float(self.target_angle - heading), -0.3, 0.3)

        self.vel_msg.angular.z = self.ang_vel
        self.vel_pub.publish(self.vel_msg)
        
        
######### CIRCLE

    def timer_callback_rotate_in_steps(self):
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
        target_ang_vel = self.pid_angle(heading.convert('rad'))
        if abs(target_ang_vel) > abs(self.ang_vel):
            self.ang_vel += sign(target_ang_vel) * min(MAX_ACCEL*dt, abs(target_ang_vel-self.ang_vel))
        else:
            self.ang_vel = target_ang_vel

        print(f"{self.heading.convert('deg')} - {odom_heading_abs.convert('deg')} = {drift:.2f} " 
              f"| {float(heading.convert('deg')):.2f} ({heading.mod}) -> \t{float(self.target_angle):.2f}")
        # self.vel_msg.angular.z = constrain(kP * float(self.target_angle - heading), -0.3, 0.3)
        self.vel_msg.angular.z = self.ang_vel
        self.vel_pub.publish(self.vel_msg)

######### SQUARE

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

        if self.start_time is None:
            self.start_time = time.monotonic()
        else:
            if time.monotonic() - self.start_time > 13:
                self.start_time = time.monotonic()
                self.target_angle += 90    # positive is CCW (trigonometric)
                self.pid_angle.setpoint = self.target_angle.convert('rad')
            elif time.monotonic() - self.start_time > 12:
                self.vel_msg.linear.x = constrain(self.vel_msg.linear.x - 0.02, 0.0, 0.3)
                # print('--', self.vel_msg.linear.x)
            elif time.monotonic() - self.start_time > 6:
                self.vel_msg.linear.x = constrain(self.vel_msg.linear.x + 0.02, 0.0, 0.3)
                # print('++', self.vel_msg.linear.x)
                pass
            else:   # 0-7 seconds
                pass    # only the rest of code below is executed

        heading = odom_heading_abs
        # heading = self.heading
        target_ang_vel = self.pid_angle(heading.convert('rad'))
        # smooth acceleration
        # if abs(target_ang_vel) > abs(self.ang_vel):
        #     self.ang_vel += sign(target_ang_vel) * min(MAX_ACCEL*dt, abs(target_ang_vel-self.ang_vel))
        # else:
        self.ang_vel = target_ang_vel

        # print(f"{self.heading.convert('deg')} - {odom_heading_abs.convert('deg')} = {drift.convert('deg'):.2f} " 
        print(f"| {float(heading.convert('deg')):.2f} ({heading.mod}) -> \t{float(self.target_angle):.2f} | {target_ang_vel:.2f}")
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
    callback = args.pop('callback', 'timer_callback_rotate_360')
    rclpy.init(args=None)

    compass_test = MinimalSubscriber(callback)

    rclpy.spin(compass_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    compass_test.destroy_node()
    rclpy.shutdown()

def main_rotate():
    main({'callback': 'timer_callback_rotate_360'})

def main_square():
    main({'callback': 'timer_callback_square'})

if __name__ == '__main__':
    main()

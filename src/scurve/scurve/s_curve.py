import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import sin, pi


class SCurve(Node):

    def __init__(self):
        super().__init__('smooth_vel_node')

        # get ros params
        self.get_ros_params()

        # init variable
        self.isLock = True
        self.cmd_speed = [0.0, 0.0]
        self.last_cmd_speed = [0.0, 0.0]
        self.speed_sp = [0.0, 0.0]
        self.speed_pv = [0.0, 0.0]
        self.speed_last_sp = [0.0, 0.0]
        self.speed_error = [0.0, 0.0]
        self.speed_error_max = 0.0

        self.sprofile_T = self.sprofile_t = self.sprofile
        
        # init
        self.current_time = self.get_clock().now().to_msg()
        self.last_time = self.get_clock().now().to_msg()
        self.time_last_cmd = self.get_clock().now().to_msg()

        # Create topics
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel_smooth', 1)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, 1)

        self.timer = self.create_timer(self.time_out, self.timeout_callback)

    def update(self):
        scurve_vel = Twist()
        self.current_time = self.get_clock().now().to_msg()
        if self.sprofile_t <= self.sprofile_T:
            for i in range(2):
                if self.speed_sp[i] > self.speed_pv[i]:
                    self.speed_pv[i] = self.speed_last_sp[i] + self.sCurves_accel_decel(self.speed_error[i], self.sprofile_t)
                elif self.speed_sp[i] < self.speed_pv[i]:
                    self.speed_pv[i] = self.speed_last_sp[i] - self.sCurves_accel_decel(self.speed_error[i], self.sprofile_t)
            if self.sprofile_t >= self.sprofile_T:
                for i in range(2):
                    self.speed_pv[i] = self.speed_sp[i]
        scurve_vel.linear.x = self.speed_pv[0]
        scurve_vel.angular.z = self.speed_pv[1]
        self.sprofile_t += 2.5 
        self.pub_cmd_vel.publish(scurve_vel)

    def sCurves_accel_decel(self, V, t):
        res = V * (2 * pi * t / self.sprofile_T - sin(2 * pi * t / self.sprofile_T)) / 2 / pi
        return res

    def even_cmd_vel_set(self):
        for i in range(2):
            self.speed_last_sp[i] = self.speed_pv[i]
            self.speed_sp[i] = self.cmd_speed[i]
            self.speed_error[i] = abs(self.speed_sp[i] - self.speed_pv[i]) 
            if self.speed_error[i] > self.speed_error_max:
                self.speed_error_max = self.speed_error[i]
        self.sprofile_T = (self.speed_error_max/self.sprofile) * 100.0
        if self.sprofile_T < 100.0:
            self.sprofile_T = 100.0
        if self.sprofile_t >= self.sprofile_T:
            self.sprofile_t = 0.0

    def timeout_callback(self, time):
        if self.current_time - self.last_time >= rclpy.Duration.from_sec(self.time_out):
            set_vel_timeout = Twist()
            set_vel_timeout.linear.x = 0.0
            set_vel_timeout.angular.z = 0.0
            self.callback_cmd_vel(set_vel_timeout)

    def callback_cmd_vel(self, twist):
        self.current_time = self.last_time = rclpy.Time.now()
        if self.last_cmd_speed[0] != twist.linear.x or self.last_cmd_speed[1] != twist.angular.z:
            self.sprofile_t = self.sprofile_T
            self.last_cmd_speed[0] = self.cmd_speed[0] = twist.linear.x
            self.last_cmd_speed[1] = self.cmd_speed[1] = twist.angular.z
            self.even_cmd_vel_set()
        

    def get_ros_params(self):
        self.frequency = rclpy.get_parameter(self.node_name + '/frequency', 30)
        self.time_out = rclpy.get_parameter(self.node_name + '/time_out', 1.2)
        self.sprofile = rclpy.get_parameter(self.node_name + '/sprofile', 1000)

    def run(self):
        rate = rclpy.Rate(self.frequency)
        while not self.context.is_shutdown():
            self.update()
            rate.sleep()

def main():
    rclpy.init()
    node = SCurve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
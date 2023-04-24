#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moteus_msgs.msg import MoteusCommand, MoteusCommandStamped, MoteusStateStamped, MoteusState
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped

class MoteusControlNode(Node):
    def __init__(self):
        super().__init__('MOTEUS_VEL_NODE')
        self.nodename = "MOTEUS_VEL_NODE"
        self.get_logger().info(f"{self.nodename} STARTED")

        # Variable
        self.wheel_left = 0.0
        self.wheel_right = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 1.0
        self.dx = 0.0 # speeds in x/rotation
        self.dr = 0.0        

        self.base_width = float(self.declare_parameter('base_width', 0.65).value)  
        self.radius_of_wheels = float(self.declare_parameter('radius_of_wheels', 0.254/2).value)  
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value        

        #publish moteus_command
        self.pub = self.create_publisher(MoteusCommandStamped, 'moteus_command', 1)

        self.pub_odom_wheel = self.create_publisher(Odometry, 'odometry', 10)

        # RATE TIMER HZ
        self.rate = self.declare_parameter("rate", 100.0).value
        self.create_timer(1.0/self.rate, self.update)      
        
        # subscribe to cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)  #scurve_cmd_vel #cmd_vel_accel_desel  



    def update(self):

        now = self.get_clock().now()

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame
        transform_stamped_msg.child_frame_id = self.base_frame
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.w = quaternion.w
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z


        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.pub_odom_wheel.publish(odom)

    def cmd_vel_callback(self, twist):
      
        V = twist.linear.x
        W = twist.angular.z

        self.wheel_left = (V + (W * self.base_width / 2)) / self.radius_of_wheels
        self.wheel_right = (V - (W * self.base_width / 2)) / self.radius_of_wheels

        moteusCommandStamped = MoteusCommandStamped()
        moteusCommandStamped.header.stamp = self.get_clock().now().to_msg()
        moteusCommandStamped.header.frame_id = "moteus_command"

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 1
        moteusCommand.velocity = self.wheel_left * -4.5
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 2
        moteusCommand.velocity = self.wheel_right * -4.5
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        self.pub.publish(moteusCommandStamped) 
     
    def callback_update(self):
        pass

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
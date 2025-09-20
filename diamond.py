import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        if self.time < 2:  # rotate ~45 deg at start
            msg = self.create_twist(0.0, 0.8)
        elif self.time < 7:
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 9:
            msg = self.create_twist(0.0, 1.6)  # 90 deg
        elif self.time < 14:
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 16:
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 21:
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 23:
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 28:
            msg = self.create_twist(1.0, 0.0)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg
    
    def timer_callback(self):
        msg = self.get_twist_msg()
        self.publisher.publish(msg)
        self.time += 1
        print("time:", self.time)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

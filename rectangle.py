import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_rectangle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def get_twist_msg(self):
        if self.time < 8:
            return self.create_twist(1.0, 0.0)  # long side
        elif self.time < 10:
            return self.create_twist(0.0, 1.6)  # turn
        elif self.time < 14:
            return self.create_twist(1.0, 0.0)  # short side
        elif self.time < 16:
            return self.create_twist(0.0, 1.6)  # turn
        elif self.time < 24:
            return self.create_twist(1.0, 0.0)  # long side
        elif self.time < 26:
            return self.create_twist(0.0, 1.6)  # turn
        elif self.time < 30:
            return self.create_twist(1.0, 0.0)  # short side
        elif self.time < 32:
            return self.create_twist(0.0, 1.6)  # final turn
        else:
            return self.create_twist(0.0, 0.0)  # stop

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def timer_callback(self):
        msg = self.get_twist_msg()
        self.publisher.publish(msg)
        self.time += 1
        print(f"time: {self.time}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

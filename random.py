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
        if self.time < 5:
            msg = self.create_twist(1.0, 0.0)  # side 1
        elif self.time < 8:
            msg = self.create_twist(0.0, 1.05)  # ~120 deg turn
        elif self.time < 13:
            msg = self.create_twist(1.0, 0.0)  # side 2
        elif self.time < 16:
            msg = self.create_twist(0.0, 1.05)  # turn again
        elif self.time < 21:
            msg = self.create_twist(1.0, 0.0)  # side 3
        else:
            msg = self.create_twist(0.0, 0.0)  # stop
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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_circle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def get_twist_msg(self):
        if self.time < 40:  # ~20 seconds drawing a circle
            msg = Twist()
            msg.linear.x = 1.0     # forward speed
            msg.angular.z = 0.5    # constant turn
        else:
            msg = Twist()  # stop
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

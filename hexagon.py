import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_hexagon')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # publish every 0.5s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def get_twist_msg(self):
        # Each side takes ~4 seconds (8 ticks of 0.5s), each turn ~2 seconds
        if self.time < 8:
            return self.create_twist(1.0, 0.0)   # side 1
        elif self.time < 12:
            return self.create_twist(0.0, 1.0)   # turn
        elif self.time < 20:
            return self.create_twist(1.0, 0.0)   # side 2
        elif self.time < 24:
            return self.create_twist(0.0, 1.0)   # turn
        elif self.time < 32:
            return self.create_twist(1.0, 0.0)   # side 3
        elif self.time < 36:
            return self.create_twist(0.0, 1.0)   # turn
        elif self.time < 44:
            return self.create_twist(1.0, 0.0)   # side 4
        elif self.time < 48:
            return self.create_twist(0.0, 1.0)   # turn
        elif self.time < 56:
            return self.create_twist(1.0, 0.0)   # side 5
        elif self.time < 60:
            return self.create_twist(0.0, 1.0)   # turn
        elif self.time < 68:
            return self.create_twist(1.0, 0.0)   # side 6
        elif self.time < 72:
            return self.create_twist(0.0, 1.0)   # final turn back to start
        else:
            return self.create_twist(0.0, 0.0)   # stop

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

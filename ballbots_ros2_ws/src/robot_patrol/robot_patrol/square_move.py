import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Square Mover Node Started! Get ready...')

    def move_robot(self):
        # Define our moves: (x_speed, y_speed, duration)
        # x = Forward/Back, y = Left/Right (Strafing)
        moves = [
            (0.2,  0.0, 5.0),  # Forward
            (0.0,  0.2, 5.0),  # Slide Left
            (-0.2, 0.0, 5.0),  # Backward
            (0.0, -0.2, 5.0)   # Slide Right
        ]

        for x, y, duration in moves:
            self.get_logger().info(f"Executing move: X={x}, Y={y} for {duration}s")
            
            # We must publish continuously or the robot safety watchdog will stop it
            end_time = time.time() + duration
            while time.time() < end_time:
                msg = Twist()
                msg.linear.x = float(x)
                msg.linear.y = float(y)
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                time.sleep(0.1) # Publish at 10Hz

        # Stop at the end
        self.get_logger().info("Sequence Complete. Stopping.")
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    
    # We run the movement function once then exit
    try:
        node.move_robot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

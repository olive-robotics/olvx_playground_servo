import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SinusoidalPublisher(Node):
    def __init__(self, magnitude=2.0, period=1.0):
        super().__init__('sinusoidal_publisher')
        self.publisher_ = self.create_publisher(Float32, 'olive/servo/x1685117345645/goal/position', 1)
        self.magnitude = magnitude
        self.period = period
        self.time = 0.0

        # Ensure timer frequency accounts for the desired sinusoidal period
        timer_frequency = 0.01  # Frequency at which timer callback is triggered, in seconds
        self.timer = self.create_timer(timer_frequency, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # Generate sinusoidal value
        msg.data = self.magnitude * math.sin((2 * math.pi * self.time) / self.period)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.time += 0.01  # Increment time

def main(args=None):
    rclpy.init(args=args)
    
    # Customize magnitude and period as needed
    magnitude = 1.2  # Amplitude of the sinusoid
    period = 6  # Period of the sinusoid in seconds
    sinusoidal_publisher = SinusoidalPublisher(magnitude, period)

    try:
        rclpy.spin(sinusoidal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sinusoidal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


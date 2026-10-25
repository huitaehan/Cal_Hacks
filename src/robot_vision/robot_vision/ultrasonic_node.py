import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DistanceSensor

class MultiUltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('multi_ultrasonic_publisher')
        
        # Create three separate publishers
        self.pub_center = self.create_publisher(Float32, 'ultrasonic_center', 10)
        self.pub_left = self.create_publisher(Float32, 'ultrasonic_left', 10)
        self.pub_right = self.create_publisher(Float32, 'ultrasonic_right', 10)
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # Initialize the three sensors with their respective BCM pins
        self.sensor_center = DistanceSensor(echo=24, trigger=23)
        self.sensor_left = DistanceSensor(echo=27, trigger=17)
        self.sensor_right = DistanceSensor(echo=6, trigger=5)
        
        self.get_logger().info('Triple Ultrasonic Node has started.')

    def timer_callback(self):
        # Read and publish Center
        msg_c = Float32()
        msg_c.data = float(self.sensor_center.distance * 100.0)
        self.pub_center.publish(msg_c)

        # Read and publish Left
        msg_l = Float32()
        msg_l.data = float(self.sensor_left.distance * 100.0)
        self.pub_left.publish(msg_l)

        # Read and publish Right
        msg_r = Float32()
        msg_r.data = float(self.sensor_right.distance * 100.0)
        self.pub_right.publish(msg_r)

def main(args=None):
    rclpy.init(args=args)
    node = MultiUltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
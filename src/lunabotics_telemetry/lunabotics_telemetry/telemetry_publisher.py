import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')
        self.pub = self.create_publisher(String, '/telemetry', 10)
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f"telemetry tick {self.count}"
        self.pub.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = TelemetryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


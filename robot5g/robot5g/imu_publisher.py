import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Imu

from rclpy.time import Time

class ImuPublisher(Node):
	def __init__(self):
		super().__init__('imu_data_publisher')
		self.new_imu_pub = self.create_publisher(Imu, 'imu/data', 10)
		self.raw_imu_sub = self.create_subscription(Imu, 'imu/raw', self.raw_imu_callback, 10)

		period = 1/40.0
		self.timer = self.create_timer(period, self.timer_callback)

		self.imu_data = Imu()
		self.get_logger().info('IMU publisher node is activated')

	def timer_callback(self):
		self.new_imu_pub.publish(self.imu_data)

	def raw_imu_callback(self, msg_in):
		self.imu_data = msg_in
		self.imu_data.header.frame_id = "imu_link"
		self.imu_data.header.stamp = self.get_clock().now().to_msg()

def main():
	rclpy.init()
	ip = ImuPublisher()

	rclpy.spin(ip)

	ip.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
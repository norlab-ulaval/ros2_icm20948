import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import geometry_msgs.msg
import qwiic_icm20948
import math

from time import sleep

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.imu_publisher = self.create_publisher(sensor_msgs.msg.Imu, '/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(sensor_msgs.msg.MagneticField, '/imu/mag_raw', 10)

    def log_info(self, message):
        self.get_logger().info(message)

    def publish_imu(self, msg):
        self.imu_publisher.publish(msg)

    def publish_mag(self, msg):
        self.mag_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()

    imu = qwiic_icm20948.QwiicIcm20948()
    while not imu.connected and rclpy.ok():
        node.log_info("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
    
    imu.begin()

    imu.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
    imu.setFullScaleRangeAccel(qwiic_icm20948.gpm16)
    
    imu_msg = sensor_msgs.msg.Imu()
    mag_msg = sensor_msgs.msg.MagneticField()
    rate = node.create_rate(100)
    while rclpy.ok():
        if imu.dataReady():
            imu.getAgmt()
            imu_msg.header.stamp = node.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu"
            imu_msg.linear_acceleration.x = imu.axRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.y = imu.ayRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.z = imu.azRaw * 9.81 / 2048.0
            imu_msg.angular_velocity.x = imu.gxRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.y = imu.gyRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.z = imu.gzRaw * math.pi / (16.4 * 180)
            imu_msg.orientation_covariance[0] = -1
            node.publish_imu(imu_msg)

            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = "imu"
            mag_msg.magnetic_field.x = imu.mxRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.y = imu.myRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.z = imu.mzRaw * 1e-6 / 0.15
            node.publish_mag(mag_msg)

        rclpy.spin_once(node)
        rate.sleep()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import math

import qwiic_icm20948
import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class ICM20948Node(Node):
    def __init__(self):
        super().__init__("icm20948_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_address", 0x69)
        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_addr = i2c_addr

        self.declare_parameter("frame_id", "imu_icm20948")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("pub_rate", 50)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        # IMU instance
        self.imu = qwiic_icm20948.QwiicIcm20948(address=self.i2c_addr)
        if not self.imu.connected:
            self.logger.info(
                "The Qwiic ICM20948 device isn't connected to the system. Please check your connection."
            )
        self.imu.begin()
        self.imu.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
        self.imu.setFullScaleRangeAccel(qwiic_icm20948.gpm16)

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(
            sensor_msgs.msg.MagneticField, "/imu/mag_raw", 10
        )
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def publish_cback(self):
        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()
        if self.imu.dataReady():
            try:
                self.imu.getAgmt()
            except Exception as e:
                self.logger.info(str(e))

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.linear_acceleration.x = self.imu.axRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.y = self.imu.ayRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.z = self.imu.azRaw * 9.81 / 2048.0
            imu_msg.angular_velocity.x = self.imu.gxRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.y = self.imu.gyRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.z = self.imu.gzRaw * math.pi / (16.4 * 180)
            imu_msg.orientation_covariance[0] = -1

            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = self.imu.mxRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.y = self.imu.myRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.z = self.imu.mzRaw * 1e-6 / 0.15

        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    icm20948_node = ICM20948Node()
    rclpy.spin(icm20948_node)

    icm20948_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

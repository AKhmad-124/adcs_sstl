import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu  # IMU message type
import sys 

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # ROS 2 Publisher for reaction wheel velocity
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rec_wheel_vel', 10)

        # ROS 2 Subscriber for IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )

        # Serial Port (Update this with the correct port)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Allow time for connection

        # Store IMU data
        self.imu_data = None

        # Start Timer to read serial
        self.create_timer(0.1, self.read_serial)

    def imu_callback(self, msg):
        """Callback to process IMU data and send to Arduino."""
        angular_x = round(msg.angular_velocity.x, 4)
        angular_y = round(msg.angular_velocity.y, 4)
        angular_z = round(msg.angular_velocity.z, 4)

        orientation_x = round(msg.orientation.x, 4)
        orientation_y = round(msg.orientation.y, 4)
        orientation_z = round(msg.orientation.z, 4)

        # Convert to CSV format for easy parsing on Arduino
        self.imu_string = f"555,{angular_x},{angular_y},{angular_z},{orientation_x},{orientation_y},{orientation_z}#"#*sacruficial "555,"in the begining for no transmission errors
        self.serial_port.write(self.imu_string.encode('utf-8'))
        # self.get_logger().info(f"Sent to Arduino: {self.imu_string.strip()}")

    def read_serial(self):
        """Read data from Arduino and publish to ROS topic."""
        if self.serial_port.in_waiting > 0:
            try:

                data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()


                # Ensure the message is properly delimited
                    
                list_of_data = data.split(',')


                # print(self.imu_string)
                

                # if len(list_of_data)!=6:
                    
                #     print(f"\033[31mbad data:::::::!\033[0m{data}")
                # else:
                #     print(list_of_data)
                rec_wheel_to_publish = []
                for i in range(3):
                    rec_wheel_to_publish.append(float(list_of_data[i]))  # First 3 values as floats
                if len(rec_wheel_to_publish) == 3:  # Ensure correct size
                    msg = Float32MultiArray()
                    msg.data = rec_wheel_to_publish
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {rec_wheel_to_publish}") #uncomment after solving serial

            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

def main():
    rclpy.init()
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

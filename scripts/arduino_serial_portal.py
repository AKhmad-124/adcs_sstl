import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32MultiArray

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rec_wheel_vel', 10)
        
        # Serial Port (Update this with the correct port)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Allow time for connection
        
        # Start Timer
        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                data_str = "hey there"+ "\n"
                self.serial_port.write(data_str.encode('utf-8'))
                self.get_logger().info(f"Sent: {data_str}")

                data = self.serial_port.readline().decode('utf-8').strip()

                list_of_data =  data.split(',') 
                print(list_of_data)
                rec_wheel_to_publish = []
                for i in range(3):
                    rec_wheel_to_publish.append(float(list_of_data[i]))#* gets the first three things from the lsit and converts them to flaot

                if len(rec_wheel_to_publish) == 3:  # makes sure it's the correct size
                    msg = Float32MultiArray()
                    msg.data = rec_wheel_to_publish
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {rec_wheel_to_publish}")

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

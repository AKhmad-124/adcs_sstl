import rclpy
from rclpy.node import Node
import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu  # IMU message type

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

        # Initialize storage for plotting
        self.time_data = []
        self.values = [[] for _ in range(6)]  # Six variables
        self.max_points = 1000  # Max points to display

        # Setup Matplotlib for real-time plotting
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f'Var {i+1}')[0] for i in range(6)]
        self.ax.legend()
        self.ax.set_xlabel("Time (samples)")
        self.ax.set_ylabel("Values")
        self.ax.set_title("Live Sensor Data")

        # Start Timer for reading serial data
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
        self.imu_string = f"555,{angular_x},{angular_y},{angular_z},{orientation_x},{orientation_y},{orientation_z}#"
        self.serial_port.write(self.imu_string.encode('utf-8'))

    def read_serial(self):
        """Read data from Arduino, store for plotting, and publish to ROS topic."""
        if self.serial_port.in_waiting > 0:
            try:
                data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                list_of_data = data.split(',')

                print(list_of_data)  # Debugging
                if len(list_of_data) != 6:
                    print(f"\033[31m[ERROR] Bad data received: {data}\033[0m")
                else:
                
                    # Store data for plotting
                    self.time_data.append(len(self.time_data))  # Increment time index
                    if len(self.time_data) > self.max_points:
                        self.time_data.pop(0)
                    
                    for i in range(6):
                        self.values[i].append(float(list_of_data[i]))
                        if len(self.values[i]) > self.max_points:
                            self.values[i].pop(0)

                    # Update the plot in real-time
                    self.update_plot()

                # Publish reaction wheel velocity (first 3 values)
                rec_wheel_to_publish = [float(list_of_data[i]) for i in range(3)]
                msg = Float32MultiArray()
                msg.data = rec_wheel_to_publish
                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

    def update_plot(self):
        """Updates the Matplotlib plot dynamically."""
        for i, line in enumerate(self.lines):
            line.set_data(self.time_data, self.values[i])

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = SerialReader()
    rclpy.spin(node)  # Runs ROS normally
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

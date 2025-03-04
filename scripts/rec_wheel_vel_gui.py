import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk

class WheelSpeedGUI(Node):
    def __init__(self):
        super().__init__('wheel_speed_gui')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rec_wheel_vel', 10)

        self.root = tk.Tk()
        self.root.title("Reaction Wheel Speed Control")

        # Frame to organize sliders
        self.frame = tk.Frame(self.root)
        self.frame.pack(padx=20, pady=10)

        self.sliders = {}
        self.labels = {}
        self.vel_names = ["XY", "YZ", "XZ"]

        for i, name in enumerate(self.vel_names):#sliders
            frame = tk.Frame(self.frame)
            frame.grid(row=0, column=i, padx=15)

            self.labels[name] = tk.Label(frame, text=f"{name}: 0.0", font=("Arial", 12))
            self.labels[name].pack()

            self.sliders[name] = tk.Scale(frame, from_=20, to=-20, resolution=0.1, orient=tk.VERTICAL, length=200, command=self.publish_speeds)
            self.sliders[name].pack()

        # Reset button
        self.reset_button = tk.Button(self.root, text="Reset", command=self.reset_speeds, bg="red", fg="white", font=("Arial", 12))
        self.reset_button.pack(pady=10)

    def publish_speeds(self, _=None):
        msg = Float32MultiArray()
        msg.data = [self.sliders[name].get() for name in self.vel_names]
        
        # Update labels
        for name in self.vel_names:
            self.labels[name].config(text=f"{name}: {self.sliders[name].get()}")

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Speeds: {msg.data}")

    def reset_speeds(self):
        for name in self.vel_names:
            self.sliders[name].set(0)
            self.labels[name].config(text=f"{name}: 0.0")

        self.publish_speeds()  # Send reset values

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui = WheelSpeedGUI()
    gui.run()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import RPi.GPIO as GPIO
import time
from math import pi
import rclpy
from geometry_msgs.msg import Twist

# Define the pin number
interrupt_pin = 16  # Change this to the pin you want to use
pwm_pin = 12  # PWM pin connected to motor

# Initialize counter variable
counter = 0
counter_prev = 0
timer = time.time()
angular_speed = 0
calc_delay = 0.33

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)  # disable warnings

GPIO.setup(pwm_pin, GPIO.OUT)
pi_pwm = GPIO.PWM(pwm_pin, 2000)  # create PWM instance with frequency
pi_pwm.start(0)  # start PWM of required Duty Cycle

ref_speed = 50
kp = 0.2
kd = 0.1

# ROS 2 setup
rclpy.init()
node = rclpy.create_node('motor_controller')

# ROS 2 callback function for cmd_vel topic
def cmd_vel_callback(msg):
    global ref_speed
    ref_speed = msg.linear.x  # Assuming linear velocity in x direction corresponds to speed

# Subscribe to cmd_vel topic
# Subscribe to cmd_vel topic with QoS profile
node.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)


# Function to increment the counter
def increment_counter(channel):
    global counter
    counter += 1

GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Add event detection to the interrupt pin
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=increment_counter)

duty = 0
error_prev = 0
try:
    while rclpy.ok():
        pi_pwm.ChangeDutyCycle(duty)
        time.sleep(calc_delay)
        counter_now = counter
        count_diff = counter - counter_prev
        angular_speed = count_diff * (360 / 140) / calc_delay  # deg per sec
        linear_speed = angular_speed * (pi / 180) * 4.75  # cm per sec

        counter_prev = counter
        error = ref_speed - linear_speed
        error_diff = (error - error_prev) / calc_delay
        duty = duty + kp * error + kd * error_prev
        if duty > 100:
            duty = 100
        if duty < 0:
            duty = 0
        error_prev = error

        print(linear_speed, "     ", error, "      ", duty)

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import cv2
# import tflite_runtime.interpreter as tflite
# import numpy as np

# class TwistPublisher(Node):
#     def __init__(self):
#         super().__init__('twist_publisher')
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.cap = cv2.VideoCapture(0)
#         self.interpreter = tflite.Interpreter(model_path="/home/STACKBOT/stackbot_ws/models/MobileNet_line_follower_resize224_Apr25_with_extra_layers.tflite")
#         self.interpreter.allocate_tensors()
#         self.input_details = self.interpreter.get_input_details()
#         self.output_details = self.interpreter.get_output_details()
#         print("donr init")

#     def preprocess_image(self, img, target_size=(224, 224)):
#         img = cv2.resize(img, target_size)
#         img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert image to RGB format
#         img = img.astype(np.float32) / 255.0  # Normalize pixel values to [0, 1]
#         return img


#     def predict_twist(self):
#         ret, frame = self.cap.read()
#         frame = self.preprocess_image(frame)
#         frame = np.expand_dims(frame, axis=0)
#         self.interpreter.set_tensor(self.input_details[0]['index'], frame)
#         output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
#         # cap.release()
#         return [output_data[0][0], output_data[0][1]]

#     def publish_twist(self):
#         while rclpy.ok():
#             linear_x, angular_z = self.predict_twist()
#             # print(type(linear_x), type(angular_z))
#             twist_msg = Twist()
#             twist_msg.linear.x = float(linear_x)
#             twist_msg.angular.z = float(angular_z)

#             # Publish the Twist message
#             self.publisher_.publish(twist_msg)

#             # Sleep for a short duration to control the publishing rate
#             rclpy.spin_once(self, timeout_sec=0.1)

# def main(args=None):
#     rclpy.init(args=args)

#     twist_publisher = TwistPublisher()

#     twist_publisher.publish_twist()

#     twist_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

"""import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import tflite_runtime.interpreter as tflite
import numpy as np
import signal
import sys

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened:
            print("cannot open camera")
        self.interpreter = tflite.Interpreter(model_path="/home/STACKBOT/stackbot_ws/models/MobileNet_line_follower_resize224_Apr25_with_extra_layers.tflite")
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.i = 0
        print("done init")

    def predict_twist(self):
        ret, frame = self.cap.read()
        if ret == False:
            print("cannot read frame")
        frame = cv2.resize(frame, (224,224))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = frame.astype(np.float32)/255
        frame = np.expand_dims(frame, axis=0)
        self.interpreter.set_tensor(self.input_details[0]['index'], frame)
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        print(output_data)
        return [output_data[0][0], output_data[0][1]]

    def publish_twist(self):
        while rclpy.ok():
            linear_x, angular_z = self.predict_twist()
            print(linear_x, angular_z)
            print(self.i)
            self.i+=1
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_x)
            twist_msg.angular.z = float(angular_z)
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

def signal_handler(sig, frame):
    print("Interrupted, releasing video capture...")
    twist_publisher.cap.release()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    global twist_publisher
    twist_publisher = TwistPublisher()

    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    twist_publisher.publish_twist()

    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Twist
import cv2
import csv
import RPi.GPIO as GPIO
import time
from math import pi

counter = 0

def increment_counter(self):
    global counter
    counter += 1

class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10  # QoS profile depth
        )
        global counter
        self.linear_x = 0
        self.angular_z = 0
        self.pid_delay = 0.1
        self.pwm_pin = 12				# PWM pin connected to motor
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)			#disable warnings
        GPIO.setup(self.pwm_pin,GPIO.OUT)
        self.pi_pwm = GPIO.PWM(self.pwm_pin,2000)		#create PWM instance with frequency
        self.pi_pwm.start(0)				#start PWM of required Duty Cycle 
        self.timer = self.create_timer(self.pid_delay, self.timer_callback)

        # Define the pin number
        self.interrupt_pin = 16  # Change this to the pin you want to use

        # Initialize counter variable
        self.counter = counter
        self.counter_prev = 0
        self.timer = time.time()
        self.angular_speed = 0

        self.ref_speed = 50
        self.kp = 0.2
        self.kd = 0.05

        # self.duty = (100/118) * ref_speed 
        self.duty = 0
        self.error_prev = 0
        self.error = 0

        GPIO.setup(self.interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Add event detection to the interrupt pin
        GPIO.add_event_detect(16, GPIO.RISING, callback=increment_counter)

        print("done initializing")

    # Function to increment the counter


    def twist_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        # print("running")

    def timer_callback(self):
        global counter
        self.counter = counter
        ref = self.linear_x * 90
        self.pi_pwm.ChangeDutyCycle(self.duty)
        # time.sleep(self.pid_delay)
        self.counter_now = self.counter
        self.count_diff = self.counter - self.counter_prev
        self.angular_speed = self.count_diff*(360/140)/self.pid_delay #deg per sec
        self.linear_speed = self.angular_speed * (pi/180) * 4.75 #cm per sec

        self.counter_prev = self.counter
        self.error = ref - self.linear_speed
        self.error_diff = (self.error - self.error_prev)/self.pid_delay
        self.duty = self.duty + self.kp*self.error + self.kd*self.error_prev
        if self.duty > 100:
            self.duty = 100
        if self.duty < 0:
            self.duty = 0
        self.error_prev = self.error
        if self.linear_speed != 0:
            print(self.linear_speed, "     ", self.error, "      ", self.duty)
        
                
            



def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
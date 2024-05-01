import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Twist
import cv2
import csv
import RPi.GPIO as GPIO
import time
from math import pi

"""
READ TWIST VALUES AND SEND DUTY CYCLE VALUES TO THE MOTORS
"""

K_l = 30
K_a = 10

class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10  # QoS profile depth
        )

        self.linear_x = 0
        self.angular_z = 0
        self.pid_delay = 0.1

        self.motor1_pwm_pin_forward = 13
        self.motor1_pwm_pin_backward = 11
        self.motor2_pwm_pin_forward = 15
        self.motor2_pwm_pin_backward = 16				

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)			#disable warnings

        GPIO.setup(self.motor1_pwm_pin_forward,GPIO.OUT)
        self.motor1_pi_pwm_forward = GPIO.PWM(self.motor1_pwm_pin_forward,2000)		#create PWM instance with frequency
        self.motor1_pi_pwm_forward.start(0)				#start PWM of required Duty Cycle 
        
        GPIO.setup(self.motor1_pwm_pin_backward,GPIO.OUT)
        self.motor1_pi_pwm_backward = GPIO.PWM(self.motor1_pwm_pin_backward,2000)		#create PWM instance with frequency
        self.motor1_pi_pwm_backward.start(0)				#start PWM of required Duty Cycle 
        

        GPIO.setup(self.motor2_pwm_pin_forward,GPIO.OUT)
        self.motor2_pi_pwm_forward = GPIO.PWM(self.motor2_pwm_pin_forward,2000)		#create PWM instance with frequency
        self.motor2_pi_pwm_forward.start(0)				#start PWM of required Duty Cycle 
        
        GPIO.setup(self.motor2_pwm_pin_backward,GPIO.OUT)
        self.motor2_pi_pwm_backward = GPIO.PWM(self.motor2_pwm_pin_backward,2000)		#create PWM instance with frequency
        self.motor2_pi_pwm_backward.start(0)				#start PWM of required Duty Cycle 

        self.timer = self.create_timer(self.pid_delay, self.timer_callback)

        # Define the pin number
        self.interrupt_pin = 16  # Change this to the pin you want to use

        print("done initializing speed_control_without_pid")

    # Function to increment the counter


    def twist_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        if self.linear_x < 0.1 and self.linear_x > -0.1:
            self.linear_x = 0
        if self.angular_z < 0.1 and self.angular_z > -0.1:
            self.angular_z = 0

    def timer_callback(self):
        lin_x = self.linear_x * K_l
        ang_z = self.angular_z * K_a
        duty_left = lin_x - ang_z
        duty_right = lin_x + ang_z
        
        ################################### LIMIT THE DUTY CYCLE TO [-100, 100]
        if duty_right > 100:
            duty_right = 100
        if duty_right < -100:
            duty_right = -100

        if duty_left > 100:
            duty_left = 100
        if duty_left < -100:
            duty_left = -100

        #################################### SET LOW DUTY CYCLE VALUES TO ZERO
        # if duty_left < 5 and duty_left > -5:
        #     duty_left = 0
        
        # if duty_right < 5 and duty_right > -5:
        #     duty_right = 0
        ################################### SEND THE DUTY CYCLE VALUES TO THE MOTOR ACCORDING TO SIGN +-
        if duty_left > 0:
            self.motor1_pi_pwm_backward.ChangeDutyCycle(0)
            self.motor1_pi_pwm_forward.ChangeDutyCycle(abs(duty_left))
        if duty_left <= 0:
            self.motor1_pi_pwm_forward.ChangeDutyCycle(0)
            self.motor1_pi_pwm_backward.ChangeDutyCycle(abs(duty_left))

        if duty_right > 0:
            self.motor2_pi_pwm_backward.ChangeDutyCycle(0)
            self.motor2_pi_pwm_forward.ChangeDutyCycle(abs(duty_right))
        if duty_right <= 0:
            self.motor2_pi_pwm_forward.ChangeDutyCycle(0)
            self.motor2_pi_pwm_backward.ChangeDutyCycle(abs(duty_right))

        if duty_left != 0 or duty_right != 0:
            print(duty_left, "      ", duty_right)

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
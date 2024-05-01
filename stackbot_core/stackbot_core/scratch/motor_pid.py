import RPi.GPIO as GPIO
from time import sleep

ledpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,2000)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 
while True:
    # for duty in range(0,101,1):
    #     pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
    #     sleep(0.1)
    #     print(duty)
    # sleep(0.5)
    # sleep(5)
    # for duty in range(100,-1,-1):
    #     pi_pwm.ChangeDutyCycle(duty)
    #     sleep(0.1)
    #     print(duty)
    # sleep(5)
    # pi_pwm.ChangeDutyCycle(20)
    pi_pwm.ChangeDutyCycle(1)

#################################################################################################################################

# import RPi.GPIO as GPIO

# # Set up GPIO using BCM numbering
# GPIO.setmode(GPIO.BOARD)

# # Define the GPIO pins you want to read from
# pin1 = 16
# pin2 = 22

# # Set the pins as input
# GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# # Initialize counters
# counter1 = 0
# counter2 = 0

# # Callback function to handle rising edge detection
# def rising_edge_callback(channel):
#     global counter1, counter2
#     if channel == pin1:
#         counter1 += 1
#     elif channel == pin2:
#         counter2 += 1

# # Add event detection for rising edges
# GPIO.add_event_detect(pin1, GPIO.RISING, callback=rising_edge_callback)
# GPIO.add_event_detect(pin2, GPIO.RISING, callback=rising_edge_callback)

# try:
#     while True:
#         # Print the current counts
#         print(f"Counter 1: {counter1}, Counter 2: {counter2}")

# except KeyboardInterrupt:
#     # Clean up GPIO on keyboard interrupt
#     GPIO.cleanup()



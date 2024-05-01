import RPi.GPIO as GPIO
import time
from math import pi

# Define the pin number
interrupt_pin = 16  # Change this to the pin you want to use
pwm_pin = 12				# PWM pin connected to motor

# Initialize counter variable
counter = 0
counter_prev = 0
timer = time.time()
angular_speed = 0
calc_delay = 0.33

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)			#disable warnings

GPIO.setup(pwm_pin,GPIO.OUT)
pi_pwm = GPIO.PWM(pwm_pin,2000)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

ref_speed = 50
kp = 0.2
kd = 0.1

# Function to increment the counter
def increment_counter(channel):
    global counter
    global timer
    global angular_speed
    counter += 1

GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Add event detection to the interrupt pin
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=increment_counter)

duty = 0
error_prev = 0
try:
    while True:
        pi_pwm.ChangeDutyCycle(duty)
        time.sleep(calc_delay)
        counter_now = counter
        count_diff = counter - counter_prev
        angular_speed = count_diff*(360/140)/calc_delay #deg per sec
        linear_speed = angular_speed * (pi/180) * 4.75 #cm per sec

        counter_prev = counter
        error = ref_speed - linear_speed
        error_diff = (error - error_prev)/calc_delay
        duty = duty + kp*error + kd*error_prev
        if duty > 100:
            duty = 100
        if duty < 0:
            duty = 0
        error_prev = error

        print(linear_speed, "     ", error, "      ", duty)


except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()

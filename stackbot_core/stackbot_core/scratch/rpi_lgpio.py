import RPi.GPIO as GPIO
import time

# Define the pin number
interrupt_pin = 16  # Change this to the pin you want to use

# Initialize counter variable
counter = 0

# Function to increment the counter
def increment_counter(channel):
    global counter
    counter += 1
    print(counter)

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Add event detection to the interrupt pin
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=increment_counter)

try:
    while True:
        # Print the value of the counter
        # print(counter)
        # print(GPIO.input(interrupt_pin))
        # Delay for readability (optional)
        time.sleep(0.001)

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()

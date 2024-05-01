#  Blink an LED with the LGPIO library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import time
import lgpio

LED = 23

# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, LED)

try:
    while True:
        # Turn the GPIO pin on
        if lgpio.gpio_read(h, LED):
            print("1")
        
except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
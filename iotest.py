import time
import RPi.GPIO as GPIO

# Pins definitions
btn_pin = 2
#led_pin = 12

# Set up pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(btn_pin, GPIO.IN)
#GPIO.setup(led_pin, GPIO.OUT)

# If button is pushed, light up LED
try:
    while True:
        print("\033c")
        if GPIO.input(btn_pin):
            #GPIO.output(led_pin, GPIO.LOW)
            print("True")
        else:
            #GPIO.output(led_pin, GPIO.HIGH)
            print("False")
        time.sleep(0.5)

# When you press ctrl+c, this will be called
finally:
    GPIO.cleanup()

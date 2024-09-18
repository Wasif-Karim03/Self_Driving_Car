import Jetson.GPIO as GPIO
import time

# Define the GPIO pin number for the servo
servo_pin = 18  # Example pin, make sure it's the right GPIO pin number

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM on the servo pin at 50Hz
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency for the servo
pwm.start(0)  # Start with a duty cycle of 0

def set_servo_angle(angle):
    # Map the angle (0 to 180) to the duty cycle (2 to 12 for most servos)
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        # Move the servo to 0 degrees
        set_servo_angle(0)
        time.sleep(1)

        # Move the servo to 90 degrees
        set_servo_angle(90)
        time.sleep(1)

        # Move the servo to 180 degrees
        set_servo_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    pass

# Cleanup
pwm.stop()
GPIO.cleanup()

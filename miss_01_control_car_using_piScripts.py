#Here I am sending commands through raspi to control the car based on some python codes
import RPi.GPIO as GPIO
import time

# ESC Wiring:
# - ESC Signal Wire (white or yellow wire) → Connect to GPIO Pin 18 (PWM0) on Raspberry Pi.
# - ESC Power Wires (red and black) → Connect directly to the Traxxas battery.
# - ESC Ground Wire (black wire) → Connect to Raspberry Pi GND (any GND pin).

# Steering Servo Wiring:
# - Servo Signal Wire (white or yellow wire) → Connect to GPIO Pin 12 (PWM1) on Raspberry Pi.
# - Servo Power Wires (red and black) → Connect to the same power source as the ESC (Traxxas battery).
# - Servo Ground Wire (black wire) → Connect to Raspberry Pi GND (shared with ESC).

# GPIO Pin Definitions
ESC_PIN = 18   # PWM0 for ESC (Throttle control)
SERVO_PIN = 12 # PWM1 for Servo (Steering control)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Set PWM frequency for ESC and servo (50Hz is standard for both)
esc_pwm = GPIO.PWM(ESC_PIN, 50)   # 50Hz for ESC
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for Servo

# Start PWM with initial duty cycle (neutral position for ESC and servo)
esc_pwm.start(7)  # Neutral throttle for ESC
servo_pwm.start(7)  # Centered servo

# Helper function to control ESC (Throttle control)
def control_esc(throttle):
    """
    Controls the throttle of the car.
    throttle: value from -100 to 100 (where 0 is neutral, 100 is full forward, and -100 is full reverse)
    """
    # Map throttle from -100 to 100% to duty cycle between 5% and 10%
    duty_cycle = 7 + (throttle / 100) * 3  # Neutral at 7%, forward range from 7% to 10%, reverse from 7% to 5%
    esc_pwm.ChangeDutyCycle(duty_cycle)

# Helper function to control Servo (Steering control)
def control_servo(angle):
    """
    Controls the angle of the steering servo.
    angle: value from 0 to 180 (0 is fully left, 180 is fully right, and 90 is straight)
    """
    # Map angle from 0 to 180 degrees to duty cycle between 5% and 10%
    duty_cycle = 7 + (angle / 180) * 3  # 7% duty cycle is neutral, 10% is full right, 5% is full left
    servo_pwm.ChangeDutyCycle(duty_cycle)

# Example of controlling the car with the wiring described
try:
    # Move forward, turn, etc.
    pass

finally:
    # Cleanup
    esc_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()

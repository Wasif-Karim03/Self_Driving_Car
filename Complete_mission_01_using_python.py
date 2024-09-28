from pymavlink import mavutil
import time

# Connect to the Pixhawk via serial (adjust to the correct serial port and baud rate)
pixhawk = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

# Wait for a heartbeat from Pixhawk to establish communication
pixhawk.wait_heartbeat()

# Function to arm the Pixhawk (enables motor control)
def arm_pixhawk():
    pixhawk.mav.command_long_send(
        pixhawk.target_system, pixhawk.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    print("Vehicle Armed")

# Function to disarm the Pixhawk (stops motor control)
def disarm_pixhawk():
    pixhawk.mav.command_long_send(
        pixhawk.target_system, pixhawk.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("Vehicle Disarmed")

# Function to send throttle and steering commands
def set_speed_and_steering(throttle, steering):
    """
    throttle: Value between -1.0 to 1.0 (-1 is full reverse, 1 is full forward)
    steering: Value between -1.0 to 1.0 (-1 is full left, 1 is full right)
    """
    # Convert throttle and steering to appropriate servo values (PWM signal between 1000-2000)
    throttle_pwm = int(1500 + (500 * throttle))  # 1500 is neutral, 2000 is forward, 1000 is reverse
    steering_pwm = int(1500 + (500 * steering))  # 1500 is center, 2000 is right, 1000 is left

    # Send RC channel override for throttle (RC channel 3) and steering (RC channel 1)
    pixhawk.mav.rc_channels_override_send(
        pixhawk.target_system, pixhawk.target_component,
        steering_pwm, 0, throttle_pwm, 0, 0, 0, 0, 0)
    print(f"Throttle: {throttle_pwm}, Steering: {steering_pwm}")

# Function to read GPS data from Pixhawk
def read_gps():
    while True:
        msg = pixhawk.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print(f"Lat: {msg.lat / 1e7}, Lon: {msg.lon / 1e7}, Alt: {msg.alt / 1000}m")
        time.sleep(1)

# Main loop
if __name__ == "__main__":
    arm_pixhawk()  # Arm the vehicle
    
    # Example mission: move forward, turn, move backward, stop
    set_speed_and_steering(0.5, 0)  # Move forward with 50% throttle
    time.sleep(5)
    
    set_speed_and_steering(0.5, -1)  # Turn left
    time.sleep(1)
    
    set_speed_and_steering(0.5, 0)  # Move forward again
    time.sleep(5)
    
    set_speed_and_steering(-0.5, 0)  # Move backward
    time.sleep(3)
    
    set_speed_and_steering(0, 0)  # Stop the car
    disarm_pixhawk()  # Disarm the vehicle

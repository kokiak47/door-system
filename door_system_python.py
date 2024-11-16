import RPi.GPIO as GPIO
import time

# Define pins for the ultrasonic sensor
TRIGGER_PIN = 10
ECHO_PIN = 11

# Define distance threshold for door detection
OPEN_DISTANCE_THRESHOLD = 15 # Adjust as needed

# Define pins for LEDs and buzzer
GREEN_LED_PIN = 4
RED_LED_PIN = 3
BUZZER_PIN = 14
servo_motor  = 17
# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Setup ultrasonic sensor pins
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Setup LED and buzzer pins
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Setup servo pin
GPIO.setup(servo_motor, GPIO.OUT)

# Create PWM object for servo motor
servo_motor = GPIO.PWM(servo_motor, 100)  # Servo pin, frequency 50Hz

# Start PWM with initial duty cycle (servo at initial position)

servo_motor.start(0)
def measure_distance():
    # Send a pulse to trigger the ultrasonic sensor
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)
    
    pulse_start = 0
    pulse_end = 0

    # Wait for the echo signal to be received
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate pulse duration to determine distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 343 m/s (17150 cm/s)
    return distance

try:
    
    while True:
        # Measure distance using ultrasonic sensor
        distance = measure_distance()
        dici = 0
        # Check if the door is open (distance below threshold)
        if distance < OPEN_DISTANCE_THRESHOLD:
         for _ in range(10):
           if distance < OPEN_DISTANCE_THRESHOLD:
               dici +=1
               print(dici)
               time.sleep(1)
               if dici >=10:
                # Move the servo motor to close the door
                servo_motor.ChangeDutyCycle(0)  # Adjust this duty cycle to close the door to the desired position
                  # Wait for 1 second (adjust as needed)
                print(distance)
                # Turn on red LED and buzzer to indicate door closed
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(0.1)  # Buzz for 0.5 seconds
                GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turn off the buzzer
        else:
            # Turn on green LED to indicate door open
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            GPIO.output(RED_LED_PIN, GPIO.LOW)
            # Stop servo motor
            servo_motor.ChangeDutyCycle(0)

        # Add a delay between distance measurements
        time.sleep(0.5)

except KeyboardInterrupt:
    # Clean up GPIO
    servo_motor.stop()
    GPIO.cleanup()

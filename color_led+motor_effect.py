from picamera2 import Picamera2
import cv2
import numpy as np
from adafruit_crickit import crickit
from adafruit_motor import stepper
import time
import threading

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# Initialize crickit hardware
LED_PIN = crickit.SIGNAL1  # LED connected to SIGNAL1
crickit.seesaw.pin_mode(LED_PIN, crickit.seesaw.OUTPUT)
STEP_MOTOR = crickit.stepper_motor  # Stepper motor connected to MOTOR section

# Motor parameters
STEPS_PER_REV = 200  # Number of steps for one full revolution
MOTOR_SPEED = 0.01   # Moderate speed for motor movement (delay between steps)

# Define color ranges in HSV
color_ranges = {
    "Red": [(0, 120, 70), (10, 255, 255)],
    "Green": [(40, 50, 50), (80, 255, 255)],
    "Blue": [(100, 150, 0), (140, 255, 255)],
    "Yellow": [(20, 100, 100), (30, 255, 255)]
}

def blink_led():
    """Blink LED 3 times with 0.5s interval"""
    for _ in range(3):
        crickit.seesaw.digital_write(LED_PIN, True)  # Turn LED on
        time.sleep(0.25)  # LED on for 0.25s
        crickit.seesaw.digital_write(LED_PIN, False)  # Turn LED off
        time.sleep(0.5)   # 0.5s interval between blinks

def run_motor():
    """Run stepper motor for one full revolution forward"""
    for _ in range(STEPS_PER_REV):
        STEP_MOTOR.onestep(direction=stepper.FORWARD)
        time.sleep(MOTOR_SPEED)

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Flags to track detections
        green_detected = False
        red_or_blue_detected = False

        for color_name, (lower, upper) in color_ranges.items():
            lower_np = np.array(lower)
            upper_np = np.array(upper)

            mask = cv2.inRange(hsv, lower_np, upper_np)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # Filter out small blobs
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)

                    # Set flags for green and red/blue detection
                    if color_name == "Green":
                        green_detected = True
                    elif color_name in ["Red", "Blue"]:  # Check for red or blue
                        red_or_blue_detected = True

        # Conditional logic for LED blinking, motor running, and printing
        if green_detected:
            print("GREEN DETECTED, IT'S A GO - LET'S RUN THE STEPPER MOTOR")
            # Create threads for blinking LED and running motor
            blink_thread = threading.Thread(target=blink_led)
            motor_thread = threading.Thread(target=run_motor)
            # Start both threads to run simultaneously
            blink_thread.start()
            motor_thread.start()
            # Wait for both threads to complete before processing the next frame
            blink_thread.join()
            motor_thread.join()
        if red_or_blue_detected:
            print("SAW A COLOR EITHER THAN GREEN WHICH IS RED OR BLUE. IT'S A NO GO!")

        cv2.imshow("Color Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    crickit.seesaw.digital_write(LED_PIN, False)  # Turn off LED
    cv2.destroyAllWindows()
    picam2.stop()
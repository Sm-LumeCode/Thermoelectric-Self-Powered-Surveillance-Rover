#!/usr/bin/env python3
import os
import time
import math
import cv2
from roboflow import Roboflow
import RPi.GPIO as GPIO
# ---------------- ROBOT CONSTANTS ----------------
# Motor Driver Pins
M1_IN1 = 17
M1_IN2 = 18
M2_IN3 = 22
M2_IN4 = 23
M1_EN = 27
M2_EN = 13
# Ultrasonic Sensor Pins
TRIG = 24
ECHO = 25
# Servo Motor Pin
SERVO = 12
SERVO_FREQ = 50
# Alert Pins
BUZZER = 5
FIRE_LED = 16
KNIFE_LED = 20
# Robot Parameters
FWD_SPEED = 80
TURN_SPEED = 70
OBSTACLE_DIST_CM = 20
TURN_DURATION_SEC = 0.6
BACKUP_DURATION_SEC = 0.2
SENSOR_CENTER_ANGLE = 90
SONIC_SPEED_CM_PER_SEC = 34300
ULTRASONIC_TIMEOUT_SEC = 0.05
# ---------------- GPIO SETUP ----------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Motors
GPIO.setup(M1_IN1, GPIO.OUT)
GPIO.setup(M1_IN2, GPIO.OUT)
GPIO.setup(M2_IN3, GPIO.OUT)
GPIO.setup(M2_IN4, GPIO.OUT)
GPIO.setup(M1_EN, GPIO.OUT)
GPIO.setup(M2_EN, GPIO.OUT)
pwm1 = GPIO.PWM(M1_EN, 100)
pwm2 = GPIO.PWM(M2_EN, 100)
pwm1.start(0)
pwm2.start(0)
# Ultrasonic
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
# Servo
GPIO.setup(SERVO, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO, SERVO_FREQ)
servo_pwm.start(0)
# Buzzer and LEDs
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(FIRE_LED, GPIO.OUT)
GPIO.setup(KNIFE_LED, GPIO.OUT)

GPIO.output(BUZZER, GPIO.LOW)
GPIO.output(FIRE_LED, GPIO.LOW)
GPIO.output(KNIFE_LED, GPIO.LOW)

# ---------------- UTILITY FUNCTIONS ----------------

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.000002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()
    timeout_start = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if (time.time() - timeout_start) > ULTRASONIC_TIMEOUT_SEC:
            return float('inf')

    timeout_start = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
        if (time.time() - timeout_start) > ULTRASONIC_TIMEOUT_SEC:
            return float('inf')

    duration = stop_time - start_time
    distance = (duration * SONIC_SPEED_CM_PER_SEC) / 2

    if distance > 400 or distance < 2:
        return float('inf')
    return distance

# ---------------- FIXED MOVEMENT LOGIC ----------------

def move_forward(speed=FWD_SPEED):
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN3, GPIO.HIGH)
    GPIO.output(M2_IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def move_backward(speed=FWD_SPEED):
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    GPIO.output(M2_IN3, GPIO.LOW)
    GPIO.output(M2_IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_left(speed=TURN_SPEED):
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    GPIO.output(M2_IN3, GPIO.HIGH)
    GPIO.output(M2_IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_right(speed=TURN_SPEED):
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN3, GPIO.LOW)
    GPIO.output(M2_IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def stop():
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN3, GPIO.LOW)
    GPIO.output(M2_IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# ---------------- ROBOFLOW SETUP ----------------
rf = Roboflow(api_key="alsDWvXDZCNT0ZpgVQ3k")
fire_project = rf.workspace().project("fire-detection-8icva-s5tlv")
fire_model = fire_project.version(1).model
knife_project = rf.workspace().project("knife-detection-obh5j-f3sag")
knife_model = knife_project.version(2).model

# ---------------- MAIN LOOP ----------------
try:
    print("Starting Robot with Fire and Knife Detection...")
    set_servo_angle(SENSOR_CENTER_ANGLE)

    last_detection_time = time.time()

    while True:

        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        # ------- Obstacle Avoidance Logic (Arduino Behaviour) -------
        if not math.isinf(distance) and distance < OBSTACLE_DIST_CM:
            print("🚨 Obstacle Detected")
            stop()
            time.sleep(0.2)

            move_backward(FWD_SPEED * 0.7)
            time.sleep(0.4)
            stop()

            set_servo_angle(150)
            time.sleep(0.4)
            dist_left = get_distance()
            print(f"Left: {dist_left:.2f} cm")

            set_servo_angle(30)
            time.sleep(0.4)
            dist_right = get_distance()
            print(f"Right: {dist_right:.2f} cm")

            set_servo_angle(90)
            time.sleep(0.2)

            if dist_left > OBSTACLE_DIST_CM and dist_left > dist_right:
                print("⬅ Turning Left")
                turn_left()
                time.sleep(0.45)

            elif dist_right > OBSTACLE_DIST_CM and dist_right > dist_left:
                print("➡ Turning Right")
                turn_right()
                time.sleep(0.45)

            else:
                print("⏪ Reversing Again - No Clear Path")
                move_backward(FWD_SPEED * 0.7)
                time.sleep(0.5)

            stop()
            time.sleep(0.2)

        else:
            move_forward(FWD_SPEED)

        # -------- Detection System  --------
        if time.time() - last_detection_time > 5:
            print("Capturing frame for detection...")
            os.system("libcamera-jpeg -o frame.jpg --width 640 --height 480 --quality 90")
            image = cv2.imread("frame.jpg")

            fire_result = fire_model.predict("frame.jpg", confidence=40, overlap=30).json()
            knife_result = knife_model.predict("frame.jpg", confidence=40, overlap=30).json()

            fire_detected = len(fire_result["predictions"]) > 0
            knife_detected = len(knife_result["predictions"]) > 0

            if fire_detected or knife_detected:
                GPIO.output(BUZZER, GPIO.HIGH)
            else:
                GPIO.output(BUZZER, GPIO.LOW)

            GPIO.output(FIRE_LED, GPIO.HIGH if fire_detected else GPIO.LOW)
            GPIO.output(KNIFE_LED, GPIO.HIGH if knife_detected else GPIO.LOW)

            for p in fire_result["predictions"]:
                x, y, w, h = int(p["x"]), int(p["y"]), int(p["width"]), int(p["height"])
                cv2.rectangle(image, (x - w//2, y - h//2), (x + w//2, y + h//2), (0, 255, 0), 2)
                cv2.putText(image, "FIRE", (x - w//2, y - h//2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)

            for p in knife_result["predictions"]:
                x, y, w, h = int(p["x"]), int(p["y"]), int(p["width"]), int(p["height"])
                cv2.rectangle(image, (x - w//2, y - h//2), (x + w//2, y + h//2), (0,0,255), 2)
                cv2.putText(image, "KNIFE", (x - w//2, y - h//2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255),2)

            cv2.imshow("Fire & Knife Detection", image)
            cv2.waitKey(1)

            last_detection_time = time.time()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped by User.")
finally:
    print("Cleaning up...")
    stop()
    pwm1.stop()
    pwm2.stop()
    servo_pwm.stop()
    GPIO.output(BUZZER, GPIO.LOW)
    GPIO.output(FIRE_LED, GPIO.LOW)
    GPIO.output(KNIFE_LED, GPIO.LOW)
    GPIO.cleanup()
    cv2.destroyAllWindows()

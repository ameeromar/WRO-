#Libraries
import RPi.GPIO as GPIO
import time
from gpiozero import Servo
import cv2 
import numpy as np

import threading
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
# time.sleep(10)
focal_length = 100

#set GPIO Pins
GPIO_TRIGGER = 12
GPIO_ECHO = 6

GPIO2_TRIGGER = 20
GPIO2_ECHO = 26
lift=0
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

l=0
oca=False
object_size = 10  
green_distance=1
red_distance=1
motor_in1 = 23
motor_in2 = 22
motor_ena = 24 

GPIO.setup(motor_in1, GPIO.OUT)
GPIO.setup(motor_in2, GPIO.OUT)
GPIO.setup(motor_ena, GPIO.OUT)

# GPIO.output(motor_in1, GPIO.HIGH)
# GPIO.output(motor_in2, GPIO.LOW)

p=GPIO.PWM(motor_ena,1000)
p.start(25)
p.ChangeDutyCycle(60)
cap = cv2.VideoCapture(0)
GPIO.output(motor_in1, GPIO.HIGH)
GPIO.output(motor_in2, GPIO.LOW)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO2_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO2_ECHO, GPIO.IN)
ora=False
l=0
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
pwm=GPIO.PWM(4, 50)
pwm.start(0)
if not cap.isOpened():
    raise Exception("Error opening the camera. Make sure your camera is connected and working.")



def camera_capture():
    while True:
        ret, frame = cap.read()
        global green_distance
        global red_distance
        if not ret:
            break
    
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        centroid_frame = frame.copy()        
        lower_green = np.array([40, 40, 40])  
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_green:
            cnt_green = max(contours_green, key=cv2.contourArea)
            M_green = cv2.moments(cnt_green)
            if M_green['m00'] != 0:
                cx_green = int(M_green['m10'] / M_green['m00'])
                cy_green = int(M_green['m01'] / M_green['m00'])
                green_apparent_size = cv2.contourArea(cnt_green)
                green_distance = (focal_length * object_size) / green_apparent_size

                centroid_frame = frame.copy()
                cv2.drawContours(centroid_frame, [cnt_green], -1, (0, 255, 0), 2)
                cv2.circle(centroid_frame, (cx_green, cy_green), 5, (0, 0, 255), -1)
                cv2.putText(centroid_frame, f"Green Distance: {green_distance:.2f} units", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        lower_red1 = np.array([0, 120, 70])    
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70]) 
        upper_red2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnt_red = max(contours_red, key=cv2.contourArea)
        M_red = cv2.moments(cnt_red)
        if M_red['m00'] != 0:
            cx_red = int(M_red['m10'] / M_red['m00'])
            cy_red = int(M_red['m01'] / M_red['m00'])
            red_apparent_size = cv2.contourArea(cnt_red)
            red_distance = (focal_length * object_size) / red_apparent_size
            centroid_frame = frame.copy()
            cv2.drawContours(centroid_frame, [cnt_red], -1, (0, 0, 255), 2)
            cv2.circle(centroid_frame, (cx_red, cy_red), 5, (0, 255, 0), -1)
            cv2.putText(centroid_frame, f"Red Distance: {red_distance:.2f} units", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow('Live Video with Object Centroids and Distances', centroid_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break
        
        if not ret:
            break
    cap.release()
    cv2.destroyAllWindows()

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
def distance2():
    # set Trigger to HIGH
    GPIO.output(GPIO2_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO2_TRIGGER, False)
 
    Start1Time = time.time()
    Stop1Time = time.time()
 
    # save StartTime
    while GPIO.input(GPIO2_ECHO) == 0:
        Start1Time = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO2_ECHO) == 1:
        Stop1Time = time.time()
 
    # time difference between start and arrival
    Time1Elapsed = Stop1Time - Start1Time
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance2 = (Time1Elapsed * 34300) / 2
 
    return distance2
def other_task():
        while True:
            global green_distance
            global red_distance
            global lift
            global ora 
            global l
            dist = distance()
            dist2=distance2()
            print ("Measured Distance = %.1f  cm" % dist)
            print ("Measured Distance = %.1f cm" % dist2)
            time.sleep(0.5)
            if  dist> 40  and (lift==2 or lift==0):
                    lift=2
                    print("Orange go roght")
                    pwm.start(5)
                    # pwm.ChangeDutyCycle(10) # neutral position
                    p.ChangeDutyCycle(60) 
                    time.sleep(1.2)
                    # pwm.stop()

                    if(ora==False):
                        l=l+1
                        ora=True
            elif dist2 > 40 and (lift==1 or lift==0):
                    lift=1
                    print("Blue for lift")
                    pwm.start(10)
                    p.ChangeDutyCycle(60)
                    # pwm.ChangeDutyCycle(5)            
                    time.sleep(1.2)
                    # pwm.stop()

                    time.sleep(1)
                    if(ora==False):
                        l=l+1
                        ora=True 
            elif 0.00 < red_distance < 0.05:
                print("Red go right===========")
                pwm.start(5)
                p.ChangeDutyCycle(60)
                # pwm.ChangeDutyCycle(5)
                time.sleep(1)
                # pwm.stop()   
            elif  0.00 < green_distance < 0.05:
                print("Green go lift=========")
                pwm.start(10)
                p.ChangeDutyCycle(60)
                # pwm.ChangeDutyCycle(10)   
                time.sleep(0.5)
                # pwm.stop()

            else :
                print("move")
                print(red_distance)
                time.sleep(0.5)
                pwm.start(7)
                p.ChangeDutyCycle(60)
                # pwm.ChangeDutyCycle(7) 
                time.sleep(0.5)
                # pwm.stop()
                ora=False
            

if __name__ == "__main__":
    
    
        # Create separate threads for camera capture and other task
        camera_thread = threading.Thread(target=camera_capture)
        other_task_thread = threading.Thread(target=other_task)
        # distance_thread=threading.Thread(target=distance)
        # distance2_thread=threading.Thread(target=distance2)

        # Start the threads
        camera_thread.start()
        other_task_thread.start()
        # distance2_thread.start()
        # distance_thread.start()

        # # Wait for both threads to finish
        # camera_thread.join()
        # other_task_thread.join()
        # distance_thread.join()
        # distance2_thread.join()

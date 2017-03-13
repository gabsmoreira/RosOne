
import numpy as np
import cv2
import cv2.cv as cv
import time
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys, select, termios, tty
bridge = CvBridge()
pub = None
media = []
centro = []
atraso = 1.5
resultado = None

def check_tennis_ball(img):
    global resultado
    lower_green = np.array([20,70,100])
    upper_green = np.array([55,255,255])
    blur =cv2.GaussianBlur(img,(5,5),10)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1.5, 100)


    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            if mask_rgb[y][x][0] == 255:
                if mask_rgb[y][x][1] == 255:
                    if mask_rgb[y][x][2] == 255:
                        cv2.circle(img, (x, y), r, (0, 255, 0), 4)
                        cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                        resultado =  [x,r]





def turn_right():
    global pub
    print("Direita")
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.3)
    twist = Twist()
    twist.linear.x = 0*speed; twist.linear.y = 0*speed; twist.linear.z = 0*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -1*turn
    pub.publish(twist)


def turn_left():
    global pub
    print("Esquerda")
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.3)
    twist = Twist()
    twist.linear.x = 0*speed; twist.linear.y = 0*speed; twist.linear.z = 0*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1*turn
    pub.publish(twist)


def forward():
    global pub
    print("Frente")
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)
    twist = Twist()
    twist.linear.x = 1*speed; twist.linear.y = 0*speed; twist.linear.z = 0*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1*turn
    pub.publish(twist)  


def stop():
    global pub
    print("PAREI")
    speed = rospy.get_param("~speed", 0)
    turn = rospy.get_param("~turn", 0)
    twist = Twist()
    twist.linear.x = 0*speed; twist.linear.y = 0*speed; twist.linear.z = 0*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0*turn
    pub.publish(twist)  

    

def recebe(imagem):
    global cv_image
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = (lag.secs+lag.nsecs/1000000000.0)
    if delay > atraso:
        return 
    #print("DELAY", delay)
    try:
        antes = time.clock()
        # Converte para imagem OpenCV
        cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("video", cv_image)
        cv2.waitKey(1)
        check_tennis_ball(cv_image)
        depois = time.clock()
        #print ("TEMPO", depois-antes)
    except CvBridgeError as e:
        print(e)



def position(x,r):
    check_tennis_ball(cv_image)
    if  340 > x > 300:
        if r < 110:
            forward()
        if r >= 110:
            return 1
    elif x < 300:
        turn_left()
    elif x > 340:
        turn_right()
    else:
        stop()


def main():
    global pub
    rospy.init_node("tennis_ball")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebedor = rospy.Subscriber("/camera/image_raw", Image, recebe, queue_size=10, buff_size = 2**24)
    #imgtime = imagem.header.stamp
    #camera = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        #check_tennis_ball(cv_image)
        if resultado != None:
            x = resultado[0]
            r = resultado[1]
            if position(x,r) == 1:
                print("EH TETRA")
                stop()
                break
            else:
                turn_right()
                check_tennis_ball(cv_image)



if __name__ == "__main__":
    main()

'''
ret, img = camera.read()
        returns = check_tennis_ball(img)
        if returns == None:
            while True:
                check_tennis_ball(cap)
                turn_right()
                if returns != None:
                    break'''
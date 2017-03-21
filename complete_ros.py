
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
from neato_node.msg import Bump

import sys, select, termios, tty
bridge = CvBridge()
pub = None
media = []
centro = []
atraso = 1.5
resultado = None
meio = np.arange(300,340,1)

def check_tennis_ball(img):
	global resultado
	global mask
	lower_green = np.array([40,50,50])
	upper_green = np.array([100,255,255])
	blur =cv2.GaussianBlur(img,(5,5),10)
	gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_green, upper_green)
	mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
	edges = cv2.Canny(mask,100,200)
	circles = cv2.HoughCircles(edges, cv.CV_HOUGH_GRADIENT, 1.8, 1000, param1 = 100,param2=40)
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, r) in circles:
			if mask_rgb[y][x][0] == 255:
				if mask_rgb[y][x][1] == 255:
					if mask_rgb[y][x][2] == 255:
						cv2.circle(img, (x, y), r, (0, 255, 0), 4)
						cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
						resultado =  [x,r]


	cv2.imshow("mask",mask)





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
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0*turn
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
	global meio
	global resultado
	check_tennis_ball(cv_image)
	if x in meio:
		if r < 50:
			forward()
		else:
			return 1
	elif x < 300:
		turn_left()
	elif x > 340:
		turn_right()





def bumper_callback(msg):
	global speed
	global pub
	global bumped
	if(msg.leftFront == 1 or msg.leftSide == 1 or msg.rightFront == 1 or msg.rightSide == 1):
		bumped = True
	else:
		bumped = False


def main():
	global pub
	global resultado
	global bumped
	rospy.init_node('tennis_ball')
	bmpSub = rospy.Subscriber('/bump', Bump, bumper_callback)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebedor = rospy.Subscriber("/camera/image_raw", Image, recebe, queue_size=10, buff_size = 2**24)
	#r = rospy.Rate(10)
	bumped = False
	while not rospy.is_shutdown():
		print(bumped)
		if bumped:
			stop()
			print("REH")
			speed = rospy.get_param("~speed", -1)
			turn = rospy.get_param("~turn", 1)
			twist = Twist()
			twist.linear.x = 1*speed; twist.linear.y = 0*speed; twist.linear.z = 0*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1*turn
			pub.publish(twist)
			time.sleep(1)
			stop()
			time.sleep(1)
		elif not bumped:
			if resultado != None:
				x = resultado[0]
				r = resultado[1]
				posicao = position(x,r)
				if posicao == 1:
					print("EH TETRA")
					stop()
					break
				else:
					#turn_right()
					check_tennis_ball(cv_image)

				resultado = None





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

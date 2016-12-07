#!/usr/bin/env python


import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
# Add additional imports for each of the message types used


wheel_radius = 3.5 / 100.0
wheel_base = 23.0 / 100.0

def publishTwist(linearVelocity, angularVelocity):

	global pub
	msg = Twist()
	msg.linear.x = linearVelocity
	msg.angular.z = angularVelocity
	pub.publish(msg)


#drive to a goal subscribed as /move_base_simple/goal
#def navToPose(goal):

#    print "spin!"

#    print "move!"
    
#	print "spin!"
    
#	print "done"
#	pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    pass  # Delete this 'pass' once implemented




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global pub
	r = wheel_radius
	b = wheel_base

	lin_vel = (r / 2) * (u1 + u2)
	ang_vel = (r / b) * (u1 - u2)
	
	twist_msg = Twist();
	stop_msg = Twist();
	
	twist_msg.linear.x = lin_vel
	twist_msg.angular.z = ang_vel
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0

	now = rospy.Time.now().secs
	while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		pub.publish(twist_msg)
	pub.publish(stop_msg)




#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose
	
	initX = pose.pose.position.x
	initY = pose.pose.position.y
	atTarget = False
	
	while (not atTarget and not rospy.is_shutdown()):
		currentX = pose.pose.position.x
		currentY = pose.pose.position.y
		
		currentDistance = math.sqrt(math.pow((currentX - initX), 2) + math.pow((currentY - initY), 2))
		if (currentDistance >= distance) :
			atTarget = True
			publishTwist(0, 0)
		else:
			publishTwist(speed, 0)
			rospy.sleep(0.15)

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global odom_list
	global pose
	
	global theta
	print theta
	tol = 2
	rospy.sleep(.1)
	thetaDegree = ((theta * 180) / math.pi)
	goal = (thetaDegree + angle)
	angleRadian = (angle * math.pi / 180)
#	if (goal > math.pi):
#		goal -= (math.pi * 2)
#	elif (goal < (-1 * math.pi)):
#		goal += (math.pi * 2)
	
	while (thetaDegree < goal - tol or thetaDegree > goal + tol):
		thetaDegree = ((theta * 180) / math.pi)
		if (angleRadian < 0):
			publishTwist(0, math.pi / -15)
		else:
			publishTwist(0, math.pi / 15)
			
		print "theta Degree"
		print thetaDegree
			
		print "goal"
		print goal
		rospy.sleep(.1)
		
	publishTwist(0, 0)
	rospy.is_shutdown()
	
	
def executeTrajectory():
	driveStraight(.5, .6)
	rotate(-90)
	driveStraight(.5, .45)
	rotate(135)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global theta
    
    lw = speed * (radius + (wheel_base/2))
    rw = speed * (radius - (wheel_base/2))
	linearSpeed = (lw + rw) / 2
	
	
	finalAngle = (theta + angle)
	
	while (theta < finalAngle - 2 or theta > finalAngle + 2):
		if (angle < 0):
			publishTwist(linearSpeed, -1 * speed)
		else:
			publishTwist(linearSpeed, speed)
			rospy.sleep(.1)
		publishTwist(0, 0)



#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        pass  # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#	rospy.Timer(rospy.Duration(.01), timerCallback)
#def timerCallback(event):
#	global pose
#	pose = Pose()

#    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    

#    pass # Delete this 'pass' once implemented


#def tCallback(event):

#	global pose
#	global xposition
#	global yposition
#	global theta
	
#	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
#	(position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
#	pose.position.x=position[0]
#	pose.position.y=position[1]

#	odomW = orientation
#	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
#	roll, pitch, yaw = euler_from_quaternion(q)
	#convert yaw to degrees
#	pose.orientation.z = yaw
#	theta = math.degrees(yaw)	

#def readOdom(msg):
#	global pose
#	global odom_tf
	
#	pose = msg.pose
#	geo_quat = pose.pose.orientation
  
#	odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
#	(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
#		rospy.Time.now(),
#		"base_footprint", 
#		"odom")
#	rospy.sleep(1)


#def odometryCb(msg):
#	print msg.pose.pose
#	rospy.sleep(1)

def odomCallback(msg):
	global x
	global y
	global theta
	global pose
	global odom_tf
	global odom_list
	px = msg.pose.pose.position.x
	py = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	
	x = px
	y = py
	theta = yaw
	
	pose = msg.pose
	geo_quat = pose.pose.orientation
  
	odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
	(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
		rospy.Time.now(),
		"base_footprint", 
		"odom")

#if bump data is received, process here
#data.bumper: LEFT (0), CENTER (1), RIGHT (2)
#data.state: RELEASED(0), PRESSED(1)
def readBumper(data):
    global bump
    if (data.state == BumperEvent.PRESSED):
        bump = True
        executeTrajectory()
    else:
        bump = False
    rospy.loginfo("Bumper Event")
    rospy.loginfo(data.bumper)

	
# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('kencolpritt_lab2')

	# These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
	global pub
	global pose
	global odom_tf
	global odom_list
	global theta
	# Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
	#bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	sub = rospy.Subscriber('/odom', Odometry, odomCallback)
	#rospy.Timer(rospy.Duration(.01), readOdom) #timer callback for robot location

    # Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
	odom_tf = tf.TransformBroadcaster()
	odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    # Use this command to make the program wait for some seconds
	rospy.sleep(1)

	


	print "Starting Lab 2"

    #make the robot keep doing something...
    #rospy.Timer(rospy.Duration(...), timerCallback)
	while not rospy.is_shutdown():
		#spinWheels(3, 3, 10)
		#driveStraight(.5, 15)
		#rotate(30)
		#executeTrajectory()
		driveArc(5, .5, 30)
    # Make the robot do stuff...

	print "Lab 2 complete!"


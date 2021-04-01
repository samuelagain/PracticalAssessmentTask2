#!/usr/bin/env python3

import rospy
import os
import actionlib
import multiprocessing

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Global coordinates stroed in shared memory to be access by odemtry callback and main process
x_pos = multiprocessing.Value('d', 0.0)
y_pos = multiprocessing.Value('d', 0.0)

#Function to read from a csv file into a 2D Vector, Each vector is <x,y>
def parseCSV( filename):

	coordsList=[]

	with open(filename) as fp:
		for line in fp:
			linearray = line.split(',')
			coord=[]
			coord.append(float(linearray[0]))
			coord.append(float(linearray[1]))
			coord.append(float(0))
			coordsList.append(coord)

	fp.close()
	return coordsList

#Function to overwrite the coordinates.csv file with the coordinates and their outcome
def saveCoords( coordsList, filename):

	with open(filename, 'w') as fp:
		fp.seek(0)
		for coord in coordsList:
			line = str(coord[0])+','+str(coord[1])
			if(coord[2]>0.5):
				line += ', success\n'
			elif(coord[2]<-0.5):
				line += ', failure\n'
			else:
				line += ', unattempted\n'
			fp.write(line)
		fp.truncate()
		fp.close()

#Callback function to overwrite global shared-memory variables of robot coords
def odomCallback(msg):
	x_pos = msg.pose.pose.position.x
	y_pos = msg.pose.pose.position.y

#function for created process to maintain odemtry subscriber callback 
def odemtrysubscriber():
		rospy.init_node("odomtracker" , anonymous=True)
		rospy.Subscriber("/odometry/filtered", Odometry, odomCallback)
		rospy.spin()

def csvgoals():
	#Global coordinates stroed in shared memory to be access by odemtry callback and main process
	#set to zero incase telemetry fails to report coords
	x_pos = multiprocessing.Value('d', 0.0)
	y_pos = multiprocessing.Value('d', 0.0)

	#create process to maintain odemtry subscriber callback
	p1 = multiprocessing.Process(target=odemtrysubscriber)
	p1.start()

	print("herloooo!")
	#remaining process to maintain movebase server for goal seeking
	rospy.init_node("simple_goal_seeker" , anonymous=True)

	filename = "coordinates.csv"
	coordslist = parseCSV(filename)

	print("Coords_list loaded!")

	##typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient

	#MoveBaseClient ac("move_base", true)

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()

	print("Connected!")

	#iterate through goal coordinates and attempt to reach each with move base server
	index = 0
	for coord in coordslist:

		#intiilaze and set up goal
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "base_link"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = coord[0] - x_pos.value
		goal.target_pose.pose.position.y = coord[1] - y_pos.value
		goal.target_pose.pose.orientation.w = 1.0

		#send goal to server
		print("Sending goal")    
		client.send_goal(goal)

		#wait for the robot to do its thing (hopefully get to the coordinate)
		wait = client.wait_for_result()

		#once coordinate is reached or movebase gives up, the outcome is saved in the third item in each vector
		if(client.get_result()):
			print("Successfully reach Coordinate!")
			coordslist[index][2] = 1
		else:
			print("Failed to reach Coordinate!")
			coordslist[index][2] = -1

		index += 1

	#after all coordinated have been attempted, the csv is rewritten with outcomes saved as well
	saveCoords( coordslist, filename)
	p1.terminate()
	return 0

if __name__ == '__main__':
	try:
		csvgoals()
	except rospy.ROSInterruptException: pass
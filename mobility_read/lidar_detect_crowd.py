#!/usr/bin/env python
from __future__ import division

import rospy
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32

# global variables declaration
pubDistance = rospy.Publisher('/input/lidar/distances', Float32MultiArray, queue_size=1)
pubAngle = rospy.Publisher('/input/lidar/angles', Float32MultiArray, queue_size=1)
numObjects = rospy.Publisher('/input/lidar/numobjects', UInt32, queue_size=1)

maxNum = 10000  # used to compare distances, this number must be more than the max number the data.ranges returns
# (not confirmed, but currently seems to be 1048, although there seems to be numbers missing from around 300 - 1048)
objectCounter = 0  # variable to keep count of how many objects were found
objectMinAngle = 6  # an object must have a minimum of this many angles to be identified
# the lidar module publishes around 35 data per second
# the "publishCounter" variable is made to control the rate in which the data gets published
# it acts as a counter to check how many iterations the callback has done
# ignore the data (do not publish) if the counter isn't equals to the iteration we want to publish in
publishCounter = 0
publishRate = 70
# Arrays to store all the data to be published
# Data stored are the average of the angles and distances for every object found
objectAnglesPublish = []
objectDistancesPublish = []


# Method to change all the negative numbers to be positive
def modAngle(i):
    while i < 0:
        i = i + 3600;
    # endWhile

    return i

# endDef

def obstructed(data, min, max):
    # Declaration of global variables modified in this method
    global objectCounter  # iterator of how many objects are found
    global objectAnglesPublish  # array to store the angles of all objects, to be published
    global objectDistancesPublish  # array to store the distances of all objects, to be published

    # The lidar checks for obstacles every 0.1 degree
    # For this case, we don't need it to be very precise
    # So we can ignore 9 out of 10 values in 1 degree
    # We will only store the closest one out of the 10 values

    counter = 1  # variable to store how many obstacles found consecutively (next to each other in terms of angles)
    objectCounter = 0  # clear the object counter every time this method is called
    objectDistances = []  # array to store the distances of one object
    objectAngles = []  # array to store the angles of one object
    objectAnglesPublish = []  # initialise the angles array to be published so that it gets emptied out every time this method gets called
    objectDistancesPublish = []  # initialise the distances array to be published so that it gets emptied out every time this method gets called
    # average = 0                   # variable to store the average angle of the objects found

    i = min

    while i < max:
        # exclude the disruptions caused by the 4 bars around the lidar
        # the disruptions that would disturb angles 950-2650 are at 1340-1370 and 2240-2270
        if (i in range(1340, 1370)) or (i in range(2240, 2270)):
            i += 1
            continue

        # this code onwards will run if the angle is not of the angles where the disruptions exist
        # re-initialise the current distance in every iteration
        currDistance = maxNum

        # Check for the closest distance in 1 degree
        for j in range(0, 10):
            i += 1
            a = modAngle(i)
            if data.ranges[a] < currDistance:
                currDistance = data.ranges[a]
                currAngle = a
            # endIf
        # endFor

        # First obstacle found on consecutive angle iteration
        if counter == 1:
            prevDistance = currDistance
            prevAngle = currAngle

            objectDistances.append(currDistance)
            objectAngles.append(currAngle)

            counter += 1
            continue
        # endIf

        # This code onwards will run if it's not the first obstacle found consecutively
        if counter == objectMinAngle:
            # Obstacle detected
            objectCounter += 1
        # endIf

        if compareDistance(prevDistance, currDistance) == True:
            if currDistance < 3:
                # ignore if distance is more than 3 meters
                counter += 1
            # endIf

            prevDistance = currDistance
            prevAngle = currAngle

            objectDistances.append(currDistance)
            objectAngles.append(currAngle)
        else:
            if counter >= objectMinAngle:
                # object detected
                # put the data of the object inside the arrays to be published
                printObjectDetails(objectDistances, objectAngles)

                average = (objectAngles[0] + objectAngles[-1]) / 2
                objectAnglesPublish.append(average)

                average = (objectDistances[0] + objectDistances[-1]) / 2
                objectDistancesPublish.append(average)
            # endIf

            counter = 1

            objectDistances = []
            objectAngles = []
        # endIf
    # endWhile


# endDef

# Check if the distances between the currently found obstacle and the previously found obstacle are similar
# If the distance are not similar, they are considered a different object
def compareDistance(prevDistance, currDistance):
    difference = prevDistance - currDistance

    if -0.1 < difference < 0.1:
        return True

    return False


# endDef

def printObjectDetails(objectDistances, objectAngles):
    # Variables to store the edited values to be printed
    distancesEdited = []  # from m to cm
    anglesEdited = []  # from tenth of degree to degree

    # edit the data inside the arrays
    distancesEdited = map(lambda item: item * 100, objectDistances)
    anglesEdited = map(lambda item: item / 10, objectAngles)

    # print the object details for every object we found
    print "Object",
    print objectCounter,
    print ":"

    print "Distances: ",
    print str(["{0:3.1f}".format(item) for item in distancesEdited]).replace("'", "")

    print "Angles   : ",
    print str(["{0:3.1f}".format(item) for item in anglesEdited]).replace("'", "")

# endDef

def callback(data):
    # data.ranges and data.intensities
    # index is from 0..3599, where
    # - 0 is rear
    # - 900 is (base's) right
    # - 1800 is front
    # - 2700 is (base's) left

    # Declaration of global variables modified in this method
    global publishCounter
    global objectAnglesPublish
    global objectDistancesPublish

    # Variables to store the edited values of angles and distances
    avgAnglesEdited = []  # from tenth of degrees to a degree
    avgDistancesEdited = []  # from m to cm

    publishCounter += 1

    # only publish if the counter is equals to the rate we want to publish in
    if publishCounter == publishRate:
        obstructed(data, 950, 2650)

        # edit the data inside the arrays
        avgDistancesEdited = map(lambda item: item * 100, objectDistancesPublish)
        avgAnglesEdited = map(lambda item: item / 10, objectAnglesPublish)

        # print the data we're publishing
        print "==============="
        print "Avg Distances: ",
        print str(["{0:3.1f}".format(item) for item in avgDistancesEdited]).replace("'", "")
        print "Avg Angles   : ",
        print str(["{0:3.1f}".format(item) for item in avgAnglesEdited]).replace("'", "")

        print "Object Found : ",
        print objectCounter

        # for index, item in enumerate(avgDistancesEdited) {
        # if item
        # }

        # define the type of message to send
        msgToSend = Float32MultiArray()
        objMsg = UInt32()

        # publish the angles
        msgToSend.data = avgAnglesEdited
        pubAngle.publish(msgToSend)

        # publish the distances
        msgToSend.data = avgDistancesEdited
        pubDistance.publish(msgToSend)

        # publish the number of objects
        objMsg.data = objectCounter
        numObjects.publish(objMsg)

        # restart the counter to 0
        publishCounter = 0
    # endIf


# endDef

# define the subscriber
def random_subscriber():
    rospy.init_node('guesta_lidar', anonymous=True)
    rospy.Subscriber('/laser_birdcage_r2000/scan', LaserScan, callback)
    rospy.spin()

# endDef

if __name__ == '__main__':
    random_subscriber()
# endIf

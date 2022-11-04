from perceptionModule import passivePerception
import rospy
from perceptionConfig import *


### STARTUP ###
# initialize functions, constants
objectFound=False
trashFound=False
goDrive=True
stop=False


rospy.init_node('robot_master')
rospy.sleep(1.0)
#TODO: set robot, camera, arm to home position

#TODO: kick off image saver code or else passive perception won't run


while not stop:
    ''' PASSIVE PERCEPTION and Patrol Loop '''
    # engage passive perception module and start/continue the patrol loop
    people=[]
    trash=[]
    maybes=[]
    while not objectFound:
        pointClouds, labels = passivePerception()
        if pointClouds is not None:
            objectFound=True
            for i, lab in enumerate(labels):
                if lab in PEOPLE_CLASSES:
                    people.append(pointClouds[i])
                elif lab in TRASH_CLASSES:
                    trash.append(pointClouds[i])
                elif lab in MAYBE_TRASH_CLASSES:
                    maybes.append(pointClouds[i])
                # we don't care about specifically keeping track of the obstacle classes since we're
                # using the obstacle map from ROS in navigation

            print(labels)
            input('enter....')
            # TODO: filter for humans vs other objects
            #TODO: what will be classified as an obstacle object vs a target object?


        #TODO: set patrol to start

        #TODO: incorporate obstacle avoidance






    ### NAVIGATION ###
    #while there is no object we want to interact with, drive the robot until we find an object
    while goDrive:
        execfile("Navigation.py")
        #drive around




    ### MANIPULATION ###

    # if object is approved trash item (meets criteria),
    # set up to pick it up and grab+dispose the item

    while trashFound:
        execfile("Manipulation.py")


    ### MONITORING ###

    #is our job done? or do we need to clean more?


### END ###
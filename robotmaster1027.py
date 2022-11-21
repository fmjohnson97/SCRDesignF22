from perceptionModule import passivePerception
import rospy
from perceptionConfig import *
from navigationModule import navToPointsFound, continueNavCircuit


### STARTUP ###
# initialize functions, constants
objectFound=False
trashFound=False
goDrive=True
stop=False


rospy.init_node('robot_master')
rospy.sleep(1.0)
#TODO: set robot, camera, arm to home position

# global_trash=[]
global_trash_labels=[]
# global_maybes=[]
global_maybes_labels=[]
# global_obstacles=[]
global_obstacles_labels=[]

while not stop:
    ''' PASSIVE PERCEPTION and Navigation Patrol Loop '''
    # engage passive perception module and start/continue the patrol loop
    while not objectFound:
        people = []
        trash = []
        trash_labels = []
        maybes = []
        maybes_labels = []
        obstacles = []
        obstacle_labels = []
        pointClouds, labels = passivePerception()
        if pointClouds is not None: #TODO: have checks for if people or maybes have objects in them (objectFound only triggers on trash found)
            for i, lab in enumerate(labels):
                if lab in PEOPLE_CLASSES:
                    people.append(pointClouds[i])
                elif lab in TRASH_CLASSES:
                    objectFound = True
                    trash.append(pointClouds[i])
                    trash_labels.append(lab)
                elif lab in MAYBE_TRASH_CLASSES:
                    maybes.append(pointClouds[i])
                    maybes_labels.append(lab)
                else:
                    obstacles.append(pointClouds[i])
                    obstacle_labels.append(lab)
    #     else:
    #         # set patrol to start
    #         continueNavCircuit()

    # print(labels)
    # navToPointsFound(people, trash, maybes)

    #reset variables and keep track of what we've found in the space
    #these are old point clouds so it might not be worthwhile to save them since we'll never be in that
    #exact position again, but maybe it'll be useful for debugging??? idk
    objectFound=False
    # global_trash.extend(trash)
    global_trash_labels.extend(trash_labels)
    # global_maybes.extend(maybes)
    global_maybes_labels.extend(maybes_labels)
    # global_obstacles.extend(obstacles)
    global_obstacles_labels.extend(obstacle_labels)












    ### MANIPULATION ###

    # if object is approved trash item (meets criteria),
    # set up to pick it up and grab+dispose the item

    while trashFound:
        execfile("Manipulation.py")


    ### MONITORING ###

    #is our job done? or do we need to clean more?


### END ###
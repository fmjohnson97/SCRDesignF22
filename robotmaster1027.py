### STARTUP ###
# initialize functions, constants
objectFound=0;
trashFound=0;
goDrive=1;
# set robot, camera, arm to home position




### PASSIVE PERCEPTION ###
# engage passive perception module
execfile("PassivePerception.py")


### ACTIVE PERCEPTION ###

# if passive perception finds an object, 
# engage Active Perception module
while objectFound:
    execfile("ActivePerception.py")
    #this module identifies the object and determines
    #whether we should pick it up
    #the object either becomes an obstacle or a target


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
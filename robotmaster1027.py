from perceptionModule import passivePerception

### STARTUP ###
# initialize functions, constants
objectFound=False
trashFound=False
goDrive=True
stop=False

#TODO: set robot, camera, arm to home position

#TODO: kick off image saver code or else passive perception won't run


while not stop:
    ''' PASSIVE PERCEPTION and Patrol Loop '''
    # engage passive perception module and start/continue the patrol loop
    while not objectFound:
        pointClouds, labels = passivePerception()
        if pointClouds is not None:
            objectFound=True
            # TODO: filter for humans vs other objects

        #TODO: set patrol to start

        #TODO: incorporate obstacle avoidance



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
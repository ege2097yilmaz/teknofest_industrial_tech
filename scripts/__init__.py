#!/usr/bin/env python3.8

from local_planner import planner 
import rospy, logging

if __name__ == "__main__":
    rospy.init_node('local_planner', anonymous=True)

    #activitng functions for ros debugger
    logger = logging.getLogger("rosout") #setting output broker of debug 
    logger.setLevel(logging.DEBUG)
    level = logger.getEffectiveLevel()

    rate = rospy.Rate(5) #hz
    object = planner()
    rospy.spin()
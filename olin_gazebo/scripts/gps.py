#!/usr/bin/python
import rospy

import sys
import rospy
from gazebo_msgs.srv import GetModelState

def model_position_client():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #print get_model_state({'model_name':'olin','relative_entity_name':'ground_plane'})
        #print get_model_state({'model_name':'olin','relative_entity_name':'ground_plane'})
        return get_model_state('olin','')
    except rospy.ServiceException, e:
        print "Service call failed: {}".format(e)

if __name__ == "__main__":
    print "{}".format(model_position_client())

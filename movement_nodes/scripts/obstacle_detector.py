#!/usr/bin/env python3

import rospy

from movement_nodes.msg import Coordinates

#Variables
obstacles_x = []
obstacles_y = []
msg = Coordinates()

rospy.init_node('obstacle_detector')
pub = rospy.Publisher('obstacle_coords', Coordinates, queue_size = 1)
rate = rospy.Rate(10)

while (not rospy.is_shutdown()):
    obstacles_x = []
    obstacles_y = []
    i = 0
    while i <= 4.5:
        j = 0
        while j <= 4.5:
            if(not (i == 0 and j == 0)):
                obstacles_x.append(i)
                obstacles_y.append(j)
            j += 1.5
        i += 1.5

    #obstacles_x = [0, 0, 0.75, 2, 2, 1, 1, 2, 2, 3.5, 3.5, -1]
    #obstacles_y = [2, -2, 0, 1, -1, -3, 3, -3, 3, 2, -2, 0]

    msg.x_coords = obstacles_x
    msg.y_coords = obstacles_y
    pub.publish(msg)
    rate.sleep()

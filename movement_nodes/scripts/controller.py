#!/usr/bin/env python3
from numpy.core.defchararray import array
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 
from movement_nodes.msg import Coordinates

#Variables
global target_path
global new_path # bool
new_path = False
stop_everything = False
target_path = [[0.00001, 0.00001]]
counter = 0
delta_time = 0.1

prop_constant = 1.85
integral_constant = 0.5
diff_constant = 0.5

angular_prop_constant = 10

angular_error = 0

error = 0
prev_error = 0

integral = 0
slope = 0

integral_saturating = False
abs_error_increasing = False

x_multiplier = 0
y_multiplier = 0

#Classes
class Robot:

    def __init__(self, linear_x, linear_y, angular_z, x_coord, y_coord, z_orient):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.z_orient = z_orient
        self.angular_z = angular_z

#Methods
def publishVelocities(velocity, angular_velocity, x_multiplier, y_multiplier):

    if(abs(velocity) > 2 ** 0.5):
        velocity = 2 ** 0.5
        integral_saturating = True
    else:
        integral_saturating = False

    robot.linear_x = velocity * x_multiplier * (target_path[counter][0] - robot.x_coord) / abs(target_path[counter][0] - robot.x_coord)
    robot.linear_y = velocity * y_multiplier * (target_path[counter][1] - robot.y_coord) / abs(target_path[counter][1] - robot.y_coord)

    #print("x_multiplier: " + str(x_multiplier) + " y_multiplier: " + str(y_multiplier))

    if(abs(angular_velocity) > 2):
        robot.angular_z = 2 * angular_velocity / abs(angular_velocity) 
    else:
        robot.angular_z = angular_velocity
    
    if(stop_everything):
        robot.linear_x = 0
        robot.linear_y = 0
        robot.angular_z = 0
    
    pub_msg.linear.x = robot.linear_x
    pub_msg.linear.y = robot.linear_y
    pub_msg.angular.z = robot.angular_z

    pub.publish(pub_msg)

    return integral_saturating

def odomCoordinates(msg):
    robot.x_coord = msg.pose.pose.position.x
    robot.y_coord = msg.pose.pose.position.y
    robot.z_orient = msg.pose.pose.orientation.z

def plannerCoordinates(msg):
    global target_path
    global new_path

    array = []
    i = 0
    while(i < len(msg.x_coords)):
        array.append([msg.x_coords[i], msg.y_coords[i]])
        i = i + 1
    
    if(array != target_path):
        new_path = True
        print("Path changed.")

    target_path.clear()
    target_path = array
    
def checkIfNodeReached(error, counter):
    if(error < 0.15):
        if(not (round(target_path[counter][0], 2) == 0 and round(target_path[counter][1], 2) == 0)):
            print("Node reached. node_x = " + str(round(target_path[counter][0], 2)) + " node_y = " + str(round(target_path[counter][1], 2))+ " robot_x = " + str(round(robot.x_coord, 2)) + " robot_y = " + str(round(robot.y_coord, 2)))
        return True
    else:
        return False

#Initializing stuff
robot = Robot(0, 0, 0, 0, 0, 0)

rospy.init_node('not_actual_controller')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
pub_msg = Twist()
sub = rospy.Subscriber('odom', Odometry, odomCoordinates)
planner_sub = rospy.Subscriber('target_path', Coordinates, plannerCoordinates)
rate = rospy.Rate(1/delta_time)

#Main loop
while not rospy.is_shutdown():
    if(new_path):
        if(len(target_path) > 1):
            counter = 1
        else:
            counter = 0
        stop_everything = False
        new_path = False

    x_multiplier = abs(target_path[counter][0] - robot.x_coord) / ((target_path[counter][0] - robot.x_coord) ** 2 + (target_path[counter][1] - robot.y_coord) ** 2) ** 0.5
    y_multiplier = abs(target_path[counter][1] - robot.y_coord) / ((target_path[counter][0] - robot.x_coord) ** 2 + (target_path[counter][1] - robot.y_coord) ** 2) ** 0.5

    prev_error = error
    error = ((target_path[counter][0] - robot.x_coord) ** 2 + (target_path[counter][1] - robot.y_coord) ** 2) ** 0.5

    if(not (abs_error_increasing and integral_saturating)):
        integral = integral + error
        
    #print("integral: " + str(integral))

    slope = (error - prev_error) / delta_time

    if(slope > 10):
        stop_everything = True

    velocity = prop_constant * error + integral_constant * integral + diff_constant * slope

    angular_error = 0 - robot.z_orient
    angular_velocity = angular_error * angular_prop_constant

    if(error * integral > 0):
        abs_error_increasing = True
        #print("integral is making things worse")
    else:
        abs_error_increasing = False
    
    if(checkIfNodeReached(error, counter) and counter < len(target_path)):
        counter += 1
        integral = integral/1.5
    if(counter == len(target_path) - 1):
        integral = 0
    if(counter == len(target_path)):
        integral = 0
        counter -= 1
        stop_everything = True

    integral_saturating = publishVelocities(velocity, angular_velocity, x_multiplier, y_multiplier)

    rate.sleep()

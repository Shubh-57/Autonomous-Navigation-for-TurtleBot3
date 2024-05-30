#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# maximum linear velocity = 0.22 m/s
# maximum angular velocity = 2.84 rad/s

x_goal = 0.0
y_goal = 0.0
phi_goal = 0.0
temp_angle = 0.0

linear_error = 0.0
linear_error_threshold = 0.01
linear_prev_error = None
linear_sum_error = 0.0

angular_error = 0.0
angular_error_threshold = 0.01
angular_prev_error = None
angular_sum_error = 0.0

linear_kp = 1.0
linear_ki = 0.0
linear_kd = 0.1

angular_kp1 = 0.3
angular_ki1 = 0.0
angular_kd1 = 0.1

angular_kp2 = 1.0
angular_ki2 = 0.0
angular_kd2 = 0.1

decay_k = 2

reached_pose = False
reached_orientation = False

def linear_speed_limit(speed):
    if speed > 0.22:
        return 0.22
    
    return speed

def angular_speed_limit(speed):
    if speed > 2.84:
        return 2.84
    
    return speed

def get_input():
    global x_goal, y_goal, phi_goal
    x_goal = float(input("Enter x-coordinate:\t"))
    y_goal = float(input("Enter y-coordinate:\t"))
    phi_goal = float(input("Enter phi-angle:\t"))

def get_linear_error(position):
    global linear_error
    linear_error = math.sqrt((position.x - x_goal)**2 + (position.y - y_goal)**2)

def get_angular_error(orientation, ref_angle):
    global angular_error
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    orientation_euler = euler_from_quaternion(orientation_list)
    _, _, yaw = orientation_euler                                       # [-pi , pi]

    angular_error = ref_angle - yaw
    if angular_error > math.pi:
        angular_error -= 2 * math.pi
    elif angular_error < -math.pi:
        angular_error += 2 * math.pi

def get_temp_angle(position):
    global temp_angle
    temp_angle = math.atan2(y_goal - position.y, x_goal - position.x)   # [-pi , pi]

def stop():
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)

def reinitialize():
    global linear_error, linear_prev_error, linear_sum_error, angular_error, angular_prev_error, angular_sum_error

    linear_error = 0.0
    linear_prev_error = None
    linear_sum_error = 0.0

    angular_error = 0.0
    angular_prev_error = None
    angular_sum_error = 0.0

def angular_pid_control(orientation, ref_angle):
    global angular_error, angular_prev_error, angular_sum_error
    cmd = Twist()

    get_angular_error(orientation, ref_angle)
    if(abs(angular_error) < angular_error_threshold):
        stop()
        reinitialize()
        return True
    
    # To ensure the contribution of the derivative term is 0 at the start
    if angular_prev_error is None:
        angular_prev_error = angular_error
    
    angular_sum_error += angular_error
    kp_value = angular_kp2 * angular_error
    ki_value = angular_ki2 * angular_sum_error
    kd_value = angular_kd2 * (angular_error - angular_prev_error)
    angular_prev_error = angular_error

    cmd.angular.z = kp_value + ki_value + kd_value
    cmd.linear.x = 0
    pub.publish(cmd)
    
    return False

def misc_control(position, orientatoin):
    global linear_error, linear_prev_error, linear_sum_error, angular_error, angular_prev_error, angular_sum_error, temp_angle

    get_linear_error(position)
    if(linear_error < linear_error_threshold):
        stop()
        reinitialize()
        return True
    
    # To ensure the contribution of the derivative term is 0 at the start
    if linear_prev_error is None:
        linear_prev_error = linear_error
    if angular_prev_error is None:
        angular_prev_error = angular_error

    cmd = Twist()

    linear_sum_error += linear_error
    kp_value = linear_kp * linear_error
    ki_value = linear_ki * linear_sum_error
    kd_value = linear_kd * (linear_error - linear_prev_error)
    linear_prev_error = linear_error
    cmd.linear.x = (kp_value + ki_value + kd_value)

    get_temp_angle(position)
    get_angular_error(orientatoin, temp_angle)
    angular_sum_error += angular_error
    kp_value = angular_kp1 * angular_error
    ki_value = angular_ki1 * angular_sum_error
    kd_value = angular_kd1 * (angular_error - angular_prev_error)

    cmd.angular.z = (kp_value + ki_value + kd_value)
    cmd.angular.z = angular_speed_limit(cmd.angular.z)
    cmd.linear.x *= math.exp(-decay_k * abs(angular_error))
    cmd.linear.x = linear_speed_limit(cmd.linear.x)

    pub.publish(cmd)

    return False

def odom_callback(odom: Odometry):
    global temp_angle, reached_pose, reached_orientation

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation

    # ******* If already at the goal position *******
    get_linear_error(position)
    if linear_error < linear_error_threshold:
        reached_pose = True

        get_angular_error(orientation, phi_goal)
        if abs(angular_error) < angular_error_threshold:
            reached_orientation = True
    
    # Miscellenious Control (Step - 1)
    if not reached_pose:
        reached_pose = misc_control(position, orientation)

    # Angular Control - Orientation (Step - 2)
    elif reached_pose and not reached_orientation:
        reached_orientation = angular_pid_control(orientation, phi_goal)

    # Reached Goal
    else:
        reached_pose = False
        reached_orientation = False
        temp_angle = 0.0

        print("*************************** Reached desired location ***************************")
        print("Enter new location coordinates:")
        get_input()
    
if __name__ == '__main__':
    rospy.init_node("test_node")
    rospy.loginfo("Node has been started!!")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    get_input()
    sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.spin()
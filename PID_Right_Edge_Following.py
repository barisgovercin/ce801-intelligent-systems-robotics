#!/usr/bin/env python3

# Right Edge Following - PID

import rclpy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


mynode_ = None
pub_ = None
regions_ = {
    'left': 0,
    'right': 0,
    'fLeft': 0,
    'fRight': 0,
    'front1': 0,
    'front2': 0,
}
twstmsg_ = None
count = 0


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_, count

    print("Data Points: ", len(msg.ranges))

    regions_ = {
        # LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
        'front': find_nearest(msg.ranges[110:130]),
        'right': find_nearest(msg.ranges[21:80]),
        'left': find_nearest(msg.ranges[219:221])
    }

    twstmsg_ = movement()


def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


#PID Controller
kp = 0.88                            #Propotional Gain
ki = 0.000                            #Integral Gain (If there are small but consistent errors increase it VERY SLOWLY!)
kd = 0.33                            #Derivative Gain (If robot shake too much increase it VERY SLOWLY!)
error = 0.0                         #Current Error
error_i = 0.0                       #Integral Error
error_d = 0.0                       #Derivative Error
error_previous = 0.0                #Previous Error
desired_distance = 0.4              #Distance from the right wall (0.4m)


def movement():
    global regions_, mynode_
    global kp, ki, kd, error, error_i, error_d, error_previous, desired_distance
    regions = regions_

    current_distance = regions['right']     #Calculates the distance from the wall
    error = desired_distance - current_distance     #Calculates the error (desired - current)

    error_i += error    #Calculates the sum of all past error
    error_d = error - error_previous #Calculates yhe change in error
    pid_output = (kp * error) + (ki * error_i) + (kd * error_d)     #Angular velocity (PID)
    error_previous = error  #Save the current error for next iteration

    msg = Twist()

    # Is there is an obstacle stop the robot
    stop_distance = 0.2

    # Distance measured by the LiDAR in front of the robot
    front_dist = regions['front']
    print(f"DEBUG -> front_dist: {front_dist}, stop_distance: {stop_distance}")

    # Stop the robot if an obstacle is too close
    if front_dist < stop_distance:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print("Obstacle in the way. Stop!")
    else:
        # Move forward when the path is clear
        msg.linear.x = 0.3
        msg.angular.z = pid_output
        print(f"PID Output: {pid_output}, Current Distance: {current_distance}, Error: {error}")
    return msg


# used to stop the rosbot
def stop():
    global mynode_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    return (msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle interrupts
    try:
        rclpy.spin(mynode_)
    except Exception as e:
        print(e)
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()


if __name__ == '__main__':
    main()
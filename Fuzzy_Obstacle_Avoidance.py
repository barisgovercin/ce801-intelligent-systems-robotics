#!/usr/bin/env python3

# Obstacle Avoidance Only FLC

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'front': 0,
    'right': 0,
    'left': 0,
}
twstmsg_ = None


# Fuzzy Front distance functions
def near_front(distance):       # If obstacle is close it's high
    if distance <= 0.30:
        return 1.0
    elif 0.30 < distance < 0.60:
        return (0.60 - distance) / (0.60 - 0.30)
    else:
        return 0.0


def far_front(distance):        # If path is clear it's high
    if distance <= 0.50:
        return 0.0
    elif 0.50 < distance < 1.0:
        return (distance - 0.50) / (1.0 - 0.50)
    else:
        return 1.0


# Fuzzy side distance using the same style as right wall sets
def close_side(distance):       # High when robot is close to a side obstacle
    if distance <= 0.25:
        return 1.0
    elif 0.25 < distance < 0.45:
        return (0.45 - distance) / (0.45 - 0.25)
    else:
        return 0.0


def open_side(distance):        # High when side is free (no close obstacle)
    return 1.0 - close_side(distance)


def defuzzify(strength, centroids):

    #strength: list of firing strengths for each rule 0 to 1
    #centroids: list of centroid values for rules
    #returning: crisp output = weighted average = sum of all(firing strength * centroid) / (sum of all firing strengths)

    if len(strength) != len(centroids):
        return 0.0

    numerator = 0.0
    denominator = 0.0

    for strength, centroid in zip(strength, centroids):
        numerator += strength * centroid        # sum of all (strength * centroid)
        denominator += strength                 # (sum of all strengths)

    # Prevent the division by zero
    if denominator == 0.0:
        return 0.0

    return numerator / denominator


# Obstacle Avoidance FLC only (no right edge follow, no blender)
def decision_obstacle_avoidance(front_distance, right_distance, left_distance):

    # Front fuzzy memberships (how close or clear the front is)
    front_near = near_front(front_distance)
    front_far  = far_front(front_distance)

    # reuse side fuzzy shapes for both right and left
    right_close = close_side(right_distance)
    left_close  = close_side(left_distance)

    right_open = open_side(right_distance)
    left_open  = open_side(left_distance)

    # Corner: front and both sides are blocked → strong escape to the left
    both_sides_close = min(left_close, right_close)
    rule_corner_escape = min(front_near, both_sides_close)

    # Rule 1: front near & left open → turn left
    rule_turn_left = min(front_near, left_open)

    # Rule 2: front near & right open → turn right
    # In this controller we never want a right turn,
    # so this rule will still produce a small left turn.
    rule_turn_right = min(front_near, right_open)

    # Rule 3: front far → go straight
    rule_forward = front_far

    strengths = [
        rule_corner_escape,
        rule_turn_left,
        rule_turn_right,
        rule_forward
    ]

    # Angular centroids for OA: [corner left, normal left, soft left (was right), straight]
    angular_centroids = [
        1.0,    # strong left turn to escape a corner
        0.6,    # normal left
        0.3,    # soft left when right side is open (no right turns)
        0.0     # go straight
    ]

    # Linear centroids for OA: [corner almost stop, turn slow, turn slow, straight faster]
    linear_centroids = [
        0.0,    # almost on the spot when escaping a corner
        0.10,   # slow when turning left
        0.10,   # slow when using the "right-open" rule (still left turn)
        0.30    # faster when going straight
    ]

    angular_z = defuzzify(strengths, angular_centroids)
    linear_x = defuzzify(strengths, linear_centroids)

    # Extra safety: never allow a right turn (negative angular velocity)
    if angular_z < 0.0:
        angular_z = 0.0

    print(f"OA ONLY -> lin_x: {linear_x:.3f}, ang_z: {angular_z:.3f}, strengths: {strengths}")

    return linear_x, angular_z


# Main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if twstmsg_ is not None:
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    print("Data Points: ", len(msg.ranges))

    # LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
    regions_ = {
        'front': find_nearest(msg.ranges[101:132]),
        'right': find_nearest(msg.ranges[0:81]),
        'left':  find_nearest(msg.ranges[132:242])
    }

    front_distance = regions_['front']
    right_distance = regions_['right']
    left_distance  = regions_['left']

    # Use the minimum of front and left to treat left-front obstacles as front
    effective_front = min(front_distance, left_distance)

    print(f"front: {front_distance:.3f}, right: {right_distance:.3f}, left: {left_distance:.3f}")

    # Obstacle avoidance fuzzy controller decides everything
    linear_x, angular_z = decision_obstacle_avoidance(effective_front, right_distance, left_distance)

    twstmsg_ = movement(angular_z=angular_z, linear_x=linear_x)


def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


def movement(angular_z, linear_x):
    msg = Twist()

    # Stop the robot if there is an obstacle
    stop_distance = 0.22
    front_dist = min(regions_['front'], regions_['left'])
    print(f"Front Distance: {front_dist}, Stop Distance: {stop_distance}")

    if front_dist <= stop_distance:
        # Stop forward motion but keep turning in place to escape
        msg.linear.x = 0.0
        msg.angular.z = 0.6
        print(f"Obstacle very close so stop forward, turning only. (angular.z={msg.angular.z})")
    else:
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        print(f"Linear Speed: {msg.linear.x}, Angular Speed: {msg.angular.z}")

    return msg


# Used to stop the rosbot
def stop():
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    return msg


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('oa_only_fuzzy')

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

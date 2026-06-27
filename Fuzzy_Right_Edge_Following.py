#!/usr/bin/env python3

# Right Edge Following - Fuzzy

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


# Fuzzy Right distance functions      High means it has the authority
def close_right(distance):          # High when robot 0.3 meter or closer to the wall
    if distance <= 0.3:
        return 1.0
    elif 0.3 < distance < 0.6:
        return (0.6-distance) / (0.6 - 0.3)
    else:
        return 0.0


def okay_right(distance):
    if distance <= 0.3 or distance >= 0.9:
        return 0.0
    elif 0.3 < distance < 0.6:      # Going to the perfect spot
        return (distance - 0.3) / (0.6 - 0.3)
    elif 0.6 <= distance < 0.9:     # Going far
        return (0.9 - distance) / (0.9 - 0.6)
    else:
        return 0.0


def far_right(distance):        # High when the robot is away from the right wall
    if distance <= 0.7:
        return 0.0
    elif 0.7 < distance < 1.2:
        return (distance - 0.7) / (1.2 - 0.7)
    else:
        return 1.0


# Fuzzy Front distance functions
def near_front(distance):       # If obstacle is close it's high
    if distance <= 0.3:
        return 1.0
    elif 0.3 < distance < 0.6:
        return (0.8 - distance) / (0.8 - 0.3)
    else:
        return 0.0


def far_front(distance):        # If path is clear it's high
    if distance <= 0.5:
        return 0.0
    elif 0.5 < distance < 1.0:
        return (distance - 0.5) / (1.0 - 0.5)
    else:
        return 1.0


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


def decision_fuzzy(front_distance, right_distance):
        right_close = close_right(right_distance)
        right_okay = okay_right(right_distance)
        right_far = far_right(right_distance)

        front_near = near_front(front_distance)
        front_far = far_front(front_distance)

        # Rule strengths
        rule_1 = front_near                         # front near => slow and strong left
        rule_2 = min(front_far, right_close)        # front far AND right close => small left
        rule_3 = min(front_near, right_far)         # front near AND right far => small right
        rule_4 = min(front_far, right_okay)         # front far AND right okay => no turn
        rule_5 = min(front_far, right_far)          # front far AND right far => strong right (search right wall)

        # List of strengths for defuzzification
        strengths = [rule_1, rule_2, rule_3, rule_4, rule_5]

        total_strength = sum(strengths)

        if total_strength == 0.0: # Default: go forward, no turn
            # Right wall search
            linear_x = 0.25
            angular_z = -0.3
            print("No fuzzy rule active -> using default motion (forward, slight right)")
            print(f"Fuzzy -> lin_x: {linear_x:.3f}, ang_z: {angular_z:.3f}, strengths: {strengths}")
            return linear_x, angular_z

        # Centroids for angular speed
        # Rule1: strongly left, Rule2: softly left, Rule3: softly right, Rule4: straight, Rule5: strong right
        angular_centroids = [0.8, 0.5, -0.5, 0.0, -0.8]

        # Centroids for linear speed (velocity)
        # Rule1: very slow, Rule2 and 3: medium, Rule4: fast, Rule5: medium-fast
        linear_centroids = [0.08, 0.18, 0.18, 0.3, 0.25]

        # Defuzzification: weighted average of all tye rules
        angular_z = defuzzify(strengths, angular_centroids)
        linear_x = defuzzify(strengths, linear_centroids)

        # Print for seeing the output values
        print(f"Fuzzy -> lin_x: {linear_x:.3f}, ang_z: {angular_z:.3f}, strengths: {strengths}")

        return linear_x, angular_z


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if twstmsg_ is not None:
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    print("Data Points: ", len(msg.ranges))

    regions_ = {
        'front': find_nearest(msg.ranges[101:132]),
        'right': find_nearest(msg.ranges[0:81]),
        'left':  find_nearest(msg.ranges[132:242])
    }

    front_distance = regions_['front']
    right_distance = regions_['right']

    # Debug: print the distance values
    print(f"front_distance: {front_distance}, right_distance: {right_distance}")

    # Fuzzy decision function
    linear_x, angular_z = decision_fuzzy(front_distance, right_distance)

    twstmsg_ = movement(angular_z=angular_z, linear_x=linear_x)




def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


def movement(angular_z, linear_x):

    msg = Twist()

    # Stop the robot if there is an obstacle
    stop_distance = 0.2
    front_dist = regions_['front']
    print(f"Front Distance: {front_dist}, Stop Distance: {stop_distance}")

    if front_dist < stop_distance:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print("Obstacle in the way. Stop!")
    else:
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        print(f"Linear Speed: {msg.linear.x}, Angular Speed: {msg.angular.z}")

    return msg


# Used to stop the rosbot
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

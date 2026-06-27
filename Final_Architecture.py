#!/usr/bin/env python3

# Obstacle Avoidance - Fuzzy Combined With Right Edge Following - Fuzzy

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
def close_right(distance):          # High when robot 0.25m or closer to the wall
    if distance <= 0.25:
        return 1.0
    elif 0.25 < distance < 0.45:
        return (0.45 - distance) / (0.45 - 0.25)
    else:
        return 0.0


def okay_right(distance):
    if distance <= 0.25 or distance >= 0.65:
        return 0.0
    elif 0.25 < distance < 0.45:      # Going toward ideal right-wall distance
        return (distance - 0.25) / (0.45 - 0.25)
    elif 0.45 <= distance < 0.65:     # Moving slightly far from ideal distance
        return (0.65 - distance) / (0.65 - 0.45)
    else:
        return 0.0


def far_right(distance):        # High when the robot is far from the right wall (>= 0.5m)
    if distance <= 0.5:
        return 0.0
    elif 0.5 < distance < 1.0:
        return (distance - 0.5) / (1.0 - 0.5)
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


def blender_fuzzy(front_distance):      # Fuzzy blender for mixing obstacle avoidance and right edge follow

    # Mode fuzzy memberships (based on front distance)
    mode_near = near_front(front_distance)   # high when front is close
    mode_far  = far_front(front_distance)    # high when front is clear

    # Middle mode: not clearly near and not clearly far
    mode_mid = 1.0 - max(mode_near, mode_far)
    if mode_mid < 0.0:
        mode_mid = 0.0

    # Debug print for modes
    print(f"near: {mode_near:.3f}, mid: {mode_mid:.3f}, far: {mode_far:.3f}")

    # If front is near: obstacle avoidance should stronger
    # If front is mid: both share control
    # If front is far: right edge follow should stronger

    # Weight for obstacle avoidance (OA FLC)
    weight_obstacle_avoidance = (
        1.0 * mode_near +   # strong near
        0.5 * mode_mid +    # medium in the middle
        0.2 * mode_far      # small when far
    )

    # Weight for right edge follow (REF FLC)
    weight_right_edge_follow = (
        0.2 * mode_near +   # small when near obstacle
        0.5 * mode_mid +    # medium in the middle
        1.0 * mode_far      # strong when far
    )

    print(
        f"Obstacle Avoidance : {weight_obstacle_avoidance:.3f}, "
        f"Right Edge Follow: {weight_right_edge_follow:.3f}"
    )

    return weight_obstacle_avoidance, weight_right_edge_follow


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


def decision_right_edge_follow(front_distance, right_distance):
        right_close = close_right(right_distance)
        right_okay = okay_right(right_distance)
        right_far = far_right(right_distance)

        front_near = near_front(front_distance)
        front_far = far_front(front_distance)

        # Rule strengths
        rule_1 = front_near
        rule_2 = min(front_far, right_close)
        rule_3 = min(front_near, right_far)
        rule_4 = min(front_far, right_okay)

        # List of strengths for defuzzification
        strengths = [rule_1, rule_2, rule_3, rule_4]

        total_strength = sum(strengths)

        if total_strength == 0.0: # Default: go forward, no turn
            linear_x = 0.2
            angular_z = 0.0
            return linear_x, angular_z

        # Centroids for angular speed
        # Rule1: strongly left, Rule2: softly left, Rule3: softly right, Rule4: straight
        angular_centroids = [0.8, 0.5, -0.5, 0.0]

        # Centroids for linear speed (velocity)
        # Rule1: very slow, Rule2 and 3: medium, Rule4: fast
        linear_centroids = [0.08, 0.18, 0.18, 0.3]

        # Defuzzification: weighted average of all tye rules
        angular_z = defuzzify(strengths, angular_centroids)
        linear_x = defuzzify(strengths, linear_centroids)

        # Print for seeing the output values
        print(f"Fuzzy -> lin_x: {linear_x:.3f}, ang_z: {angular_z:.3f}, strengths: {strengths}")

        return linear_x, angular_z


# What to do when there is an obstacle
def decision_obstacle_avoidance(front_distance, right_distance, left_distance):
    # Front fuzzy memberships (same style as REF)
    front_near = near_front(front_distance)
    front_far_val = far_front(front_distance)

    # Use right fuzzy set also for left side
    right_close = close_right(right_distance)
    left_close = close_right(left_distance)

    right_open = 1.0 - right_close
    left_open = 1.0 - left_close

    # Extra: corner situation (front and both sides close)
    both_sides_close = min(left_close, right_close)
    rule_escape_left = min(front_near, both_sides_close)

    # Rule 1: Front is near and left side is more open -> normal left turn
    rule_turn_left = min(front_near, left_open)

    # Rule 2: Front is near and right side is more open -> turn right
    rule_turn_right = min(front_near, right_open)

    # Rule 3: Front is far -> go forward
    rule_forward = front_far_val

    # Put corner-escape rule first so we can give it a stronger action
    strengths = [rule_escape_left, rule_turn_left, rule_turn_right, rule_forward]

    total_strength = sum(strengths)
    if total_strength == 0.0:
        # No strong opinion so no rule
        linear_obstacle_avoidance = 0.0
        angular_obstacle_avoidance = 0.0
        print("No active rule.")
        return linear_obstacle_avoidance, angular_obstacle_avoidance

    # Angular centroids for obstacle avoidance:
    # Corner-escape strong left, normal left, right, straight
    angular_centroids_obstacle_avoidance = [1.0, 0.8, -0.8, 0.0]

    # Linear centroids for obstacle avoidance:
    # Corner-escape: almost on the spot, others same as before
    linear_centroids_obstacle_avoidance = [0.0, 0.08, 0.08, 0.25]

    angular_obstacle_avoidance = defuzzify(strengths, angular_centroids_obstacle_avoidance)
    linear_obstacle_avoidance = defuzzify(strengths, linear_centroids_obstacle_avoidance)

    # Safety: obstacle avoidance is not allowed to turn right (no negative angular)
    if angular_obstacle_avoidance < 0.0:
        angular_obstacle_avoidance = 0.0

    print(
        f"Obstacle Avoidance -> lin_x: {linear_obstacle_avoidance:.3f}, "
        f"ang_z: {angular_obstacle_avoidance:.3f}, strengths: {strengths}"
    )

    return linear_obstacle_avoidance, angular_obstacle_avoidance


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
    left_distance  = regions_['left']

    effective_front = min(front_distance, left_distance)    # left front crash dealing

    # 1) specialist FLCs
    lin_ref, ang_ref = decision_right_edge_follow(front_distance, right_distance)
    lin_oa,  ang_oa  = decision_obstacle_avoidance(effective_front, right_distance, left_distance)

    # How near is the obstacle in front? (0..1 fuzzy value)
    front_near_val = near_front(effective_front)

    if front_near_val > 0.3:
        # Obstacle is close -> only obstacle avoidance controls the robot
        lin_final = lin_oa
        # extra safety: never turn right here
        ang_final = max(ang_oa, 0.0)
        print(f"Obstacle Avoidance lin: {lin_final:.3f}, ang: {ang_final:.3f}, front_near={front_near_val:.3f}")
    else:
        # Front is not too close -> normal blended control
        w_oa, w_ref = blender_fuzzy(effective_front)

        den = w_oa + w_ref
        if den > 0.0:
            lin_final = (lin_oa * w_oa + lin_ref * w_ref) / den
            ang_final = (ang_oa * w_oa + ang_ref * w_ref) / den
        else:
            lin_final = 0.0
            ang_final = 0.0
        if front_near_val > 0.0 and ang_final < 0.0:
            ang_final = 0.0

        print(f"Blended lin: {lin_final:.3f}, ang: {ang_final:.3f}, "
              f"w_oa={w_oa:.3f}, w_ref={w_ref:.3f}, front_near={front_near_val:.3f}")

    twstmsg_ = movement(angular_z=ang_final, linear_x=lin_final)






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
        # Stop forward motion but keep turning
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
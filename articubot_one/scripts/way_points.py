#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time
import copy
import math

"""
Basic navigation demo to go to pose.
"""
time.sleep(5)

# Quaternion class
class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0



# A function that converts euler angles to quaternions
def euler_to_quaternion(roll=0, pitch=0, yaw=0):
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q = Quaternion()
    q.w = cy * cr * cp + sy * sr * sp
    q.x = cy * sr * cp - sy * cr * sp
    q.y = cy * cr * sp + sy * sr * cp
    q.z = sy * cr * cp - cy * sr * sp
    return q

def set_angle(pose, yaw):
    q = euler_to_quaternion(0, 0, yaw)
    pose.pose.orientation.x = q.x
    pose.pose.orientation.y = q.y
    pose.pose.orientation.z = q.z
    pose.pose.orientation.w = q.w
    return pose

def draw_rect(h, w, navigator):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0

    h, w = float(h), float(w)
    
    goal_poses = []
    for _ in range(3):
        goal_poses.append(copy.deepcopy(goal_pose))

# - Translation: [0.636, -0.419, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, -0.057, 0.998]
# - Rotation: in RPY (radian) [0.000, 0.000, -0.114]
# - Rotation: in RPY (degree) [0.000, 0.000, -6.549]

# - Translation: [6.825, -0.721, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, 0.045, 0.999]
# - Rotation: in RPY (radian) [0.000, -0.000, 0.089]
# - Rotation: in RPY (degree) [0.000, -0.000, 5.126]

# - Translation: [0.505, -0.359, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, 0.053, 0.999]
# - Rotation: in RPY (radian) [0.000, -0.000, 0.106]
# - Rotation: in RPY (degree) [0.000, -0.000, 6.075]

# - Translation: [0.375, 1.492, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, 0.030, 1.000]
# - Rotation: in RPY (radian) [0.000, -0.000, 0.059]
# - Rotation: in RPY (degree) [0.000, -0.000, 3.391]

# - Translation: [2.620, 1.462, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, -0.037, 0.999]
# - Rotation: in RPY (radian) [0.000, 0.000, -0.074]
# - Rotation: in RPY (degree) [0.000, 0.000, -4.238]





    goal_poses[0].pose.position.x = 6.440
    goal_poses[0].pose.position.y = 3.147
    goal_poses[0] = set_angle(goal_poses[0], -0.201)

    goal_poses[1].pose.position.x = 6.923
    goal_poses[1].pose.position.y = 1.242
    goal_poses[1] = set_angle(goal_poses[1], -0.160)

    goal_poses[2].pose.position.x = 0.874
    goal_poses[2].pose.position.y = 1.621
    goal_poses[2] = set_angle(goal_poses[2], -0.201)

    # goal_poses[3].pose.position.x = 6.893
    # goal_poses[3].pose.position.y = 1.369
    # goal_poses[3] = set_angle(goal_poses[3], 1.685)

    # goal_poses[4].pose.position.x = 1.442
    # goal_poses[4].pose.position.y = 2.095
    # goal_poses[4] = set_angle(goal_poses[4], 0.053)

    # goal_poses[5].pose.position.x = 1.341
    # goal_poses[5].pose.position.y = -0.827
    # goal_poses[5] = set_angle(goal_poses[5], 1.703)

    print("------- GOAL POSES -------")
    for pose in goal_poses:
        print(pose, end="\n\n")

    return goal_poses


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()
    print("Waiting for Nav2 to activate...")

    # Wait for navigation to fully activate, since autostarting nav2


    # navigator.waitUntilNav2Active()
    print("Nav2 activated!")

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_poses = draw_rect(6.5, 3.5, navigator)

    

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    for goal in goal_poses:
        navigator.goToPose(goal)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
            

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
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
from threading import Thread

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

"""
Basic navigation demo to go to pose.
"""
time.sleep(5)

tf_msg = TransformStamped()
odom_msg = Odometry()

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



def arcelik_path(navigator):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0
    
    goal_poses = []
    for _ in range(12):
        goal_poses.append(copy.deepcopy(goal_pose))

    goal_poses[0].pose.position.x = -2.0
    goal_poses[0].pose.position.y = -2.0
    goal_poses[0] = set_angle(goal_poses[0], 0.0)

    goal_poses[1].pose.position.x = -2.0
    goal_poses[1].pose.position.y = -2.0
    goal_poses[1] = set_angle(goal_poses[1], 0.0)

    goal_poses[2].pose.position.x = -0.2
    goal_poses[2].pose.position.y = -2.0
    goal_poses[2] = set_angle(goal_poses[2], 0.0)

    goal_poses[3].pose.position.x = -2.0
    goal_poses[3].pose.position.y = -2.0
    goal_poses[3] = set_angle(goal_poses[3], 0.0)
    #---------------------------------------------

    goal_poses[4].pose.position.x = -2.0
    goal_poses[4].pose.position.y = -4.0
    goal_poses[4] = set_angle(goal_poses[4], 0.0)

    goal_poses[5].pose.position.x = -0.2
    goal_poses[5].pose.position.y = -4.0
    goal_poses[5] = set_angle(goal_poses[5], 0.0)

    goal_poses[6].pose.position.x = -2.0
    goal_poses[6].pose.position.y = -4.0
    goal_poses[6] = set_angle(goal_poses[6], 0.0)
    #---------------------------------------------

    goal_poses[7].pose.position.x = -2.0
    goal_poses[7].pose.position.y = -6.0
    goal_poses[7] = set_angle(goal_poses[7], 0.0)

    goal_poses[8].pose.position.x = -0.2
    goal_poses[8].pose.position.y = -6.0
    goal_poses[8] = set_angle(goal_poses[8], 0.0)

    goal_poses[9].pose.position.x = -2.0
    goal_poses[9].pose.position.y = -6.0
    goal_poses[9] = set_angle(goal_poses[9], 0.0)
    #---------------------------------------------

    goal_poses[10].pose.position.x = -2.0
    goal_poses[10].pose.position.y = -8.0
    goal_poses[10] = set_angle(goal_poses[10], 0.0)

    goal_poses[11].pose.position.x = -0.2
    goal_poses[11].pose.position.y = -8.0
    goal_poses[11] = set_angle(goal_poses[11], 0.0)

    enter_cooridor_indices = [2, 5, 8, 11]

    print("------- GOAL POSES -------")
    for pose in goal_poses:
        print(pose, end="\n\n")

    return goal_poses, enter_cooridor_indices

def transform_callback(msg):
    global tf_msg
    tf_msg = msg
    print("Transform callback called")

def odometry_callback(msg):
    global odom_msg
    odom_msg = msg
    # print("Odometry callback called")

def main():
    rclpy.init()
    node = rclpy.create_node('waypoints')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin)
    executor_thread.start()

    follow_cooridor_client = node.create_client(FollowCooridor, 'follow_cooridor')
    run_follow_cooridor = FollowCooridor.Request()
    run_follow_cooridor.run = True

    stop_follow_cooridor = FollowCooridor.Request()
    stop_follow_cooridor.run = False
    
    # spin_thread = Thread(target=rclpy.spin, args=(node,))
    # spin_thread.start()

    # tf_sub = node.create_subscription(TransformStamped, '/tf', transform_callback, 10)
    odom_sub = node.create_subscription(Odometry, 'diff_cont/odom', odometry_callback, 10)

    tf_buffer = Buffer()


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
    goal_poses, enter_cooridor_indices = arcelik_path(navigator)

    

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    for goal_index in range(len(goal_poses)):
        navigator.goToPose(goal_poses[goal_index])

        i = 0
        while not navigator.isTaskComplete():
            
            

            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                # print('Estimated time of arrival: ' + '{0:.0f}'.format(
                #     Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                #     + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()


        if goal_index in enter_cooridor_indices:
                print("ENTERING COORIDOR")
                print() 
                print("Current pose y: ", feedback.current_pose.pose.position.y)
                print("Current pose x: ", feedback.current_pose.pose.position.x)
                print()

                future = follow_cooridor_client.call_async(run_follow_cooridor)

                # Spin until the response is received
                rclpy.spin_until_future_complete(node, future)
                print("Response: %r" % (future.result(),))


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

    # navigator.lifecycleShutdown()
    # node.destroy_node()
    # rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()
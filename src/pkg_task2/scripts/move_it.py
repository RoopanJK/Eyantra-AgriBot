#! /usr/bin/python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('agribot_moveit', anonymous=True)

        self._planning_group = "Robot_Arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)
        self._group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self._group.clear_pose_targets()

        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class Ur5Gripper:

    # Constructor
    def __init__(self):
        self._planning_group = "Robot_Gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Gripper init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)
        self._group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self._group.clear_pose_targets()
        rospy.loginfo(
        '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    griper = Ur5Gripper()

    while not rospy.is_shutdown():

        rospy.sleep(10)
#* first tomato
        print("execute 1")
        ur5.go_to_predefined_pose("T1_Arm_1")
        rospy.sleep(2)
        print("execute 2")
        ur5.go_to_predefined_pose("T1_Arm_2")
        rospy.sleep(2)
        print("execute 3")
        griper.go_to_predefined_pose("Grip_Close")
        rospy.sleep(2)
        print("execute 4")
        ur5.go_to_predefined_pose("T1_Arm_2")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("Home")
        rospy.sleep(2)
        griper.go_to_predefined_pose("Grip_Open")

#* second tomato
        ur5.go_to_predefined_pose("T2_Arm_1")
        rospy.sleep(2)

        griper.go_to_predefined_pose("Grip_Close")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("Home")
        rospy.sleep(2)
        griper.go_to_predefined_pose("Grip_Open")
#* third tomato
        ur5.go_to_predefined_pose("T3_Arm_1")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("T3_Arm_2")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("T3_Arm_3")
        rospy.sleep(2)
        griper.go_to_predefined_pose("Grip_Close")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("Home")

        griper.go_to_predefined_pose("Grip_Open")
        break
    del ur5


if __name__ == '__main__':
    main()
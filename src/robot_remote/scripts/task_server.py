#! /usr/bin/env python3
import rospy
import actionlib
from robot_remote.msg import RobotTaskAction, RobotTaskResult
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# from std_msgs.msg import Float32
# from moveit_commander.conversions import pose_to_list

class TaskServer(object):
    # create messages that are used to publish feedback/result
    result_ = RobotTaskResult()
    arm_goal_ = []
    gripper_goal_ = []
    # x = 0
    # y = 0

    def __init__(self, name):
        # Constructor
        # function that inizialize the RobotTaskAction class and creates 
        # a Simple Action Server from the library actionlib
        # Constructor that gets called when a new instance of this class is created
        # it basically inizialize the MoveIt! API that will be used throughout the script
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # rospy.Subscriber("/image_x", Float32, self.set_x)
        # rospy.Subscriber("/image_y", Float32, self.set_y)
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        # create a move group commander object that will be the interface with the robot joints
        self.arm_move_group_ = moveit_commander.MoveGroupCommander('Robot_arm')

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # create a move group commander object for the gripper
        self.gripper_move_group_ = moveit_commander.MoveGroupCommander('Robot_hand')

        self.scene = moveit_commander.PlanningSceneInterface()

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.action_name_ = name
        self.as_ = actionlib.SimpleActionServer(self.action_name_, RobotTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self.as_.start()

    # def set_x(self, msg):
    #     self.x = msg

    # def set_y(self, msg):
    #     self.y = msg


    def execute_cb(self, goal):

        success = True     

        # start executing the action
        # based on the goal id received, send a different goal 
        # to the robot
        if goal.task_number == 0:
            self.arm_goal_ = [0.0,0.0,0.0]
            self.gripper_goal_ = [-0.7, 0.7]
            self.arm_move_group_.go(self.arm_goal_, wait=True)
            self.gripper_move_group_.go(self.gripper_goal_, wait=True)
            self.arm_move_group_.stop()
            self.gripper_move_group_.stop()
        elif goal.task_number == 1:
            # self.arm_goal_ = geometry_msgs.msg.Pose()
            # self.arm_goal_.orientation.w = 1
            # self.arm_goal_.orientation.x = 1e-6
            # self.arm_goal_.orientation.y = 1e-6
            # self.arm_goal_.orientation.z = 1e-6
            # We can get the name of the reference frame for this robot:
            planning_frame = self.arm_move_group_.get_planning_frame()
            print("============ Planning frame: %s" % planning_frame)

            # We can also print the name of the end-effector link for this group:
            eef_link = self.arm_move_group_.get_end_effector_link()
            print("============ End effector link: %s" % eef_link)

            # We can get a list of all the groups in the robot:
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", self.robot.get_group_names())

            # Sometimes for debugging it is useful to print the entire state of the
            # robot:
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")
            x = rospy.get_param("x")
            y = rospy.get_param("y")
            print("x: ", x)
            print("y: ", y)
            waypoints = []

            wpose = self.arm_move_group_.get_current_pose().pose
            if x < 0:
                wpose.position.x = x + 0.05 # First move up (x)
            else:
                wpose.position.x = x - 0.05 # First move up (x)
            if y < 0:
                wpose.position.y = y + 0.25 # First move up (y)
            else:
                wpose.position.y = y - 0.25 # First move up (y)
            wpose.position.z = 2.35 - y # Second move forward/backwards in (z)
            if(wpose.position.z > 0.7):
                wpose.position.z = 0.6
            # waypoints.append(copy.deepcopy(wpose))

            waypoints.append(copy.deepcopy(wpose))

            # We want the Cartesian path to be interpolated at a resolution of 1 cm
            # which is why we will specify 0.01 as the eef_step in Cartesian
            # translation.  We will disable the jump threshold by setting it to 0.0,
            # ignoring the check for infeasible jumps in joint space, which is sufficient
            # for this tutorial.
            (plan, fraction) = self.arm_move_group_.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory)
            self.arm_move_group_.execute(plan, wait=True)
            self.gripper_goal_ = [-0.12, 0.12]
            self.gripper_move_group_.go(self.gripper_goal_, wait=True)
            self.gripper_move_group_.stop()
            # self.arm_goal_ = [0.0,0.0,0.0]
            # self.gripper_goal_ = [-0.7, 0.7]
            self.arm_goal_ = [0.0,0.0,0.0]
            self.gripper_goal_ = [-0.7, 0.7]
            self.arm_move_group_.go(self.arm_goal_, wait=True)
            self.gripper_move_group_.go(self.gripper_goal_, wait=True)
        elif goal.task_number == 2:
            self.arm_goal_ = [-1.57,0.0,-1.0]
            self.gripper_goal_ = [0.0, 0.0]
            self.arm_move_group_.go(self.arm_goal_, wait=True)
            self.gripper_move_group_.go(self.gripper_goal_, wait=True)
            self.arm_move_group_.stop()
            self.gripper_move_group_.stop()
        else:
            rospy.logerr('Invalid goal')
            return

        # Sends a goal to the moveit API
        # self.arm_move_group_.go(self.arm_goal_, wait=True)
        # self.gripper_move_group_.go(self.gripper_goal_, wait=True)

        # Make sure that no residual movement remains
        self.arm_move_group_.stop()
        self.gripper_move_group_.stop()

        # check that preempt has not been requested by the client
        if self.as_.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name_)
            self.as_.set_preempted()
            success = False
       
        # check if the goal request has been executed correctly
        if success:
            self.result_.success = True
            rospy.loginfo('%s: Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)        


if __name__ == '__main__':

    # Inizialize a ROS node called task_server
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()

#!/usr/bin/env python

import enum
import rospy
import moveit_commander
import geometry_msgs.msg 
from math import pi, radians
from tf import transformations as tf
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

'''
Created on 08.11.2021

@author: Andy Chien
'''

ROTATION_AXIS = 'sxyz'

class CheckScene(EventState):
	'''
	Uses dcma planner to plan or plan and move the specified joints to the target configuration.

	-- group_name       string      move group name

	># joint_config		float[]		Target configuration of the joints.
									Same order as their corresponding names in joint_names.

	#> joint_trajectory JointTrajectory  planned or executed trajectory

	<= done 						Target joint configuration has been planned.
	<= failed 				Failed to find a plan to the given joint configuration.
	'''


	def __init__(self, group_name):
		'''
		Constructor
		'''
		super(CheckScene, self).__init__(outcomes=['failed', 'done'])
		# group_name = ""
		self._check_scene_client = ProxyServiceCaller({'/check_state_validity': GetStateValidity})
		self._group_name = group_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._scene = moveit_commander.PlanningSceneInterface()
		self._robot = moveit_commander.RobotCommander()
		self._move_group.allow_looking(True)
		self._result = None

	def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
		scene = self._scene
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			attached_objects = scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0

			is_known = box_name in scene.get_known_object_names()

			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			rospy.sleep(0.1)
			seconds = rospy.get_time()

		return False

	def add_box(self, box_name, parent_frame, box_pos, box_euler, box_size, timeout=4):
		scene = self._scene
		quaternion = tf.quaternion_from_euler(radians(box_euler[0]), radians(box_euler[1]), radians(box_euler[2]), axes=ROTATION_AXIS)
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = parent_frame
		box_pose.pose.orientation.x = quaternion[0]
		box_pose.pose.orientation.y = quaternion[1]
		box_pose.pose.orientation.z = quaternion[2]
		box_pose.pose.orientation.w = quaternion[3]
		box_pose.pose.position.x = box_pos[0]
		box_pose.pose.position.y = box_pos[1]
		box_pose.pose.position.z = box_pos[2]
		scene.add_box(box_name, box_pose, size=box_size)
		return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=True, timeout=timeout)

	def execute(self, userdata):
		'''
		Execute this state
		'''
		gsv_req = GetStateValidityRequest()
		gsv_req.robot_state = self._robot.get_current_state()
		gsv_req.group_name = self._group_name
		gsv_res = self._check_scene_client.call('/check_state_validity', gsv_req)
		
		# self._scene.remove_world_object('box_1')
		# self.wait_for_state_update('box_1', box_is_known=True, box_is_attached=True, timeout=10)
		# self._scene.remove_world_object('box_2')
		# self.wait_for_state_update('box_2', box_is_known=True, timeout=10)

		if gsv_res.valid:
			return 'done'
		else:
			return 'failed'

	def on_enter(self, userdata):
		# self.add_box('box_1', 'world', [0.9, 0, 0], [0, 0, 0], [0.5, 0.5, 0.5], timeout=1)
		# self.add_box('box_2', 'world', [1.3, 0, 0], [0, 0, 0], [0.5, 0.5, 0.5], timeout=1)
		# self._move_group.attach_object('box_1', 'ee_link', ['ee_link', 'tool0', 'wrist_3_link', 'wrist_2_link', 'wrist_1_link', 'forearm_link'])
		# self.wait_for_state_update('box_1', box_is_known=True, timeout=2)
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)

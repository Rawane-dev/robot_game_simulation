#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import os
import actionlib

from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

POSE_LIBRARY_FILE = "/tmp/fetch_pose_library.yaml"

HEAD_JOINTS = ["head_pan_joint", "head_tilt_joint"]
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "upperarm_roll_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint"
]
TORSO_JOINT = ["torso_lift_joint"]
ALL_JOINTS = HEAD_JOINTS + ARM_JOINTS + TORSO_JOINT
MODEL_NAME = "fetch"

class PoseSnapshot:
    def __init__(self):
        #rospy.init_node("pose_snapshot_node", anonymous=True)

        self.joint_data = {}
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")

        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.head_pub = rospy.Publisher("/head_controller/command", JointTrajectory, queue_size=1)

        self.arm_client = actionlib.SimpleActionClient(
            "/arm_with_torso_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for arm_with_torso trajectory action server...")
        self.arm_client.wait_for_server()
        rospy.loginfo("Arm_with_torso controller is ready.")

        rospy.sleep(1)

    def joint_state_callback(self, msg):
        self.joint_data = dict(zip(msg.name, msg.position))

    def save_pose(self, name):
        if not name:
            rospy.logerr("Pose name must not be empty.")
            return

        rospy.loginfo("📸 Capturing pose as '{}'...".format(name))
        data = {}

        for joint in ALL_JOINTS:
            if joint in self.joint_data:
                data[joint] = self.joint_data[joint]
            else:
                rospy.logwarn("Joint {} not found in joint_states!".format(joint))

        state = self.get_model_state(MODEL_NAME, "world")
        if state.success:
            data["base_position"] = {
                "x": state.pose.position.x,
                "y": state.pose.position.y
            }
        else:
            rospy.logerr("Failed to get model state.")

        if os.path.exists(POSE_LIBRARY_FILE):
            with open(POSE_LIBRARY_FILE, "r") as f:
                all_poses = yaml.safe_load(f) or {}
        else:
            all_poses = {}

        all_poses[name] = data

        with open(POSE_LIBRARY_FILE, "w") as f:
            yaml.dump(all_poses, f)

        rospy.loginfo("✅ Saved pose '{}' to {}".format(name, POSE_LIBRARY_FILE))

    def restore_pose(self, name):
        rospy.loginfo("🔁 Restoring pose '{}'...".format(name))

        if not os.path.exists(POSE_LIBRARY_FILE):
            rospy.logerr("Pose file not found: {}".format(POSE_LIBRARY_FILE))
            return

        with open(POSE_LIBRARY_FILE, "r") as f:
            all_poses = yaml.safe_load(f)

        if name not in all_poses:
            rospy.logerr("Pose '{}' not found in file.".format(name))
            return

        data = all_poses[name]

        # Restore head
        head_msg = JointTrajectory()
        head_msg.joint_names = HEAD_JOINTS
        head_point = JointTrajectoryPoint()
        head_point.positions = [
            data.get("head_pan_joint", 0.0),
            data.get("head_tilt_joint", 0.0)
        ]
        head_point.time_from_start = rospy.Duration(1.0)
        head_msg.points.append(head_point)
        self.head_pub.publish(head_msg)
        rospy.loginfo("🧠 Head reset command sent.")

        # Restore arm + torso
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ARM_JOINTS + TORSO_JOINT
        point = JointTrajectoryPoint()
        point.positions = [data.get(j, 0.0) for j in ARM_JOINTS + TORSO_JOINT]
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

        rospy.loginfo("💪 Sending arm + torso trajectory goal...")
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(5.0))
        rospy.loginfo("✅ Arm and torso reset complete.")

        # Restore base
        if "base_position" in data:
            pos = data["base_position"]
            state = ModelState()
            state.model_name = MODEL_NAME
            state.pose.position.x = pos["x"]
            state.pose.position.y = pos["y"]
            state.pose.position.z = 0.0
            state.pose.orientation.w = 1.0
            self.set_model_state(state)
            rospy.loginfo("🚗 Base position reset.")
        else:
            rospy.logwarn("No base position in this pose.")

    def list_poses(self):
        if not os.path.exists(POSE_LIBRARY_FILE):
            rospy.logwarn("Pose library file not found.")
            return []

        with open(POSE_LIBRARY_FILE, "r") as f:
            all_poses = yaml.safe_load(f) or {}

        rospy.loginfo("📚 Available poses: {}".format(", ".join(all_poses.keys())))
        return list(all_poses.keys())

    def delete_pose(self, name):
		if not os.path.exists(POSE_LIBRARY_FILE):
		    rospy.logerr("Pose library file not found: {}".format(POSE_LIBRARY_FILE))
		    return

		with open(POSE_LIBRARY_FILE, "r") as f:
		    all_poses = yaml.safe_load(f) or {}

		if name not in all_poses:
		    rospy.logwarn("Pose '{}' not found.".format(name))
		    return

		del all_poses[name]

		with open(POSE_LIBRARY_FILE, "w") as f:
		    yaml.dump(all_poses, f)

		rospy.loginfo("🗑️ Deleted pose '{}'.".format(name))

# Optional: auto restore a default pose if script is run directly
if __name__ == "__main__":
    snap = PoseSnapshot()
    rospy.sleep(2.0)
    snap.list_poses()
    snap.restore_pose("detection_pose")  # Replace "default" with your saved pose name

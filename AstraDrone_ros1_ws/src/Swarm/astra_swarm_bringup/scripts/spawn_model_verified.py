#!/usr/bin/env python3
"""Spawn one Gazebo model and verify that the entity actually appears."""

import argparse
import math
import sys
import time

import rospy
from gazebo_msgs.srv import GetModelState, SpawnModel
from geometry_msgs.msg import Pose


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--param", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--robot-namespace", default="")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--verify-timeout", type=float, default=30.0)
    return parser.parse_args(rospy.myargv()[1:])


def _quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    rospy.init_node("verified_vehicle_spawn", anonymous=True)
    args = _arguments()
    if args.verify_timeout <= 0.0:
        rospy.logerr("verify timeout must be positive")
        return 2

    model_xml = rospy.get_param(args.param)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = args.x, args.y, args.z
    qx, qy, qz, qw = _quaternion(args.roll, args.pitch, args.yaw)
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=60.0)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    response = spawn(args.model, model_xml, args.robot_namespace, pose, "world")
    if not response.success:
        rospy.logwarn(
            "Gazebo spawn service returned failure for %s: %s; "
            "verifying actual entity state before failing",
            args.model,
            response.status_message,
        )

    deadline = time.monotonic() + args.verify_timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            state = get_state(args.model, "world")
            if state.success:
                rospy.loginfo("verified Gazebo entity exists: %s", args.model)
                return 0
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "model verification service error: %s", error)
        time.sleep(0.1)

    rospy.logerr("Gazebo entity did not appear before timeout: %s", args.model)
    return 1


if __name__ == "__main__":
    sys.exit(main())

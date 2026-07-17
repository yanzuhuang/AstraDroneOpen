#!/usr/bin/env python3
"""Print beginner-friendly Stage 4 tracking metrics from one rosbag."""

import argparse
import math
import statistics
import sys

import rosbag


ACTIVE_TOPIC = "/autoarming_control/tracking_active"
SETPOINT_TOPIC = "/mavros/setpoint_position/local"
ODOM_TOPIC = "/mavros/local_position/odom"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def percentile(values, ratio):
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * ratio))
    return ordered[index]


def analyze(path):
    tracking_active = False
    latest_odom = None
    first_time = None
    last_time = None
    previous_setpoint = None
    previous_time = None
    errors = []
    yaw_errors = []
    reference_speeds = []
    actual_speeds = []
    yaw_steps = []

    topics = [ACTIVE_TOPIC, SETPOINT_TOPIC, ODOM_TOPIC]
    with rosbag.Bag(path, "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        missing = [topic for topic in topics if topic not in available]
        if missing:
            raise ValueError("bag 缺少话题: " + ", ".join(missing))

        for topic, message, stamp in bag.read_messages(topics=topics):
            if topic == ACTIVE_TOPIC:
                tracking_active = bool(message.data)
                continue
            if topic == ODOM_TOPIC:
                latest_odom = message
                continue
            if not tracking_active or latest_odom is None:
                continue

            time_sec = stamp.to_sec()
            if first_time is None:
                first_time = time_sec
            last_time = time_sec
            errors.append(distance(
                message.pose.position, latest_odom.pose.pose.position))
            yaw_errors.append(abs(angle_difference(
                yaw_from_quaternion(message.pose.orientation),
                yaw_from_quaternion(latest_odom.pose.pose.orientation))))
            velocity = latest_odom.twist.twist.linear
            actual_speeds.append(math.sqrt(
                velocity.x * velocity.x +
                velocity.y * velocity.y +
                velocity.z * velocity.z))

            if previous_setpoint is not None:
                dt = time_sec - previous_time
                if 0.0 < dt < 0.5:
                    step = distance(
                        message.pose.position,
                        previous_setpoint.pose.position)
                    reference_speeds.append(step / dt)
                    yaw_steps.append(abs(angle_difference(
                        yaw_from_quaternion(message.pose.orientation),
                        yaw_from_quaternion(previous_setpoint.pose.orientation))))
            previous_setpoint = message
            previous_time = time_sec

    if not errors:
        raise ValueError("没有找到 TRACKING 阶段样本；确认录制了 tracking_active")

    moving_reference_speeds = [value for value in reference_speeds if value > 0.02]
    duration = max(0.0, last_time - first_time)
    rmse = math.sqrt(sum(value * value for value in errors) / len(errors))
    yaw_rmse = math.sqrt(
        sum(value * value for value in yaw_errors) / len(yaw_errors))
    print("阶段 4 rosbag 分析")
    print("  TRACKING 时长:       {:.2f} s".format(duration))
    print("  样本数:              {}".format(len(errors)))
    if moving_reference_speeds:
        print("  参考点移动速度中位数: {:.3f} m/s".format(
            statistics.median(moving_reference_speeds)))
    print("  实际速度中位数:       {:.3f} m/s".format(
        statistics.median(actual_speeds)))
    print("  位置误差 RMSE:        {:.3f} m".format(rmse))
    print("  位置误差 P95 / 最大:  {:.3f} / {:.3f} m".format(
        percentile(errors, 0.95), max(errors)))
    print("  yaw 误差 RMSE:        {:.2f} deg".format(
        math.degrees(yaw_rmse)))
    if yaw_steps:
        print("  相邻参考 yaw 最大步长: {:.2f} deg".format(
            math.degrees(max(yaw_steps))))


def main():
    parser = argparse.ArgumentParser(
        description="分析 stage4_trajectory.launch 录制的 rosbag")
    parser.add_argument("bag", help="rosbag 文件路径")
    args = parser.parse_args()
    try:
        analyze(args.bag)
    except (OSError, rosbag.bag.ROSBagException, ValueError) as error:
        print("分析失败: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

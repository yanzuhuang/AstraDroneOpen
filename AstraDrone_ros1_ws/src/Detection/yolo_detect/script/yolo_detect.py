#!/usr/bin/env python3
"""Parameterised, read-only ROS1 PPE detector.

This node only subscribes to an image and publishes detections/annotations. It
has no dependency on, or publisher for, mission, planner or MAVROS interfaces.
"""

import os
import time

import rospy
from astra_custom_msgs.msg import AstraDetection2D, AstraDetection2DArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.model_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~model_path", "")))
        self.image_topic = rospy.get_param("~image_topic")
        self.detections_topic = rospy.get_param("~detections_topic")
        self.annotated_topic = rospy.get_param("~annotated_topic")
        self.camera_name = rospy.get_param("~camera_name")
        self.confidence = float(rospy.get_param("~confidence_threshold", 0.35))
        self.iou = float(rospy.get_param("~iou_threshold", 0.45))
        self.image_size = int(rospy.get_param("~image_size", 640))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.max_rate = float(rospy.get_param("~max_inference_rate", 5.0))
        self.publish_annotated = bool(
            rospy.get_param("~publish_annotated_image", False))
        self.expected_names = list(rospy.get_param(
            "~expected_class_names",
            ["person", "helmet", "no_helmet", "safety_vest", "no_safety_vest"]))
        self.allowed_names = set(rospy.get_param(
            "~allowed_class_names", ["person", "no_helmet", "no_safety_vest"]))
        self.last_inference = 0.0

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError("YOLO model not found: {}".format(
                self.model_path))

        rospy.loginfo("[%s] loading model %s on %s",
                      self.camera_name, self.model_path, self.device)
        self.model = YOLO(self.model_path)
        names = self.model.names
        self.names = dict(names) if isinstance(names, dict) else dict(enumerate(names))
        available = set(self.names.values())
        missing = sorted(set(self.expected_names) - available)
        if missing:
            raise RuntimeError("model is missing expected classes: {}".format(missing))
        self.allowed_ids = {class_id for class_id, name in self.names.items()
                            if name in self.allowed_names}

        self.detections_pub = rospy.Publisher(
            self.detections_topic, AstraDetection2DArray, queue_size=1)
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = rospy.Publisher(
                self.annotated_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback,
            queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[%s] ready: %s -> %s",
                      self.camera_name, self.image_topic, self.detections_topic)

    def image_callback(self, message):
        now = time.monotonic()
        if self.max_rate > 0.0 and now - self.last_inference < 1.0 / self.max_rate:
            return
        self.last_inference = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            started = time.monotonic()
            result = self.model.predict(
                source=image, conf=self.confidence, iou=self.iou,
                imgsz=self.image_size, device=self.device, verbose=False)[0]
            inference_ms = (time.monotonic() - started) * 1000.0
        except (CvBridgeError, Exception) as error:
            rospy.logerr_throttle(2.0, "[%s] inference failed: %s",
                                  self.camera_name, error)
            return

        output = AstraDetection2DArray()
        output.header = message.header
        output.camera_name = self.camera_name
        output.image_width = message.width
        output.image_height = message.height
        output.inference_ms = inference_ms
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                if class_id not in self.allowed_ids:
                    continue
                xywh = box.xywh[0].tolist()
                detection = AstraDetection2D()
                detection.class_id = class_id
                detection.class_name = self.names[class_id]
                detection.confidence = float(box.conf[0].item())
                detection.center_x, detection.center_y = xywh[0], xywh[1]
                detection.width, detection.height = xywh[2], xywh[3]
                output.detections.append(detection)
        self.detections_pub.publish(output)

        if self.annotated_pub is not None:
            annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding="bgr8")
            annotated.header = message.header
            self.annotated_pub.publish(annotated)


if __name__ == "__main__":
    rospy.init_node("ppe_yolo")
    try:
        YoloDetector()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("YOLO startup failed: %s", error)
        raise

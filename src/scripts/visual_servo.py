#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge


class VisualServoNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.user_cmd_topic = rospy.get_param('~user_cmd_topic', '/user_cmd')

        self.kp_yaw = rospy.get_param('~kp_yaw', 1.2)
        self.max_yaw = rospy.get_param('~max_yaw', 0.8)
        self.forward_speed = rospy.get_param('~forward_speed', 0.25)
        self.search_yaw = rospy.get_param('~search_yaw', 0.3)

        self.min_area = rospy.get_param('~min_area', 2000.0)
        self.stop_area = rospy.get_param('~stop_area', 15000.0)
        self.center_tol = rospy.get_param('~center_tol', 0.08)  # normalized

        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/visual_servo/debug_image')
        self.hsv_lower1 = np.array(rospy.get_param('~hsv_lower1', [0, 80, 50]), dtype=np.uint8)
        self.hsv_upper1 = np.array(rospy.get_param('~hsv_upper1', [10, 255, 255]), dtype=np.uint8)
        self.hsv_lower2 = np.array(rospy.get_param('~hsv_lower2', [160, 80, 50]), dtype=np.uint8)
        self.hsv_upper2 = np.array(rospy.get_param('~hsv_upper2', [179, 255, 255]), dtype=np.uint8)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.user_pub = rospy.Publisher(self.user_cmd_topic, Int8, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)

        self.last_log = time.time()
        self.last_cmd = Twist()

        rospy.loginfo(f"visual_servo: subscribed to {self.image_topic}")
        rospy.loginfo(f"visual_servo: debug image topic {self.debug_image_topic}")
        self._send_start_commands()

    def _send_start_commands(self):
        rospy.sleep(0.5)
        self.user_pub.publish(Int8(data=2))  # FIXEDSTAND
        rospy.sleep(0.8)
        self.user_pub.publish(Int8(data=1))  # START
        rospy.sleep(0.5)

    def _compute_red_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower1, self.hsv_upper1)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"visual_servo: cv_bridge conversion failed: {e}")
            return

        h, w = bgr.shape[:2]
        mask = self._compute_red_mask(bgr)
        m = cv2.moments(mask)
        area = float(m['m00'])

        cmd = Twist()
        if area < self.min_area:
            cmd.angular.z = self.search_yaw
            cmd.linear.x = 0.0
            target = False
            error = 0.0
        else:
            cx = m['m10'] / m['m00']
            error = (cx - (w / 2.0)) / (w / 2.0)
            if abs(error) < self.center_tol:
                error = 0.0
            yaw = -self.kp_yaw * error
            yaw = max(-self.max_yaw, min(self.max_yaw, yaw))
            cmd.angular.z = yaw
            cmd.linear.x = 0.0 if area >= self.stop_area else self.forward_speed
            target = True

        debug = bgr.copy()
        if area >= self.min_area:
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            cv2.circle(debug, (cx, cy), 6, (0, 255, 0), -1)
            cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        cv2.putText(debug, f"area={area:.0f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(debug, f"err={error:.3f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception:
            pass

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

        now = time.time()
        if now - self.last_log >= 1.0:
            rospy.loginfo(
                f"visual_servo: area={area:.0f} error={error:.3f} cmd=(v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}) target={target}"
            )
            self.last_log = now


def main():
    rospy.init_node('visual_servo', anonymous=True)
    VisualServoNode()
    rospy.spin()


if __name__ == '__main__':
    main()

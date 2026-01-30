#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisionPerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_log = rospy.Time(0)
        self.topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.sub = rospy.Subscriber(self.topic, Image, self.on_image, queue_size=1)
        rospy.loginfo(f"vision_perception: subscribed to {self.topic}")

    def on_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"vision_perception: cv_bridge conversion failed: {e}")
            return

        now = rospy.Time.now()
        if (now - self.last_log).to_sec() >= 1.0:
            h, w = cv_img.shape[:2]
            rospy.loginfo(f"vision_perception: image {w}x{h}, encoding={msg.encoding}")
            self.last_log = now


def main():
    rospy.init_node('vision_perception', anonymous=True)
    VisionPerceptionNode()
    rospy.spin()


if __name__ == '__main__':
    main()

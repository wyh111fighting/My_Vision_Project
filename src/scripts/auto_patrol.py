#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetWorldProperties
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int8
import time
import math

def _detect_model_name():
    try:
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=2.0)
        getp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        resp = getp()
        for name in resp.model_names:
            if 'go1' in name:
                return name
        for name in resp.model_names:
            if 'ground' not in name and 'plane' not in name:
                return name
    except Exception:
        pass
    return 'go1_gazebo'


def _get_rpy(model_name, timeout=0.2):
    try:
        ms = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=timeout)
    except Exception:
        return None
    if model_name in ms.name:
        i = ms.name.index(model_name)
        q = ms.pose[i].orientation
        return euler_from_quaternion((q.x, q.y, q.z, q.w))
    return None


def _get_pose(model_name, timeout=0.2):
    try:
        ms = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=timeout)
    except Exception:
        return None
    if model_name in ms.name:
        i = ms.name.index(model_name)
        return ms.pose[i]
    return None


def square_patrol(pub, cmd_pub, model_name,
                  forward_speed=0.5, turn_speed=0.8,
                  forward_time=11.874, turn_time=3.285, loops=1,
                  accel_time=0.6, decel_time=0.4, settle_time=0.2,
                  max_roll_deg=35.0, max_pitch_deg=35.0,
                  turn_factor=1.3, min_turn_time=4.5, use_feedback=True,
                  yaw_tolerance_deg=1.5, distance_tolerance=0.04):
    rate_hz = 10
    rate = rospy.Rate(rate_hz)

    def is_tipped():
        rpy = _get_rpy(model_name)
        if rpy is None:
            return False
        roll, pitch, _ = rpy
        return abs(math.degrees(roll)) > max_roll_deg or abs(math.degrees(pitch)) > max_pitch_deg

    def ramped_publish(linear_x, angular_z, duration):
        if duration <= 0:
            return
        steps = max(1, int(duration * rate_hz))
        for i in range(steps):
            if rospy.is_shutdown():
                break
            if is_tipped():
                rospy.logerr("Tip detected, stopping and switching to FIXEDSTAND")
                pub.publish(Twist())
                try:
                    cmd_pub.publish(Int8(data=2))
                except Exception:
                    pass
                return
            # triangular ramp profile
            t = (i + 1) / float(steps)
            scale = min(1.0, t / max(1e-3, accel_time / max(duration, 1e-3)))
            if t > 1.0 - (decel_time / max(duration, 1e-3)):
                scale = max(0.2, (1.0 - t) / max(1e-3, decel_time / max(duration, 1e-3)))
            twist = Twist()
            twist.linear.x = linear_x * scale
            twist.angular.z = angular_z * scale
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())
    def forward_with_feedback(target_dist, max_time):
        pose0 = _get_pose(model_name)
        if pose0 is None:
            ramped_publish(forward_speed, 0.0, max_time)
            return
        q = pose0.orientation
        _, _, yaw0 = euler_from_quaternion((q.x, q.y, q.z, q.w))
        x0, y0 = pose0.position.x, pose0.position.y

        rate = rospy.Rate(rate_hz)
        end = time.time() + max_time
        while not rospy.is_shutdown() and time.time() < end:
            pose = _get_pose(model_name)
            if pose is None:
                break
            dx = pose.position.x - x0
            dy = pose.position.y - y0
            dist = dx * math.cos(yaw0) + dy * math.sin(yaw0)
            if dist >= max(0.0, target_dist - distance_tolerance):
                break
            if is_tipped():
                rospy.logerr("Tip detected, stopping and switching to FIXEDSTAND")
                pub.publish(Twist())
                try:
                    cmd_pub.publish(Int8(data=2))
                except Exception:
                    pass
                return
            twist = Twist()
            twist.linear.x = forward_speed
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())

    def turn_with_feedback(target_yaw, max_time):
        pose0 = _get_pose(model_name)
        if pose0 is None:
            ramped_publish(0.0, turn_speed, max_time)
            return
        q = pose0.orientation
        _, _, yaw0 = euler_from_quaternion((q.x, q.y, q.z, q.w))

        rate = rospy.Rate(rate_hz)
        end = time.time() + max_time
        tol = math.radians(yaw_tolerance_deg)
        while not rospy.is_shutdown() and time.time() < end:
            pose = _get_pose(model_name)
            if pose is None:
                break
            q2 = pose.orientation
            _, _, yaw = euler_from_quaternion((q2.x, q2.y, q2.z, q2.w))
            delta = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
            if abs(delta) >= abs(target_yaw) - tol:
                break
            if is_tipped():
                rospy.logerr("Tip detected, stopping and switching to FIXEDSTAND")
                pub.publish(Twist())
                try:
                    cmd_pub.publish(Int8(data=2))
                except Exception:
                    pass
                return
            twist = Twist()
            twist.angular.z = math.copysign(abs(turn_speed), target_yaw)
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())

    for l in range(loops):
        for side in range(4):
            # move forward
            rospy.loginfo(f"Patrol: forward side {side+1}/{4}, loop {l+1}/{loops}")
            target_dist = abs(forward_speed) * float(forward_time)
            if use_feedback:
                forward_with_feedback(target_dist, forward_time)
            else:
                ramped_publish(forward_speed, 0.0, forward_time)
            rospy.sleep(settle_time)

            # rotate
            rospy.loginfo(f"Patrol: turning")
            turn_time_scaled = max(turn_time * float(turn_factor), float(min_turn_time))
            if use_feedback:
                turn_with_feedback(math.pi / 2.0, turn_time_scaled)
            else:
                ramped_publish(0.0, turn_speed, turn_time_scaled)
            rospy.sleep(settle_time)

    # final stop
    pub.publish(Twist())
    rospy.loginfo("Patrol finished, robot stopped.")

def main():
    rospy.init_node('auto_patrol', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # publisher for user command (int): 1=START, 2=L2_A (fixedStand)
    from std_msgs.msg import Int8
    cmd_pub = rospy.Publisher('/user_cmd', Int8, queue_size=1)
    # configurable params
    forward_speed = rospy.get_param('~forward_speed', 0.5)
    turn_speed = rospy.get_param('~turn_speed', 0.8)
    forward_time = rospy.get_param('~forward_time', 11.874)
    turn_time = rospy.get_param('~turn_time', 3.285)
    side_length = rospy.get_param('~side_length', None)
    # Correction factor to compensate for controller scaling of angular command
    turn_factor = rospy.get_param('~turn_factor', 1.3)
    loops = rospy.get_param('~loops', 1)
    accel_time = rospy.get_param('~accel_time', 0.6)
    decel_time = rospy.get_param('~decel_time', 0.4)
    settle_time = rospy.get_param('~settle_time', 0.2)
    max_roll_deg = rospy.get_param('~max_roll_deg', 35.0)
    max_pitch_deg = rospy.get_param('~max_pitch_deg', 35.0)
    min_turn_time = rospy.get_param('~min_turn_time', 4.5)
    use_feedback = rospy.get_param('~use_feedback', True)
    yaw_tolerance_deg = rospy.get_param('~yaw_tolerance_deg', 1.5)
    distance_tolerance = rospy.get_param('~distance_tolerance', 0.04)

    # If user specifies desired side length (meters), compute forward_time = side_length / forward_speed
    # and compute turn_time to perform ~90 degrees: turn_time = (pi/2) / abs(turn_speed)
    if side_length is not None:
        try:
            import math
            if forward_speed == 0:
                rospy.logwarn("forward_speed is 0, cannot compute forward_time from side_length")
            else:
                forward_time = float(side_length) / float(abs(forward_speed))
            # compute turn_time based on angular speed to achieve 90 degrees
            if turn_speed == 0:
                rospy.logwarn("turn_speed is 0, cannot compute turn_time for 90 degrees")
            else:
                turn_time = (math.pi/2.0) / float(abs(turn_speed))
            rospy.loginfo(f"Computed timings from side_length={side_length}: forward_time={forward_time:.3f}, turn_time={turn_time:.3f}")
        except Exception as e:
            rospy.logwarn(f"Failed to compute timings from side_length: {e}")
    else:
        pass

    model_name = _detect_model_name()
    rospy.loginfo(f"Detected model name: {model_name}")
    rospy.loginfo(f"auto_patrol starting: forward_speed={forward_speed}, turn_speed={turn_speed}, forward_time={forward_time}, turn_time={turn_time}, loops={loops}")

    # wait for subscribers to appear (optional)
    timeout = time.time() + 5.0
    while not rospy.is_shutdown() and (pub.get_num_connections() == 0) and time.time() < timeout:
        rospy.loginfo_once("Waiting for /cmd_vel subscribers...")
        rospy.sleep(0.2)

    # send mode commands: first go to FIXEDSTAND (L2_A=2), then START(1) to enable trotting
    rospy.sleep(0.5)
    rospy.loginfo("auto_patrol: requesting FIXEDSTAND")
    cmd_pub.publish(Int8(data=2))
    rospy.sleep(0.8)
    rospy.loginfo("auto_patrol: requesting START (trotting)")
    cmd_pub.publish(Int8(data=1))
    rospy.sleep(0.5)

    try:
        square_patrol(
            pub, cmd_pub, model_name,
            forward_speed, turn_speed, forward_time, turn_time, loops,
            accel_time=accel_time, decel_time=decel_time, settle_time=settle_time,
            max_roll_deg=max_roll_deg, max_pitch_deg=max_pitch_deg,
            turn_factor=turn_factor, min_turn_time=min_turn_time, use_feedback=use_feedback,
            yaw_tolerance_deg=yaw_tolerance_deg, distance_tolerance=distance_tolerance,
        )
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

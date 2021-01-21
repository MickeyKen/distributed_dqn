#!/usr/bin/env python3
import os
import rospy
import numpy as np
import math
from math import pi
import random
import quaternion
import time

from std_msgs.msg import Float64, Int32, Float64MultiArray
from geometry_msgs.msg import Twist, Point, Pose, Vector3, Quaternion
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

out_path = 'environment_output_test_0110_1.txt'

diagonal_dis = 6.68
goal_model_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..'
                                , 'models', 'person_standing', 'model.sdf')

PAN_LIMIT = math.radians(90)  #2.9670
TILT_MIN_LIMIT = math.radians(90) - math.atan(2.5/0.998)
TILT_MAX_LIMIT = math.radians(90) - math.atan(1.5/0.998)

PAN_STEP = math.radians(5)
TILT_STEP = math.radians(3)

HUMAN_XMAX = 2.8
HUMAN_XMIN = 0.0
HUMAN_YMAX = 4.8
HUMAN_YMIN = 2.5

EYE_AREA = 45.0

class Env1():
    def __init__(self, is_training, ROS_MASTER_URI):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(ROS_MASTER_URI) + '/'
        self.position = Pose()
        self.projector_position = Pose()
        self.goal_position = Pose()
        self.goal_projector_position = Pose()

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)

        self.sub_target_human_pose = rospy.Subscriber('/filtering/scan/detect_leg_person', Pose, self.getTargetHumanPose)

        self.head_pub = rospy.Publisher('/dynamixel_workbench_head/joint_trajectory', JointTrajectory, queue_size=100)

        self.image_pub = rospy.Publisher('/ubiquitous_display/image', Int32, queue_size=10)
        self.view_pub = rospy.Publisher('/view', Float64MultiArray, queue_size=10)
        self.past_distance = 0.
        self.past_distance_rate = 0.
        self.past_projector_distance = 0.
        self.yaw = 0
        self.pan_ang = 0.
        self.tilt_ang = 0.
        self.v = 0.
        self.ud_x = 0.
        self.diff_distance = 0.
        self.diff_angle = 0.


        if is_training:
            self.threshold_arrive = 0.2
            self.min_threshold_arrive = 1.5
            self.max_threshold_arrive = 2.5
        else:
            self.threshold_arrive = 0.5
            self.min_threshold_arrive = 1.5
            self.max_threshold_arrive = 2.5

    def publishPantilt(self, control_pan_pos, control_tilt_pos):
        head_msg = JointTrajectory()
        head_msg.joint_names = [ "pan_joint", "tilt_joint"]
        jtp_msg.positions = [control_pan_pos,control_tilt_pos]
        jtp_msg.velocities = [0.5,0.5]
        jtp_msg.time_from_start = rospy.Duration.from_sec(0.00000002)

        head_msg.points.append(jtp_msg)
        head_pub.publish(head_msg)

    def getTargetHumanPose(self, msg):
        print msg
        self.goal_position.position.x = msg.position.x
        self.goal_position.position.y = msg.position.y
        self.goal_projector_position.position.x = msg.position.x
        self.goal_projector_position.position.y = msg.position.y - 2.5

    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input

        return input

    def getGoalDistace(self, x_distance, y_distance):
        goal_distance = math.hypot(x_distance, y_distance)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        self.ud_x = self.position.x
        self.v = odom.twist.twist.linear.x
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw = 0

    def getProjState(self):
        reach = False

        radian = math.radians(self.yaw) + self.pan_ang + math.radians(90)
        distance = 0.998 * math.tan(math.radians(90) - self.tilt_ang)
        self.projector_position.position.x = distance * math.cos(radian) + self.position.x
        self.projector_position.position.y = distance * math.sin(radian) + self.position.y
        diff = math.hypot(self.goal_projector_position.position.x - self.projector_position.position.x, self.goal_projector_position.position.y - self.projector_position.position.y)
        # print ("now: ", self.projector_position.position.x, self.projector_position.position.y)
        # print ("goal: ", self.goal_projector_position.position.x, self.goal_projector_position.position.y)
        if diff <= self.threshold_arrive:
            # done = True
            reach = True
        # print ("position x: ", round(self.position.x,1))
        # print ("projector x: ", round(self.projector_position.position.x,1), ",projector y: ", round(self.projector_position.position.y,1))
        return diff, reach

    def getState(self, scan):
        scan_range = []
        yaw = self.yaw
        min_range = 0.8
        done = False
        arrive = False

        rel_dis_x = round(self.goal_projector_position.position.x - self.position.x, 1)
        rel_dis_y = round(self.goal_projector_position.position.y - self.position.y, 1)
        diff_distance = math.hypot(rel_dis_x, rel_dis_y)
        rel_dis_hu_x = round(self.goal_position.position.x - self.position.x, 1)
        rel_dis_hu_y = round(self.goal_position.position.y - self.position.y, 1)
        diff_hu_distance = math.hypot(rel_dis_hu_x, rel_dis_hu_y)

        if rel_dis_x > 0 and rel_dis_y > 0:
            theta = math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x > 0 and rel_dis_y < 0:
            theta = 2.0 * math.pi + math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x < 0 and rel_dis_y < 0:
            theta = math.pi + abs(math.atan(rel_dis_y / rel_dis_x))
        elif rel_dis_x < 0 and rel_dis_y > 0:
            theta = math.pi + math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x == 0 and rel_dis_y > 0:
            theta = 1.0 / 2.0 * math.pi
        elif rel_dis_x == 0 and rel_dis_y < 0:
            theta = 3.0 / 2.0 * math.pi
        elif rel_dis_y == 0 and rel_dis_x > 0:
            theta = 0
        else:
            theta = math.pi

        rel_theta = round(math.degrees(theta), 2)
        diff_angle = abs(rel_theta - yaw)

        # print "rel_dis_x: ",rel_dis_x, ", rel_dis_y: ", rel_dis_y, ", rel_theta: ", rel_theta, ", diff_angle: ", diff_angle

        if diff_angle <= 180:
            diff_angle = round(diff_angle, 2)
        else:
            diff_angle = round(360 - diff_angle, 2)


        if rel_dis_hu_x > 0 and rel_dis_hu_y > 0:
            theta2 = math.atan(rel_dis_hu_y / rel_dis_hu_x)
            h_theta = (1.0 / 2.0 * math.pi) - theta2
        elif rel_dis_hu_x > 0 and rel_dis_hu_y < 0:
            theta2 = 2.0 * math.pi + math.atan(rel_dis_hu_y / rel_dis_hu_x)
            h_theta = math.radians(450) - theta2
        elif rel_dis_hu_x < 0 and rel_dis_hu_y < 0:
            theta2 = math.pi + abs(math.atan(rel_dis_hu_y / rel_dis_hu_x))
            h_theta = theta2 - (1.0 / 2.0 * math.pi)
        elif rel_dis_hu_x < 0 and rel_dis_hu_y > 0:
            theta2 = math.pi + math.atan(rel_dis_hu_y / rel_dis_hu_x)
            h_theta = theta2 - (1.0 / 2.0 * math.pi)
        elif rel_dis_hu_x == 0 and rel_dis_hu_y > 0:
            theta2 = 1.0 / 2.0 * math.pi
            h_theta = 0.0
        elif rel_dis_hu_x == 0 and rel_dis_hu_y < 0:
            theta2 = 3.0 / 2.0 * math.pi
            h_theta = math.pi
        elif rel_dis_hu_y == 0 and rel_dis_hu_x > 0:
            theta2 = 0
            h_theta = (1.0 / 2.0 * math.pi)
        else:
            theta2 = math.pi
            h_theta = (1.0 / 2.0 * math.pi)

        rel_theta2 = round(math.degrees(theta2), 2)
        diff_angle2 = abs(rel_theta2 - yaw)

        rel_h_theta = round(math.degrees(h_theta), 2)
        diff_hu_angle = abs(rel_h_theta)
        # print diff_hu_angle
        # print "rel_dis_x: ",rel_dis_x, ", rel_dis_y: ", rel_dis_y, ", rel_theta: ", rel_theta, ", diff_angle: ", diff_angle

        if diff_hu_angle <= 180:
            diff_hu_angle = round(diff_hu_angle, 2)
        else:
            diff_hu_angle = round(360 - diff_hu_angle, 2)

        scan_range.append(scan.ranges[0])
        scan_range.append(scan.ranges[540])

        if min_range > min(scan_range) > 0:
            done = True

        # current_distance = math.hypot(self.goal_projector_position.position.x- self.position.position.x, self.goal_projector_position.position.y - self.position.position.y)
        if diff_distance >= self.min_threshold_arrive and diff_distance <= self.max_threshold_arrive and diff_hu_angle < EYE_AREA:
            # done = True
            arrive = True

        # print "diff_distance: ", diff_distance, ", diff_angle: ", diff_angle

        return scan_range, yaw, diff_angle, diff_distance, diff_hu_angle, diff_hu_distance, done, arrive

    def setReward(self, done, arrive, step):

        _, reach = self.getProjState()
        # current_projector_distance, reach = self.getProjState()
        # current_distance = math.hypot(self.goal_projector_position.position.x - self.position.x, self.goal_projector_position.position.y - self.position.y)

        reward = -1

        if done:
            reward = -200
            self.pub_cmd_vel.publish(Twist())
            filehandle = open(out_path, 'a+')
            filehandle.write("done" + ',' + str(self.goal_projector_position.position.x)+ ',' + str(self.goal_projector_position.position.y) +  ',' + str(self.goal_position.position.x) + ',' + str(self.goal_position.position.y) + "\n")
            filehandle.close()
        else:

            if step == 149:
                reward = -200
                filehandle = open(out_path, 'a+')
                filehandle.write("timeout" + ',' + str(self.goal_projector_position.position.x)+ ',' + str(self.goal_projector_position.position.y) +  ',' + str(self.goal_position.position.x) + ',' + str(self.goal_position.position.y) + "\n")
                filehandle.close()

            else:
                if arrive and reach:
                    reward = 150
                    done = True
                    filehandle = open(out_path, 'a+')
                    filehandle.write("arrive" + ',' + str(self.goal_projector_position.position.x)+ ',' + str(self.goal_projector_position.position.y) +  ',' + str(self.goal_position.position.x) + ',' + str(self.goal_position.position.y) + "\n")
                    filehandle.close()

        #         else:
        #             diff_pos = self.past_distance - current_distance
        #             diff_projector_pos = self.past_projector_distance - current_projector_distance
        #
        #             # if current_distance > self.min_threshold_arrive:
        #             r_c = 150.0 * abs(self.past_distance - current_distance)
        #
        #             r_p = 8.0 * abs(self.past_projector_distance - current_projector_distance)
        #
        #             if diff_pos < 0 or diff_projector_pos < 0:
        #                 reward = -((r_c+0.1) * r_p)
        #             else:
        #                 reward = (r_c+0.1) * r_p
        #
        #
        # self.past_distance = current_distance
        # self.past_projector_distance = current_projector_distance

        return reward, arrive, reach, done


    def step(self, action, past_action, step):

        vel_cmd = Twist()
        # if action == 0:
        #     self.pub_cmd_vel.publish(vel_cmd)
        if action == 0:
            vel_cmd.linear.x = 0.2
            self.pub_cmd_vel.publish(vel_cmd)
        elif action == 1:
            vel_cmd.linear.x = -0.2
            self.pub_cmd_vel.publish(vel_cmd)
        elif action == 2:
            pass
        elif action == 3:
            self.pan_ang = self.constrain(self.pan_ang + PAN_STEP, -PAN_LIMIT, PAN_LIMIT)
            self.pan_pub.publish(self.pan_ang)
        elif action == 4:
            self.pan_ang = self.constrain(self.pan_ang - PAN_STEP, -PAN_LIMIT, PAN_LIMIT)
            self.pan_pub.publish(self.pan_ang)
        elif action == 5:
            self.tilt_ang = self.constrain(self.tilt_ang + TILT_STEP, TILT_MIN_LIMIT, TILT_MAX_LIMIT)
            self.tilt_pub.publish(self.tilt_ang)
        elif action == 6:
            self.tilt_ang = self.constrain(self.tilt_ang - TILT_STEP, TILT_MIN_LIMIT, TILT_MAX_LIMIT)
            self.tilt_pub.publish(self.tilt_ang)
        elif action == 7:
            self.pub_cmd_vel.publish(vel_cmd)
        else:
            print ("Error action is from 0 to 6")

        time.sleep(0.2)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=5)
            except:
                pass


        state, yaw, diff_angle, diff_distance, diff_hu_angle, diff_hu_distance, done, arrive = self.getState(data)
        state = [i / 25. for i in state]

        state.append(self.constrain(self.pan_ang / PAN_LIMIT, -1.0, 1.0))
        state.append(self.constrain(self.tilt_ang / TILT_MAX_LIMIT, -1.0, 1.0))
        state.append(self.constrain(self.v, -1.0, 1.0))

        state.append(diff_angle / 180)
        state.append(self.constrain(diff_distance / diagonal_dis, -1.0, 1.0))
        state.append(diff_hu_angle / 180)
        state.append(self.constrain(diff_hu_distance / diagonal_dis, -1.0, 1.0))
        # print state
        # state = state + [yaw / 360, rel_theta / 360, diff_angle / 180]
        reward, arrive, reach, done = self.setReward(done, arrive,step)

        return np.asarray(state), reward, done, reach, arrive

    def cal_actor_pose(self, distance):
        xp = 0.
        yp = 0.
        rxp = 0.
        ryp = 0.
        rq = Quaternion()
        # xp = random.uniform(-2.8, 2.8)
        # yp = random.uniform(3.0, 5.0)
        # ang = 0
        # rxp = xp + (distance * math.sin(math.radians(ang)))
        # ryp = yp - (distance * math.cos(math.radians(ang)))
        # q = quaternion.from_euler_angles(0,0,math.radians(ang))
        # rq.x = q.x
        # rq.y = q.y
        # rq.z = q.z
        # rq.w = q.w
        while True:
            xp = random.uniform(HUMAN_XMIN, HUMAN_XMAX)
            yp = random.uniform(HUMAN_YMIN, HUMAN_YMAX)
            ang = 0
            rxp = xp + (distance * math.sin(math.radians(ang)))
            ryp = yp - (distance * math.cos(math.radians(ang)))
            human_ud_distance = math.hypot(rxp, ryp)
            if rxp < HUMAN_XMAX and rxp > HUMAN_XMIN and ryp < HUMAN_YMAX and ryp > HUMAN_YMIN - distance and human_ud_distance > self.max_threshold_arrive:
                # print human_ud_distance
                q = quaternion.from_euler_angles(0,0,math.radians(ang))
                rq.x = q.x
                rq.y = q.y
                rq.z = q.z
                rq.w = q.w
                break
        return xp, yp, rxp, ryp, rq

    def reset(self):

        self.pan_ang = 0.0
        self.tilt_ang = TILT_MIN_LIMIT

        self.pan_pub.publish(self.pan_ang)
        self.tilt_pub.publish(self.tilt_ang)
        self.pub_cmd_vel.publish(Twist())

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=5)
            except:
                pass

        state, yaw, diff_angle, diff_distance, diff_hu_angle, diff_hu_distance, done, arrive = self.getState(data)
        state = [i / 25. for i in state]

        state.append(0.0)
        state.append(TILT_MIN_LIMIT / TILT_MAX_LIMIT)
        state.append(self.constrain(self.v, -1.0, 1.0))

        state.append(diff_angle / 180)
        state.append(self.constrain(diff_distance / diagonal_dis, -1.0, 1.0))
        state.append(diff_hu_angle / 180)
        state.append(self.constrain(diff_hu_distance / diagonal_dis, -1.0, 1.0))
        # state = state + [yaw / 360, rel_theta / 360, diff_angle / 180]

        return np.asarray(state)
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion

class RobotFollower:
    def __init__(self):
        rospy.init_node('control')
        # 初始化robot1的位置信息
        self.robot1_x = 0
        self.robot1_y = 0
        self.robot1_initialized = False

        # 订阅robot1和robot2的里程计话题
        rospy.Subscriber('/mbot1_odom', Odometry, self.robot1_odom_callback)
        rospy.Subscriber('/mbot2_odom', Odometry, self.robot2_odom_callback)

        # 发布robot2的速度控制命令
        self.cmd_vel_pub = rospy.Publisher('/mbot2_vel', Twist, queue_size=10)

        # 初始化速度控制消息
        self.cmd_vel_msg = Twist()

        # 设置控制参数
        self.linear_speed = 0.2  # 线速度
        self.angular_speed = 0.5  # 角速度

        # PID控制参数
        self.Kp = 0.8
        self.Ki = 0.0
        self.Kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0

        # PID控制参数
        self.vx_Kp = 1.5
        self.vx_Ki = 0.1
        self.vx_Kd = 0.5
        self.dis_prev_error = 0.0
        self.dis_last_prev_error = 0.0

    def robot1_odom_callback(self, msg):
        # 当接收到robot1的里程计数据时，更新robot1的位置信息
        # print("get_odom1")
        self.robot1_x = msg.pose.pose.position.x        #- 1
        self.robot1_y = msg.pose.pose.position.y        #- 0.2
        self.robot1_initialized = True

    def robot2_odom_callback(self, msg):
        # print("get_odom2")
        # 当接收到robot2的里程计数据时，计算控制命令以跟随robot1
        if not self.robot1_initialized:
            rospy.logwarn("Robot1 position not yet initialized!")
            return
        # 当接收到robot2的里程计数据时，计算控制命令以跟随robot1
        robot2_x = msg.pose.pose.position.x
        robot2_y = msg.pose.pose.position.y

        # 计算机器人2跟随机器人1的线速度和角速度
        distance = math.sqrt((robot2_x - self.robot1_x)**2 + (robot2_y - self.robot1_y)**2) - 1
        # 设置机器人2的速度控制命令
        # self.cmd_vel_msg.linear.x = self.linear_speed * distance *1.8
        self.cmd_vel_msg.linear.x = self.linear_speed *(self.vx_Kp*distance + self.vx_Kd*self.dis_prev_error + self.vx_Kd*(distance+self.dis_prev_error+self.dis_last_prev_error) )
        self.dis_last_prev_error=self.dis_prev_error
        self.dis_prev_error=distance

        angle_to_robot1 = math.atan2(self.robot1_y - robot2_y, self.robot1_x - robot2_x)
        # 将角度统一到0到2π之间
        if angle_to_robot1 < 0:
            angle_to_robot1 += 2 * math.pi
        # 转换四元数姿态为欧拉角，并计算偏航角误差
        quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)
        if yaw <0:
            yaw += 2 * math.pi
        # print(yaw)
        angle_err = angle_to_robot1 - yaw

        if(angle_err > math.pi):
            angle_err -= 2*math.pi
        elif (angle_err < -math.pi):
            angle_err += 2*math.pi

        # PID控制
        self.integral += angle_err
        derivative = angle_err - self.prev_error
        self.prev_error = angle_err
        self.cmd_vel_msg.angular.z = self.Kp * angle_err + self.Ki * self.integral + self.Kd * derivative

        # print(msg.pose.pose.orientation.z)
        # angle_err = angle_to_robot1 - msg.pose.pose.orientation.z
        # PID控制
        # self.cmd_vel_msg.angular.z = 0.8 *angle_err
        # self.cmd_vel_msg.angular.z = self.angular_speed * (angle_to_robot1 - msg.pose.pose.orientation.z)

        # 发布速度控制命令
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

if __name__ == '__main__':
    try:
        robot_follower = RobotFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
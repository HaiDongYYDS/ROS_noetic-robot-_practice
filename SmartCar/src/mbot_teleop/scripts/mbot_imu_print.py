#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import Imu  
from geometry_msgs.msg import Twist  
import numpy as np  
import math  
import tf  

# 定义IMU话题和速度控制话题  
imu_topic = '/imu_raw'  # 假设IMU数据发布到这个话题  
cmd_vel_topic = '/cmd_vel'  # 速度控制指令发布到这个话题  
  
# 定义IMU滤波器参数（这里使用简单的互补滤波器作为示例）  
alpha = 0.98  # 互补滤波器参数，用于平衡当前测量和先前估计的权重  
  
# 初始化姿态和角速度估计  
orientation_estimate = np.array([0.0, 0.0, 0.0])  # 假设初始姿态为0  
angular_velocity_filtered = np.array([0.0, 0.0, 0.0])  # 初始角速度估计为0  

linear_speed = 0.4
angular_speed = 0.5
duration1 = 12.58
duration2 = 15.85
count=0

def circle():

    twist_msg = Twist()  
    twist_msg.linear.x = linear_speed  
    twist_msg.angular.z = angular_speed  
    cmd_vel_publisher.publish(twist_msg) 
    rospy.sleep(duration1) 
    twist_msg.angular.z = -angular_speed*0.8
    cmd_vel_publisher.publish(twist_msg) 
    rospy.sleep(duration2)
# 定义IMU回调函数  
def callback_imu(imu_msg):  
    global orientation_estimate, angular_velocity_filtered  
    global count  # 声明我们将使用全局变量count 
    
   # 解算IMU数据（这里仅提取四元数并转换为欧拉角）  
    orientation_quat = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]  
    orientation_euler = tf.transformations.euler_from_quaternion(orientation_quat)  
    
    # 提取角速度数据（这里仅提取x, y, z轴上的角速度）  
    angular_velocity_raw = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])  
      
    # 对角速度进行滤波（使用互补滤波器）  
    angular_velocity_filtered = alpha * angular_velocity_filtered + (1 - alpha) * angular_velocity_raw  

    count+=1  
     # 输出角度和角速度信息 
    if count>100: 
        rospy.loginfo(f"Roll: {orientation_euler[0]:.2f}°, Pitch: {orientation_euler[1]:.2f}°, Yaw: {orientation_euler[2]:.2f}°")  
        rospy.loginfo(f"Angular Velocity (deg/sec): {angular_velocity_filtered[0]:.2f}, {angular_velocity_filtered[1]:.2f}, {angular_velocity_filtered[2]:.2f}")
        count=0  
        

if __name__ == '__main__':  
    # 初始化ROS节点  
    rospy.init_node('mbot_imu_print', anonymous=True)  
    # 创建IMU订阅者和速度发布者  
    imu_subscriber = rospy.Subscriber(imu_topic, Imu, callback_imu)  
    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10) 
    while(True):
        circle()
    # 主循环，保持节点运行  
    rospy.spin()
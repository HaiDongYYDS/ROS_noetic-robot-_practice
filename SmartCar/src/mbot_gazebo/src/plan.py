#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

import math

msg = """
Control mbot!
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def plan_T(speed1, time):
    linear_speed = 0.2  # m/s
    angular_speed = 0.5  # rad/s

    if time <= 132:
        speed1[0] = linear_speed
        speed1[5] = angular_speed
    else:
        speed1[0] = linear_speed
        speed1[5] = -angular_speed

def car_model(t):
    def car(x, t):
        dxdt = np.zeros(3)
        dxdt[0] = ((np.cos(t/20)/2 - np.cos(t/20 + 1/200)/2)/(10*np.sin(t/10) - 10*np.sin(t/10 + 1/100)) - ((10*np.sin(t/20) - 10*np.sin(t/20 + 1/200))*(np.cos(t/10) - np.cos(t/10 + 1/100)))/(10*np.sin(t/10) - 10*np.sin(t/10 + 1/100))**2)/((10*np.sin(t/20) - 10*np.sin(t/20 + 1/200))**2/(10*np.sin(t/10) - 10*np.sin(t/10 + 1/100))**2 + 1)
        dxdt[1] = ((np.cos(t/10)**2 + np.cos(t/20)**2/4)**(1/2))*np.cos(x[0])
        dxdt[2] = ((np.cos(t/10)**2 + np.cos(t/20)**2/4)**(1/2))*np.sin(x[0])
        return dxdt
    
    x0 = [0.4637, 0, 0]  # Initial state
    x = odeint(car, x0, [t])

    forward_speed = ((np.cos(t/10)**2 + np.cos(t/20)**2/4)**(1/2)) * np.cos(x[0][0])
    angular_speed = ((np.cos(t/10)**2 + np.cos(t/20)**2/4)**(1/2)) * np.sin(x[0][0])

    return forward_speed, angular_speed

def differential_drive_robot(t):
    # 计算路径参数
    X = 2 * np.sin(t/1)
    Y = 2 * np.sin(t/2)

    # 计算速度控制参数
    dx_dt = 2/1 * np.cos(t/1)  # X 的导数
    dy_dt = 2/2 * np.cos(t/2)  # Y 的导数

    # 计算前进速度
    forward_speed = np.sqrt(dx_dt**2 + dy_dt**2)

    # # 计算转向角速度
    # if t == 0:
    #     angular_speed = 0
    # else:
    #     delta_t = 0.01  # 时间步长
    #     dx_dt_prev = 3/1 * np.cos((t-delta_t)/1)
    #     dy_dt_prev = 3/2 * np.cos((t-delta_t)/2)
    #     dX_dt = (dx_dt - dx_dt_prev) / delta_t
    #     dY_dt = (dy_dt - dy_dt_prev) / delta_t
    #     angular_speed = (dY_dt * dx_dt - dX_dt * dy_dt) / (dx_dt**2 + dy_dt**2)

    delta_t = 0.01  # 时间步长
    x_next = 2 * np.sin((t+delta_t)/1)
    y_next = 2 * np.sin((t+delta_t)/2)
    x_now = 2* np.sin(t/1)
    y_now = 2* np.sin(t/2)
    d_w = math.atan2(y_next-y_now,x_next-x_now)
    angular_speed = d_w

    return forward_speed, angular_speed

speed = .2
turn = 1

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('plan')
    # cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_pub = rospy.Publisher('/mbot1_vel', Twist, queue_size=10)

    linear_speed = 0.2  # m/s
    angular_speed = 0.5  # rad/s
    duration = 12

    loop_rate = rospy.Rate(50)
    msg = Twist()

    times = 0
    speed = [0] * 6
    stop_flag = 0
    
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(str(msg))
        while not rospy.is_shutdown():
            key = getKey()
            # print("get_key:", key)
            if key == 's' or key == 'q':
                stop_flag = 1
                print("get_key:", key, ",", stop_flag)
            elif key == 'g':
                stop_flag = 0
                print("get_key:", key, ",", stop_flag)

            if stop_flag == 0:
                if times < 88:
                    forward_speed, angular_speed = car_model(0.1*times)
                    msg.linear.x = forward_speed
                    msg.angular.z = angular_speed
               
                else :
                    forward_speed, angular_speed = car_model(0.1*(times-88))
                    msg.linear.x = forward_speed
                    msg.angular.z = -angular_speed
                cmd_pub.publish(msg)
                times += 1
            else:
                msg.linear.x = 0
                msg.angular.z = 0
                cmd_pub.publish(msg)

            # print(times)

            time.sleep(0.1)

            if times > 87*2:
                times = 0
            
    except KeyboardInterrupt:  # 捕获Ctrl+C信号
        print("KeyboardInterrupt, Exiting...")
        print("err")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        cmd_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)




#! /usr/bin/env python
import pygame
from pygame.locals import *
import rospy
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import tf.transformations

vel = Vector3()
pos = Point()
cmd = PositionCommand()
odom = Odometry()
cur_pos = [0]*4
target_pos = [0]*3
target_vel = [0]*3

init = False

def odomCallbck(msg):
    global init
    if not init:
        init = True
        target_pos[2] = msg.pose.pose.position.z

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    # 将四元数转换为欧拉角
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # 解包欧拉角
    roll, pitch, yaw = euler

    cur_pos[0] = msg.pose.pose.position.x
    cur_pos[1] = msg.pose.pose.position.y
    cur_pos[2] = msg.pose.pose.position.z
    cur_pos[3] = yaw


def main():
    # param
    move_speed = 2
    move_step = 0.2
    yaw_speed = math.pi * 2
    # ros node init
    rospy.init_node('keyboard_control')
    rospy.Subscriber('/quad_0/lidar_slam/odom', Odometry, odomCallbck)
    pos_cmd_pub = rospy.Publisher("planning/pos_cmd", PositionCommand, queue_size=50)

    key_name = [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d]
    key_axes = [0]*8 #key_value
    last_key_axes = [0]*8
    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 750, 272)
    screen = pygame.display.set_mode(window_size.size)
    img = pygame.image.load("../keyboard_control/files/keyboard_control.png")
    screen.blit(img, (1,1))
    pygame.display.flip()
    while not init:
        rospy.logwarn("waitting odometry")
        
    rospy.sleep(3)
    rospy.logwarn("keyboard_control load sucessfully")

    while not rospy.is_shutdown():
        rospy.sleep(0.005) # 20Hz
        for event in pygame.event.get():
            # 按键按下
            if event.type == pygame.KEYDOWN: 
                for i in range(8):
                    if event.key == key_name[i]:
                        key_axes[i] = 1
                        break
            # 按键松开
            elif event.type == pygame.KEYUP:
                for i in range(8):
                    if event.key == key_name[i]:
                        print("key_up")
                        key_axes[i] = 0
                        break

        send_msg = False
        # direct = 1
        # for i in range(6):
        #     if key_axes[i] == 1:
        #         target_pos[int(i/2)] = cur_pos[int(i/2)] + move_step*direct
        #         target_vel[int(i/2)] = move_speed*direct
        #         send_msg = True
        #     elif last_key_axes[i] == 1 and key_axes[i] == 0:
        #         print("stop")
        #         target_pos[int(i/2)] = cur_pos[int(i/2)]
        #         target_vel[int(i/2)] = 0
        #         send_msg = True
        #     else:
        #         target_pos[int(i/2)] = cur_pos[int(i/2)]
        #     direct = -direct
        target_yaw = cur_pos[3]
        target_yaw_dot = 0

        if key_axes[4] == 1: # pygame.K_w
            target_pos[0] = math.cos(cur_pos[3])*move_step + cur_pos[0]
            target_pos[1] = math.sin(cur_pos[3])*move_step + cur_pos[1]
            target_vel[0] = math.cos(cur_pos[3])*move_speed
            target_vel[1] = math.sin(cur_pos[3])*move_speed
            send_msg = True

        if key_axes[5] == 1: # pygame.K_s
            target_pos[0] = -math.cos(cur_pos[3])*move_step + cur_pos[0]
            target_pos[1] = -math.sin(cur_pos[3])*move_step + cur_pos[1]
            target_vel[0] = -math.cos(cur_pos[3])*move_speed
            target_vel[1] = -math.sin(cur_pos[3])*move_speed
            send_msg = True
        # Left
        if key_axes[6] == 1:
            target_yaw = cur_pos[3] + yaw_speed/20
            target_yaw_dot = yaw_speed
            send_msg = True
        # Right
        if key_axes[7] == 1:
            target_yaw = cur_pos[3] - yaw_speed/20
            target_yaw_dot = -yaw_speed
            send_msg = True

        if key_axes[0] == 1:
            target_pos[2] = cur_pos[2] + move_step
            target_vel[2] = move_speed
            send_msg = True

        
        if key_axes[1] == 1:
            target_pos[2] = cur_pos[2] - move_step
            target_vel[2] = -move_speed
            send_msg = True

        

        if last_key_axes[4] == 1 and key_axes[4] == 0:
            target_vel[0] = 0
            target_vel[1] = 0
            send_msg = True

        if last_key_axes[5] == 1 and key_axes[5] == 0:
            target_vel[0] = 0
            target_vel[1] = 0
            send_msg = True

        if last_key_axes[6] == 1 and key_axes[6] == 0:
            target_yaw_dot = 0
            send_msg = True

        if last_key_axes[7] == 1 and key_axes[7] == 0:
            target_yaw_dot = 0
            send_msg = True

        if last_key_axes[0] == 1 and key_axes[0] == 0:
            target_vel[2] = 0
            send_msg = True

        if last_key_axes[1] == 1 and key_axes[1] == 0:
            target_vel[2] = 0
            send_msg = True
        


        last_key_axes = key_axes.copy()

        if send_msg:
            cmd.header.stamp = rospy.Time.now()
            cmd.trajectory_id = 1
            
            cmd.position.x = target_pos[0]
            cmd.position.y = target_pos[1]
            cmd.position.z = target_pos[2]

            cmd.velocity.x = target_vel[0]
            cmd.velocity.y = target_vel[1]
            cmd.velocity.z = target_vel[2]

            cmd.yaw = target_yaw
            cmd.yaw_dot = target_yaw_dot

            pos_cmd_pub.publish(cmd) 
            print(cur_pos)
            # cmd.acceleration.x = 0
            # cmd.acceleration.y = 0
            # cmd.acceleration.z = 0
            # cmd.yaw = 0
            # cmd.yaw_dot = 0
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



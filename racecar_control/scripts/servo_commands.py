#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    L = 0.25 # vehicle length
    t = 0.08 # half tire distance
    r = 0.03 # wheel radius

    omega = data.drive.speed/r
    alpha = data.drive.steering_angle
    R = L/tan(alpha) if alpha != 0 else 0
    rho = tan(alpha) * t/L

    lomega = omega * (1 - rho)
    romega = omega * (1 + rho)

    pub_vel_left_rear_wheel.publish(lomega)
    pub_vel_left_front_wheel.publish(lomega)

    pub_vel_right_rear_wheel.publish(romega)
    pub_vel_right_front_wheel.publish(romega)

    if R == 0:
        lsteer = 0
        rsteer = 0
    else:
        lsteer = atan(L/(R-t))
        rsteer = atan(L/(R+t))

    pub_pos_left_steering_hinge.publish(lsteer)
    pub_pos_right_steering_hinge.publish(rsteer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass

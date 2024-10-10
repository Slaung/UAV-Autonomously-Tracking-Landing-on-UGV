#! /usr/bin/env python2

import rospy

class PID_Controller_Aruco_Marker:
    def __init__(self):
        self.x0_err    = 0 
        self.y0_err    = 0
        self.x_err     = 0
        self.y_err     = 0
        self.z_err     = 0
        self.x_err_err = 0
        self.y_err_err = 0
        self.x_integral = 0
        self.y_integral = 0
        self.x_vel     = 0
        self.y_vel     = 0
        self.z_vel     = 0
        self.kp_x      = 0.002
        self.kp_y      = 0.002
        self.kp_z      = 0.1
        self.kd_x      = 0.001
        self.kd_y      = 0.001
        self.ki_x      = 0.001
        self.ki_y      = 0.001

        ### Car1_vel=60 ###
        # kp_x, kp_y = 0.002, area > 9000 success

    def speed_controller(self, pos_x, pos_y, x_gain, y_gain):
        self.kp_x = x_gain
        self.kp_y = y_gain

        # Test UAV:N / Cam:S
        self.x_err = abs(320 - pos_x) 
        self.y_err = abs(pos_y - 240)
        
        # Test N
        #self.x_err = pos_x - 320
        #self.y_err = 240 - pos_y

        # Test E
        #self.x_err = 240 - pos_y
        #self.y_err = 320 - pos_x

        # Test S
        #self.x_err = 320 - pos_x
        #self.y_err = pos_y - 240

        # Test W
        #self.x_err = pos_y - 240
        #self.y_err = pos_x - 320
        
        # X,Y 0.37~0.85
        # 302,181/284,408
        # if(0.37 <= (pos_x / 640) <= 0.85 and 0.37 <= (pos_y / 480) <= 0.85):
        #     self.z_err = -1
        # else:
        #     self.z_err = 0
        self.z_err = -1

        ### Integral ###
        self.x_integral += self.x_err
        self.y_integral += self.y_err

        ### Derivative ###
        self.x_err_err = self.x_err - self.x0_err
        self.y_err_err = self.y_err - self.y0_err
        self.x0_err = self.x_err
        self.y0_err = self.y_err

        ### P control ###
        self.x_vel = self.kp_x * self.x_err
        self.y_vel = self.kp_y * self.y_err
        self.z_vel = self.kp_z * self.z_err

        ### PD control ###
        # self.x_vel = self.kp_x * self.x_err + self.kd_x * self.x_err_err
        # self.y_vel = self.kp_y * self.y_err + self.kd_y * self.y_err_err
        # self.z_vel = self.kp_z * self.z_err

        ### PID control ###
        # self.x_vel = self.kp_x * self.x_err + self.ki_x * self.x_integral + self.kd_x * self.x_err_err
        # self.y_vel = self.kp_y * self.y_err + self.ki_y * self.y_integral + self.kd_y * self.y_err_err
        # self.z_vel = self.kp_z * self.z_err

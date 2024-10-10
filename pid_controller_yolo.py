#! /usr/bin/env python2

import rospy

class PID_Controller_Yolo:
    def __init__(self):
        self.x0_err    = 0 
        self.y0_err    = 0
        self.z0_err    = 0
        self.x_err     = 0
        self.y_err     = 0
        self.z_err     = 0
        self.x_err_err = 0
        self.y_err_err = 0
        self.z_err_err = 0
        self.x_vel     = 0
        self.y_vel     = 0
        self.z_vel     = 0
        self.kp_x      = 0.0045 #Test4: 0.0001, Test5:0.0015, Test6: 0.0025
        self.kp_y      = 0.0045 #Test4: 0.0001, Test5:0.0015, Test6:0.0025
        self.kp_z      = 0.1 #Test4: 0.05, Test5: 0.075


    def local_pos_control(self, pos_x, pos_y, pos_z):
        
        # Test UAV:N / Cam:S
        self.x_err = 320 - pos_x 
        self.y_err = pos_y -240
        
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

        if(pos_z >= 1):
            self.z_err = 0 - pos_z
        elif(pos_y <= 260 and pos_y >=220 and pos_x <= 340 and pos_x >= 300):
            self.z_err = 0 - pos_z
        else:
            self.z_err = 0

        if(pos_z <= 2):
            self.kp_x = 0.0025
            self.kp_y = 0.0025
        elif(pos_z <= 1):
            self.kp_x = 0.0005
            self.kp_y = 0.0005

        self.x_err_err = self.x_err - self.x0_err
        self.y_err_err = self.y_err - self.y0_err
        self.z_err_err = self.z_err - self.z0_err

        self.x0_err = self.x_err
        self.y0_err = self.y_err
        self.z0_err = self.z_err

        self.x_vel = self.kp_x * self.x_err 
        self.y_vel = self.kp_y * self.y_err 
        self.z_vel = self.kp_z * self.z_err

        

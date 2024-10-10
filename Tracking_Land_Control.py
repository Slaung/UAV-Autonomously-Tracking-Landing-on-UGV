#! /usr/bin/env python

import rospy
import cv2 as cv
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import Image, NavSatFix
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from marker_detection import MarkerDetector
from pid_controller_yolo import PID_Controller_Yolo
from pid_controller_aruco_marker import PID_Controller_Aruco_Marker
from fuzzy_controller_x_gain import Fuzzy_Controller_X_Gain
from fuzzy_controller_y_gain import Fuzzy_Controller_Y_Gain
from std_msgs.msg import String, Float64

########## Init parameter ##########
gnd_gps = None
air_gps = None
air_altitude_initial = 700
rotation_target_yaw_angle = 360.0

########## Callback ##########
##### System state #####
current_state = State()
def state_callback(msg):
    global current_state
    current_state = msg

##### Ground gps state #####
# def Gnd_gps_callback(msg):
#     global gnd_gps
#     gnd_gps = msg.latitude, msg.longitude, msg.altitude

##### Air gps state #####
def Air_gps_callback(msg):
    global air_gps
    air_gps = msg.latitude, msg.longitude, msg.altitude

##### Darknet bounding box #####
bbox_state = BoundingBoxes()
def bbox_callback(msg):
    global bbox_state
    bbox_state = msg

##### Anaconda height predictor #####
anaconda_callback_string = ""
def anaconda_callback(data):
    global anaconda_callback_string
    anaconda_callback_string = data.data

##### Aruco_x #####
aruco_x_callback_string = ""
def aruco_x_callback(data):
    global aruco_x_callback_string
    aruco_x_callback_string = data.data

##### Aruco_y #####
aruco_y_callback_string = ""
def aruco_y_callback(data):
    global aruco_y_callback_string
    aruco_y_callback_string = data.data

##### Aruco_area #####
aruco_area_callback_string = ""
def aruco_area_callback(data):
    global aruco_area_callback_string
    aruco_area_callback_string = data.data

##### Aruco_id #####
aruco_id_callback_string = ""
def aruco_id_callback(data):
    global aruco_id_callback_string
    aruco_id_callback_string = data.data

##### Rotation yaw #####
def yaw_callback(msg):
    global current_yaw
    current_yaw = msg.data

########## Controller ##########
##### Guided control #####
def guided_control():
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'GUIDED'
    if(set_mode_client.call(offb_set_mode).mode_sent == True):
        print("GUIDED enabled")

##### Arm control #####
def unlock_control():
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    if(arming_client.call(arm_cmd).success == True):
        print("Vehicle unlock")

def lock_control():
    rate = rospy.Rate(10)  # 10 Hz
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = False
    rospy.loginfo("Vehicle lock")
    for i in range(100):   
        if(arming_client.call(arm_cmd).success == True):
            rate.sleep()

##### Take off control #####
def takeoff_control():
    rate = rospy.Rate(20)  # 10 Hz
    takeoff_cmd = CommandTOLRequest()
    takeoff_cmd.altitude = 3
    # takeoff_cmd.yaw = 180
    if(takeoff_client.call(takeoff_cmd)):
       print("Takeoff Success")
    
    ### wait take off ###
    for i in range(140):   
        rate.sleep()

##### Land control #####
def land_control():
    ### Land ###
    land_cmd = CommandTOLRequest()
    if(land_client.call(land_cmd)):
        print("Trying to land")

##### Rotation control #####
def control_yaw(target_yaw):
    pos_target = PositionTarget()
    pos_target.header.stamp = rospy.Time.now()
    pos_target.coordinate_frame = PositionTarget.FRAME_BODY_NED
    pos_target.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.FORCE |PositionTarget.IGNORE_YAW

    if(current_yaw >= 0 and current_yaw < 6):
        pos_target.yaw_rate = 0
    elif(current_yaw >= 7 and current_yaw < 10):
        pos_target.yaw_rate = 0.1
    elif(current_yaw >= 10 and current_yaw < 45):
        pos_target.yaw_rate = 0.2
    elif(current_yaw >= 45 and current_yaw < 90):
        pos_target.yaw_rate = 0.5
    elif(current_yaw >= 90 and current_yaw < 180):
        pos_target.yaw_rate = 1
    elif(current_yaw >= 180 and current_yaw < 360):
        pos_target.yaw_rate = 2

    local_raw_pub.publish(pos_target)
    print("Current yaw angle:", current_yaw)

##### Linear speed control #####
def linear_speed_control(x, y, z):
    speed = Twist()
    speed.linear.x = x
    speed.linear.y = y
    speed.linear.z = z
    velocity_pub.publish(speed)

##### Land control #####
def control_drone(roll, pitch, yaw, throttle):
    rate = rospy.Rate(10)  # 10 Hz
    for i in range(100):   
        # Publish attitude control commands
        attitude_msg = AttitudeTarget()
        attitude_msg.header.stamp = rospy.Time.now()
        attitude_msg.orientation = Quaternion(0, 0, 0, 1)  # Don't control orientation
        attitude_msg.body_rate.x = roll  # Desired roll rate (rad/s)
        attitude_msg.body_rate.y = pitch  # Desired pitch rate (rad/s)
        attitude_msg.body_rate.z = yaw  # Desired yaw rate (rad/s)
        attitude_msg.type_mask = 128  # Ignore orientation
        attitude_msg.thrust = throttle
        attitude_pub.publish(attitude_msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Tracking_Land_Control_py")
    
    ##### Topic definition #####
    ### Darknet ###
    bbox_sub = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callback = bbox_callback)
    
    ### Aruco connect ###
    aruco_x_sub = rospy.Subscriber("aruco_x", String, callback = aruco_x_callback)
    
    aruco_y_sub = rospy.Subscriber("aruco_y", String, callback = aruco_y_callback)
    
    aruco_area_sub = rospy.Subscriber("aruco_area", String, callback = aruco_area_callback)
    
    aruco_id_sub = rospy.Subscriber("aruco_id", String, callback = aruco_id_callback)

    ### Ros and Anaconda connect ###
    ros_detection_area_pub = rospy.Publisher("ros_detection_area", String, queue_size = 10)
    
    anaconda_predict_height_sub = rospy.Subscriber("anaconda_predict_height", String, callback = anaconda_callback)

    ### Mavros ###
    state_sub = rospy.Subscriber("/mavros/state", State, callback = state_callback)
    
    #global_ground_pos_sub = rospy.Subscriber("/ugv/mavros_ugv/global_position/global", NavSatFix, Gnd_gps_callback)
    
    global_air_pos_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, Air_gps_callback)
    
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 10)
    
    local_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size = 10)
    rospy.wait_for_service("/mavros/cmd/arming")
    
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.wait_for_service("/mavros/cmd/takeoff")
    
    takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
    rospy.wait_for_service("/mavros/cmd/land")
    
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)   
    
    #local_sub = rospy.Subscriber("mavros/local_position/pose" , PoseStamped, callback = local_callback)
    
    yaw_sub = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, yaw_callback)
    
    attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    
    ##### Rate set #####
    delay_rate = rospy.Rate(20)
    detection_rate = rospy.Rate(5)

    ### Wait for Flight Controller connection ##
    while(not rospy.is_shutdown() and not current_state.connected):
        delay_rate.sleep()

    ##### Initial #####
    ### Get init air altitude ###
    # while True: 
    #     if air_gps is not None:
    #         air_altitude_initial = air_gps[2]
    #         rospy.loginfo("air_init_gps: ", air_altitude_initial)
    #         break
    #     else:
    #         rospy.loginfo("Not get air_init_gps")
    #     delay_rate.sleep()
    
    ### PID controller ###
    Yolo_PID = PID_Controller_Yolo()
    Aruco_PID = PID_Controller_Aruco_Marker()

    ### Fuzzy controller for aruco marker ###
    fuzzy_controller_x_gain = Fuzzy_Controller_X_Gain() 
    fuzzy_controller_y_gain = Fuzzy_Controller_Y_Gain() 

    ### Flight log ###
    t_yaw_log = []
    t_track_log = []
    t_log = []
    current_yaw_log = []
    bbox_x_log = []
    bbox_y_log = []
    bbox_area_log = []
    aruco_x_log = []
    aruco_y_log = []
    e_aruco_x_log = []
    e_aruco_y_log = []
    delta_aruco_x_log = []
    delta_aruco_y_log = []
    aruco_area_log = []
    yolo_x_speed_log = []
    yolo_y_speed_log = []
    yolo_z_speed_log = []
    x_speed_log = []
    y_speed_log = []
    z_speed_log = []
    yolo_drone_latitude_log = []
    yolo_drone_longitude_log = []
    yolo_drone_altitude_log = []
    drone_latitude_log = []
    drone_longitude_log = []
    drone_altitude_log = []
    pred_h_log = []
    controller_log = [] # 0:Aruco / 1:YOLO / 2:UP

    ### Control parameter ###
    yaw_count = 0
    track_count = 0
    Count = 0
    Bbox_x = 0
    Bbox_y = 0
    Bbox_area = 0
    Bbox_n1_area = 0
    Aruco_x = 0
    Aruco_y = 0
    Aruco_area = 0
    Aruco_id = -1
    last_Aruco_x = 0
    last_Aruco_y = 0
    GPS_arrive = 0
    Anaconda_height = 5 # init
    anaconda_callback_string = ""
    Track_check = 0 # 0:not arrive / 1:already arrive
    Land_check = 0 # 0:not land / 1:already land

    ##### Takeoff controller #####
    ### GUIDED ###
    guided_control()

    ### Unlock ###
    unlock_control()

    ### Take off ###
    takeoff_control()

    ### Velocity controller ###
    # linear_speed_control(0, 0, 0)

    ##### Rotation controller #####
    rospy.loginfo("Rotation control...")
    while(not rospy.is_shutdown()):
        try:
            control_yaw(rotation_target_yaw_angle)
        except rospy.ROSInterruptException:
            pass
        if(current_yaw >= 0 and current_yaw < 5):
            break

        ### Update log ###
        t_yaw_log.append(yaw_count)
        current_yaw_log.append(current_yaw)
        yaw_count += 1

        detection_rate.sleep()

    ##### Tracking controller #####
    while((not rospy.is_shutdown()) and Track_check == 0):
        ### YOLO detection ###
        if(len(bbox_state.bounding_boxes) > 0):
            ### Bbox image ###
            Bbox_x = (bbox_state.bounding_boxes[0].xmax - bbox_state.bounding_boxes[0].xmin)/2 + bbox_state.bounding_boxes[0].xmin
            Bbox_y = (bbox_state.bounding_boxes[0].ymax - bbox_state.bounding_boxes[0].ymin)/2 + bbox_state.bounding_boxes[0].ymin
            Bbox_area = (bbox_state.bounding_boxes[0].xmax - bbox_state.bounding_boxes[0].xmin) * (bbox_state.bounding_boxes[0].ymax - bbox_state.bounding_boxes[0].ymin)

            ### Anaconda: height predictor ###
            ros_publish_string = str(Bbox_area)
            ros_detection_area_pub.publish(ros_publish_string)

            ### Wait anaconda inference ###
            if(anaconda_callback_string != ""):
                Anaconda_height =  float(anaconda_callback_string)
            
            ##### PID-Controller of tracking #####
            if(Bbox_n1_area != 0 and Bbox_n1_area == Bbox_area):
                print("YOLO lost detection")
            else:
                ### YOLO PID controller ###
                Yolo_PID.local_pos_control(Bbox_x, Bbox_y, Anaconda_height)

                ### Velocity control ###
                linear_speed_control(Yolo_PID.x_vel, Yolo_PID.y_vel, Yolo_PID.z_vel)
                print("!!!YOLO Controller!!!")
        else:
            print("YOLO failed detection")
        
        ### Aruco marker detection ###
        if(aruco_id_callback_string != ""):
            Aruco_x = int(aruco_x_callback_string)
            Aruco_y = int(aruco_y_callback_string)
            Aruco_area = int(aruco_area_callback_string)
            Aruco_id = int(aruco_id_callback_string)
            if(Aruco_id == 23 and Aruco_area > 1000):
                Track_check = 1

        ##### Show status and update log #####
        ### Update Bbox ###
        Bbox_n1_area = Bbox_area

        ### print ###
        print("t: ", track_count)
        print("Bbox(x, y): (", Bbox_x, ", ", Bbox_y, ")", sep='')
        print("Bbox area: ", Bbox_area)
        print("Aruco_area: ", Aruco_area)
        if(Bbox_n1_area != 0 and Bbox_n1_area == Bbox_area):
            print("yolo_speed(x, y, z): (", "{:.6f}".format(Yolo_PID.x_vel), ", ", "{:.6f}".format(Yolo_PID.y_vel), ", ", "{:.6f}".format(Yolo_PID.z_vel), ")", sep='')
        print("drone(lat, lon, alt): (", air_gps[0], ", ", air_gps[1], ", ",air_gps[2], ")", sep='')
        print("pred_h: ", anaconda_callback_string)

        ### Update log ###
        t_track_log.append(track_count)
        bbox_x_log.append(Bbox_x)
        bbox_y_log.append(Bbox_y)
        bbox_area_log.append(Bbox_area)
        if(Bbox_n1_area != 0 and Bbox_n1_area == Bbox_area):
            yolo_x_speed_log.append(Yolo_PID.x_vel)
            yolo_y_speed_log.append(Yolo_PID.y_vel)
            yolo_z_speed_log.append(Yolo_PID.z_vel)
        else:
            yolo_x_speed_log.append(0)
            yolo_y_speed_log.append(0)
            yolo_z_speed_log.append(0)
        yolo_drone_latitude_log.append(air_gps[0])
        yolo_drone_longitude_log.append(air_gps[1])
        yolo_drone_altitude_log.append(air_gps[2])
        pred_h_log.append(Anaconda_height)

        ##### Clear #####
        Bbox_x = 0
        Bbox_y = 0
        Bbox_area = 0
        Aruco_x = 0
        Aruco_y = 0
        Aruco_area = 0

        track_count += 1
        detection_rate.sleep()

    ##### Land controller #####
    while((not rospy.is_shutdown()) and Land_check == 0):
        ### Aruco marker detection ###
        if(aruco_id_callback_string != ""):
            Aruco_x = int(aruco_x_callback_string)
            Aruco_y = int(aruco_y_callback_string)
            Aruco_area = int(aruco_area_callback_string)
            Aruco_id = int(aruco_id_callback_string)
            print("Marker ID=23")
        else:
            print("Aruco failed detection")

        ##### PID-Controller of landing #####
        if(Aruco_id == 23):
            ### Fuzzy-PID controller ###
            if(last_Aruco_x == 0 and last_Aruco_y == 0):
                fuzzy_controller_x_gain.inference(320 - Aruco_x, 0)
                fuzzy_controller_y_gain.inference(Aruco_y - 240, 0)
            else:
                fuzzy_controller_x_gain.inference(320 - Aruco_x, Aruco_x - last_Aruco_x)
                fuzzy_controller_y_gain.inference(Aruco_y - 240, Aruco_y - last_Aruco_y)

            ### Aruco PID controller ###
            Aruco_PID.speed_controller(Aruco_x, Aruco_y, fuzzy_controller_x_gain.gain, fuzzy_controller_y_gain.gain)

            ### Land condition ###
            if(Aruco_area > 7500 and 0.37 <= (Aruco_x / 640) <= 0.85 and 0.37 <= (Aruco_y / 480) <= 0.85):
                ### Land mode for static vehicle ###
                # land_control()
                
                ### Attitude land ###
                print("Landing...")  
                linear_speed_control(0, 0, -0.2)
                control_drone(0, 0, 0, 0)
                
                ### Lock ###
                print("Vehicle lock")
                lock_control()
                Land_check = 1

            ### Velocity control ###
            linear_speed_control(Aruco_PID.x_vel, Aruco_PID.y_vel, Aruco_PID.z_vel)
            print("!!!Aruco Controller!!!")
        else:
            ### Velocity control ###
            linear_speed_control(0, 0, 0.2)
            print("!!!Up Controller!!!")
       

        ##### Show status and update log #####
        ### Print status ###
        print("t: ", Count)
        print("Aruco(x, y): (", Aruco_x, ", ", Aruco_y, ")", sep='')
        print("e_Aruco(x, y): (", 320 - Aruco_x, ", ", Aruco_y - 240, ")", sep='')
        if(last_Aruco_x == 0 and last_Aruco_y == 0):
            print("delta_Aruco(x, y): (", 0, ", ", 0, ")", sep='')
        else:
            print("delta_Aruco(x, y): (", Aruco_x - last_Aruco_x, ", ", Aruco_y - last_Aruco_y, ")", sep='')
        print("Aruco_area: ", Aruco_area)
        if(Aruco_id == 23):
            print("aruco_speed(x, y, z): (", "{:.6f}".format(Aruco_PID.x_vel), ", ", "{:.6f}".format(Aruco_PID.y_vel), ", ", "{:.6f}".format(Aruco_PID.z_vel), ")", sep='')
            print("aruco_gain(x, y): (", "{:.6f}".format(fuzzy_controller_x_gain.gain), ", ", "{:.6f}".format(fuzzy_controller_y_gain.gain), ")", sep='')
        else:
            print("up_speed(x, y, z): (0, 0, 0.5)")
        print("drone(lat, lon, alt): (", air_gps[0], ", ", air_gps[1], ", ",air_gps[2], ")", sep='')
        
        
        ### Update log ###
        t_log.append(Count)
        aruco_x_log.append(Aruco_x)
        aruco_y_log.append(Aruco_y)
        e_aruco_x_log.append(320 - Aruco_x)
        e_aruco_y_log.append(Aruco_y - 240)
        delta_aruco_x_log.append(Aruco_x - last_Aruco_x)
        delta_aruco_y_log.append(Aruco_y - last_Aruco_y)
        aruco_area_log.append(Aruco_area)
        if(Aruco_id == 23):
            x_speed_log.append(Aruco_PID.x_vel)
            y_speed_log.append(Aruco_PID.y_vel)
            z_speed_log.append(Aruco_PID.z_vel)
            controller_log.append(0)
        else:
            x_speed_log.append(0)
            y_speed_log.append(0)
            z_speed_log.append(0.2)
            controller_log.append(2)
        drone_latitude_log.append(air_gps[0])
        drone_longitude_log.append(air_gps[1])
        drone_altitude_log.append(air_gps[2])
        

        ##### Update #####
        if(aruco_id_callback_string != ""):
            last_Aruco_x = Aruco_x
            last_Aruco_y = Aruco_y

        ##### Clear #####
        Aruco_x = 0
        Aruco_y = 0
        Aruco_area = 0

        Count += 1
        detection_rate.sleep()
    
    ##### Write log #####   
    current_time = rospy.Time.now()
    file_name = "log_{}.txt".format(current_time)

    with open(file_name, 'w') as file:
        for i in range(yaw_count):
            file.write("t_yaw: {}\n".format(t_yaw_log[i]))
            file.write("current_yaw: {}\n".format(current_yaw_log[i]))

        for i in range(track_count):
            file.write("t_track: {}\n".format(t_track_log[i]))
            file.write("bbox_x: {}\n".format(bbox_x_log[i]))
            file.write("bbox_y: {}\n".format(bbox_y_log[i]))
            file.write("yolo_area: {}\n".format(bbox_area_log[i]))
            file.write("x_speed: {}\n".format(yolo_x_speed_log[i]))
            file.write("y_speed: {}\n".format(yolo_y_speed_log[i]))
            file.write("z_speed: {}\n".format(yolo_z_speed_log[i]))
            file.write("drone_latitude: {}\n".format(yolo_drone_latitude_log[i]))
            file.write("drone_longitude: {}\n".format(yolo_drone_longitude_log[i]))
            file.write("drone_altitude: {}\n".format(yolo_drone_altitude_log[i]))
            file.write("pred_h: {}\n".format(pred_h_log[i]))
        for i in range(Count):
            file.write("t: {}\n".format(t_log[i]))
            file.write("aruco_x: {}\n".format(aruco_x_log[i]))
            file.write("aruco_y: {}\n".format(aruco_y_log[i]))
            file.write("e_aruco_x: {}\n".format(e_aruco_x_log[i]))
            file.write("e_aruco_y: {}\n".format(e_aruco_y_log[i]))
            file.write("delta_aruco_x: {}\n".format(delta_aruco_x_log[i]))
            file.write("delta_aruco_y: {}\n".format(delta_aruco_y_log[i]))
            file.write("aruco_area: {}\n".format(aruco_area_log[i]))
            file.write("x_speed: {}\n".format(x_speed_log[i]))
            file.write("y_speed: {}\n".format(y_speed_log[i]))
            file.write("z_speed: {}\n".format(z_speed_log[i]))
            file.write("drone_latitude: {}\n".format(drone_latitude_log[i]))
            file.write("drone_longitude: {}\n".format(drone_longitude_log[i]))
            file.write("drone_altitude: {}\n".format(drone_altitude_log[i]))
            file.write("controller: {}\n".format(controller_log[i]))
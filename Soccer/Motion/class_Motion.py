#  Walking engine for Starkit Roki on RPI
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json
import numpy as np
import starkit
from math import pi
from Robots.roki2met import roki2met
from Soccer.rotation_math import Rotation
from Robots.class_Robot_Roki_2 import Robot
from Soccer.Motion.class_Motion_extention_1 import Motion_extention_1
from Soccer.config_paths import init_param_read_path

from Soccer.Motion.path_planning import PathPlan

def uprint(*text):
    #with open(current_work_directory + "Soccer/log/output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class Motion(Robot, Motion_extention_1):

    def __init__(self, glob, vision):
        super().__init__()
        self.glob = glob
        self.params = self.glob.params
        self.glob.motion = self

        self.MOTION_SLOT_DICT = {0:['',0], 1:['',0], 2:['',0], 3:['',0], 4:['',0], 5:['Get_Up_Inentification',7000],
                    6:['Soccer_Get_UP_Stomach_N', 5000], 7:['Soccer_Get_UP_Face_Up_N', 5000], 8:['',0], 9:['',0], 10:['',0],
                    11:['',0], 12:['',0], 13:['',0], 14:['',0], 15:['',0],
                    16:['',0], 17:['',0], 18:['Soccer_Kick_Forward_Right_Leg',5000], 19: ['Soccer_Kick_Forward_Left_Leg',5000], 20:['',0],
                    21:['Get_Up_From_Defence',1000], 22:['',0], 23:['PanaltyDefenceReady_Fast',500], 24:['PenaltyDefenceF',300], 25:['Zummer',0],
                    26:['Soccer_Speed_UP',0], 27:['',0], 28:['',0], 29:['',0], 30:['',0],
                    31:['',0], 32:['',0], 33: ['',0], 34:['',0],
                    35: ['',0], 36: ['PenaltyDefenceR',2000], 37: ['PenaltyDefenceL',2000]}
        self.TIK2RAD = 0.00058909
        self.slowTime   = 0.0             # seconds
        if self.glob.SIMULATION == 0: self.slowTime   = 0.5
        self.simThreadCycleInMs = 20
        self.frame_delay = self.glob.params['FRAME_DELAY']
        self.frames_per_cycle = self.glob.params['FRAMES_PER_CYCLE']
        self.motion_shift_correction_x = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
        self.motion_shift_correction_y = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
        self.first_step_yield = self.glob.first_step_yield
        self.cycle_step_yield = self.glob.cycle_step_yield
        self.side_step_right_yield = self.glob.side_step_right_yield
        self.side_step_left_yield = self.glob.side_step_left_yield
        self.imu_drift_speed = math.radians(self.glob.params['IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT'])/ 360
        self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg = True
        self.initPoses = 400//self.simThreadCycleInMs
        self.exitFlag = 0
        self.falling_Flag = 0
        self.neck_pan = 0
        self.neck_tilt = 0
        self.old_neck_pan = 0
        self.body_euler_angle ={}
        self.local = 0 # Local
        self.vision = vision
        self.with_Vision = True
        self.p = PathPlan(self.glob)
        self.old_neck_tilt = 0
        self.direction_To_Attack = 0
        self.activePose = []
        self.xtr = 0
        self.ytr = -self.d10   #-53.4
        self.ztr = -self.gaitHeight
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10   # 53.4
        self.ztl = -self.gaitHeight
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.tors_angle = 0
        self.fase_offset = 0.7
        self.xtl0 = 0
        self.xtr0 = 0
        self.dx0_typical = 0
        self.dy0_typical = 0
        self.alpha01 = 0
        self.lateral_offset = 0
        self.ugol_torsa = 0.4
        self.dobavka_x_ot_torsa = 0
        self.euler_angle = {}
        self.robot_In_0_Pose = True
        self.keep_hands_up = False
        self.kick_power = 100       # from 0 to 100
        self.motion_slot_progress = False
        #self.start_point_for_imu_drift = 0
        self.motions_recorded = []
        if self.glob.SIMULATION == 5 :
            #import Roki
            self.Roki = self.glob.Roki
            self.stm_channel = self.glob.stm_channel
            self.rcb = self.glob.rcb
            with open(init_param_read_path(self.glob.current_work_directory, "Real/Real_calibr.json"), "r") as f:
                data1 = json.loads(f.read())
            self.neck_calibr = data1['neck_calibr']
            self.neck_play_pose = data1['neck_play_pose']
            self.head_pitch_with_horizontal_camera = data1['head_pitch_with_horizontal_camera']
            self.neck_tilt = self.neck_calibr
        self.start_point_for_imu_drift = time.perf_counter()
        #self.focal_distance_in_pixels = self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 / math.tan(math.radians(self.params['CAMERA_APERTURE'] / 2))

    #-------------------------------------------------------------------------------------------------------------------------------
    def pause_in_ms(self, time_in_ms):
        if self.glob.SIMULATION == 5:
            time.sleep(time_in_ms / 1000)
        else: self.sim_Progress(time_in_ms/1000)

    def wait_for_gueue_end(self, with_Vision = True):
        frame_time_s = self.glob.params['FRAME_DELAY']/1000
        queue_length = self.stm_channel.mb.GetBodyQueueInfo()[1].Size
        if queue_length > 1: queue_length -= 1
        sleeping_time = queue_length * frame_time_s
        #print('wait_for_gueue_end. sleeping time: ', sleeping_time)
        if with_Vision:
            if sleeping_time > 0.1 :
                if self.glob.role == 'marathon':
                    self.glob.vision.detect_Line_Follow_One_Shot()
                else:  
                    self.glob.vision.detect_Ball_in_One_Shot()
                queue_length = self.stm_channel.mb.GetBodyQueueInfo()[1].Size
                if queue_length > 1: queue_length -= 1
                sleeping_time = queue_length * frame_time_s
        
        time.sleep(sleeping_time )
        for counter in range(1000):
            if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 2: break
            time.sleep(0.01)
        


    def imu_body_yaw(self):
        yaw = self.body_euler_angle['yaw']
        yaw = self.norm_yaw(yaw)
        return yaw

    def norm_yaw(self, yaw):
        yaw %= 2 * pi
        if yaw > pi:  yaw -= 2* pi
        if yaw < -pi: yaw += 2* pi
        return yaw

    def from_vrep_quat_to_conventional_quat(self, quaternion):
            x,y,z,w = quaternion
            return [w,x,y,z]

    def quaternion_to_euler_angle(self, quaternion):
        euler_angle = {}
        w,x,y,z = quaternion
        ysqr = y*y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0,t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3,t4))
        euler_angle['yaw'] = math.radians(Z)
        euler_angle['pitch'] = math.radians(Y)
        euler_angle['roll'] = math.radians(X)
        return euler_angle

    def push_Button(self, labels, message = "'Waiting for button'"):
        from button_test import Button_Test
        button = Button_Test(labels)
        pressed_button = button.wait_for_button_pressing(message = message)
        uprint("нажато")
        return pressed_button
        #ala = 0
        #while(ala==0):
        #    if (self.pin2.value()== 0):   # нажатие на кнопку 2 на голове
        #        ala = 1
        #        uprint("нажато")

    def play_Motion_Slot(self, name = ''):
        self.simulateMotion(name = name)

    def fill_queue_with_frames(self, frames):
        servoData =  self.Roki.Rcb4.ServoData()
        servoData.Id, servoData.Sio, servoData.Data = 30, 1, 0      # 30 - is servo ID, which is not in use
        for _ in range(frames):
            a=self.rcb.setServoPosAsync([servoData], 1, 0)           # Filling queue with empty commands in STM

    def camera_elevation(self):
        if self.motion_slot_progress: return False, 0
        self.refresh_Orientation()
        if abs(self.body_euler_angle['pitch'] ) > pi/4 or abs(self.body_euler_angle['roll'])  > pi/4  :  return False, 0
        pitch_rotation = Rotation.from_euler('y', self.body_euler_angle['pitch'], degrees=False)
        roll_rotation = Rotation.from_euler('x', self.body_euler_angle['roll'], degrees=False)
        right_leg_vector = [self.xtr, self.ytr, self.ztr]
        left_leg_vector = [self.xtl, self.ytl, self.ztl]
        right_leg_vector = pitch_rotation.apply(roll_rotation.apply(right_leg_vector))
        left_leg_vector = pitch_rotation.apply(roll_rotation.apply(left_leg_vector))
        com_2_camera_vector_z = self.params['HEIGHT_OF_CAMERA'] + self.params['HEIGHT_OF_NECK'] * (math.cos(self.neck_tilt * self.TIK2RAD) - 1 ) + self.ztr0 - 1
        com_2_camera_vector_x = self.params['HEIGHT_OF_NECK'] * math.sin(-self.neck_tilt * self.TIK2RAD) * math.cos(self.neck_pan * self.TIK2RAD)
        com_2_camera_vector_y = self.params['HEIGHT_OF_NECK'] * math.sin(-self.neck_tilt * self.TIK2RAD) * math.sin(self.neck_pan * self.TIK2RAD)
        com_2_camera_vector = [com_2_camera_vector_x, com_2_camera_vector_y, com_2_camera_vector_z]
        com_2_camera_vector = pitch_rotation.apply(roll_rotation.apply(com_2_camera_vector))
        camera_elevation = com_2_camera_vector[2] - min(right_leg_vector[2], left_leg_vector[2])
        return True, camera_elevation

    def play_Soft_Motion_Slot_(self, name = '', motion_list = None, hands_on = True):             # the slot from file will be played in robot
        print('playing : ', name) 
        self.motion_slot_progress = True
        if self.glob.SIMULATION == 5:
            if motion_list == None:
                with open(self.glob.current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
                    slots = json.loads(f.read())
                motion_list = slots[name]
            #joint_number = len(self.ACTIVESERVOS)
            with open('Slot_log.txt', "a") as log_file:
                print('playing slot', file=log_file)
                motion_num = 0
                for motion in motion_list:
                    motion_num += 1
                    print('pose number = ', motion_num, file=log_file)
                    joint_number = len(motion) -1
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number - 1 + self.ACTIVESERVOS[i][1]
                                pos = int(motion[i+1] * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(motion[i+1] * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(motion[i+1] * self.ACTIVESERVOS[i][2] + 7500) 
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            #print(i, servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data, file=log_file)
                    frames_number = int(motion[0]) 
                    a=self.rcb.setServoPosAsync(servoDatas, frames_number, frames_number-1)
                    time.sleep(self.glob.params['FRAME_DELAY']/1000 * (frames_number-1))
            self.wait_for_gueue_end(with_Vision = False)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 1: break
            #     time.sleep(0.02)
        else:
            self.simulateMotion(name = name, motion_list = motion_list, hands_on = hands_on)
        self.motion_slot_progress = False

    def play_Soft_Motion_Slot(self, name = '', motion_list = None, hands_on = True, soft_factor = 1):   # the slot from file will be played in robot
        print('playing : ', name) 
        self.motion_slot_progress = True
        if self.glob.SIMULATION == 5:
            if motion_list == None:
                with open(self.glob.current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
                    slots = json.loads(f.read())
                motion_list = slots[name]
            #joint_number = len(self.ACTIVESERVOS)
            with open('Slot_log.txt', "a") as log_file:
                print('playing slot', file=log_file)
                motion_num = 0
                for motion in motion_list:
                    motion_num += 1
                    print('pose number = ', motion_num, file=log_file)
                    joint_number = len(motion) -1
                    servoDatas = []
                    for i in range(joint_number):
                        if (self.ACTIVESERVOS[i][0] in self.hand_servo_ids) and (not hands_on): continue
                        if self.model == 'Roki_2':
                            if self.ACTIVESERVOS[i][0] == 8:
                                pos = int(motion[i+1] * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoData = self.Roki.Rcb4.ServoData()
                                servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoDatas.append(servoData)
                            else: pos = int(motion[i+1] * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                        else:
                            pos = int(motion[i+1] * self.ACTIVESERVOS[i][2] + 7500) 
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                            #print(i, servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data, file=log_file)
                    frames_number = int(motion[0])
                    frames = int(round(frames_number * soft_factor))
                    pause = int(frames_number - 1)
                    a=self.rcb.setServoPosAsync(servoDatas, frames, pause)
                    time.sleep(self.glob.params['FRAME_DELAY']/1000 * pause)
            self.wait_for_gueue_end(with_Vision = False)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 1: break
            #     time.sleep(0.02)
        else:
            self.simulateMotion(name = name, motion_list = motion_list, hands_on = hands_on)
        self.motion_slot_progress = False

    def falling_Test(self):
        self.refresh_Orientation()
        if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
            key = 0
            if self.ms.kbhit():
                key = self.ms.getch()
            if key == b'p' :
                #self.lock.acquire()
                if self.glob.SIMULATION == 3:
                    self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                key = 0
                while (True):
                    if self.ms.kbhit():
                        key = self.ms.getch()
                    if key == b'p':
                        #self.lock.release()
                        if self.glob.SIMULATION == 3:
                            self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                        key = 0
                        break
            if key == b's': # or self.transfer_Data.stop_Flag:
                uprint('Simulation STOP by keyboard')
                #self.transfer_Data.stop += 1
                self.sim_Stop()
                self.sim_Disable()
                #while True:
                #    if self.transfer_Data.stop == self.numberOfRobots:
                #        self.sim_Disable()
                #        sys.exit(0)
                #    time.sleep(0.1)
                self.falling_Flag = 3
                #self.transfer_Data.stop_Flag = True
                return self.falling_Flag
            #returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            #Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
            #self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            if (self.body_euler_angle['pitch']) > 0.785:
                self.falling_Flag = 1                   # on stomach
                self.simulateMotion(name = self.model + '_Get_UP_Stomach')
            if (self.body_euler_angle['pitch']) <  -0.785:
                self.falling_Flag = -1                  # face up
                self.simulateMotion(name = self.model + '_Get_UP_Face_Up')
            if (self.body_euler_angle['roll']) > 0.785:
                self.falling_Flag = -2                  # on right side
                self.simulateMotion(name = 'Get_Up_Right')
            if -135< (self.body_euler_angle['roll']) < -0.785:
                self.falling_Flag = 2                   # on left side
                self.simulateMotion(name = 'Get_Up_Left')
        if self.glob.SIMULATION == 5:
            if abs(self.body_euler_angle['pitch']) <= 1 and abs(self.body_euler_angle['roll']) <= 1:
                self.falling_Flag = 0
            if self.body_euler_angle['pitch'] > 1:                  # on stomach
                self.falling_Flag = 1                               # on stomach
                self.stm_channel.mb.ResetBodyQueue()                    # cleans queue of commands to controller
                self.play_Soft_Motion_Slot(name = self.model + '_Get_UP_Stomach')
                time.sleep(1)
            if self.body_euler_angle['pitch'] < -1:
                self.falling_Flag = -1                              # face up
                self.stm_channel.mb.ResetBodyQueue()                    # cleans queue of commands to controller
                self.play_Soft_Motion_Slot(name = self.model + '_Get_UP_Face_Up')
            if self.body_euler_angle['roll'] > 1: 
                self.falling_Flag = -2                              # on right side
                self.play_Soft_Motion_Slot(name = 'Get_Up_Right')
            if self.body_euler_angle['roll'] < -1:
                self.falling_Flag = 2                               # on left side
                self.play_Soft_Motion_Slot(name = 'Get_Up_Left')
        return self.falling_Flag

    def computeAlphaForWalk(self, hands_on = True ):
        angles =[]
        anglesR=[]
        anglesL=[]
        anglesR = starkit.alpha_calculation(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, self.SIZES, self.LIMALPHA)
        anglesL = starkit.alpha_calculation(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, self.SIZES, self.LIMALPHA)
        if len(anglesR)>1:
            for i in range(len(anglesR)):
                if len(anglesR)==1: break
                if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                else: anglesR.pop(0)
        elif len(anglesR)==0:
            return[]
        if len(anglesL)>1:
            for i in range(len(anglesL)):
                if len(anglesL)==1: break
                if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                else: anglesL.pop(0)
        elif len(anglesL)==0:
            return[]
        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6): angles.append(anglesR[0][j])
            if hands_on: angles.append(1.973)
            #if hands_on: angles.append(1.507 +(-0.147 - self.xtl/30))
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.147 - self.xtl/114) 
            #if hands_on: angles.append(-0.147 - self.xtl/30)
            else: angles.append(0.0)
            angles.append(0.0)
            for j in range(6): angles.append(-anglesL[0][j])
            if hands_on: angles.append(-1.973)
            #if hands_on: angles.append(-1.507 + 0.147 + self.xtr/30)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.147 + self.xtr/114)
            #if hands_on: angles.append(0.147 + self.xtr/30)
            else: angles.append(0.0)
        else:
            for j in range(6): angles.append(anglesL[0][j])
            if hands_on: angles.append(1.973)    #(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.147 - self.xtr/114)  #(0.524 - self.xtr/57.3)
            else: angles.append(0.0)
            angles.append(0.0)                                  # Tors
            for j in range(6): angles.append(-anglesR[0][j])
            if hands_on: angles.append(-1.973)  #(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.147 + self.xtl/114)  #(-0.524 + self.xtl/57.3)
            else: angles.append(0.0)
        self.activePose = angles
        if self.glob.record_motions: self.motions_recorded.append(angles)
        return angles

    def activation(self):
        if self.glob.SIMULATION == 5:
            for _ in range(10):
                result, pitch, roll, yaw = self.stm_channel.pitch_roll_yaw_from_imu_in_body()
                if result: break
                time.sleep(0.002)
            self.body_euler_angle['pitch'], self.body_euler_angle['roll'], self.body_euler_angle['yaw'] = pitch, roll, yaw
            for _ in range(10):
                result, pitch, roll, yaw = self.stm_channel.pitch_roll_yaw_from_imu_in_head()
                if result: break
                time.sleep(0.002)
            self.euler_angle['pitch'], self.euler_angle['roll'], self.euler_angle['yaw'] = pitch, roll, yaw
            self.direction_To_Attack += self.body_euler_angle['yaw']
        else:
            time.sleep(0.1)
            if self.glob.SIMULATION != 0:
                self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            Dummy_Hquaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
            self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
            #self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.direction_To_Attack += self.euler_angle['yaw']
            #uprint('body_euler_angle = ', self.body_euler_angle)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            self.Dummy_HData.append(Dummy_Hposition)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        self.direction_To_Attack = self.norm_yaw(self.direction_To_Attack)
        self.glob.imu_drift_last_correction_time = time.perf_counter()

    def walk_Initial_Pose(self, amplitude = 24, start_mixing = True):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.xtr = self.xtl = 0
        framestep = self.simThreadCycleInMs//10
        if self.glob.SIMULATION == 5 and start_mixing:
            self.rcb.motionPlay(3)
            self.wait_for_gueue_end(with_Vision = False)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 1: break
            #     time.sleep(0.02)
        for j in range (self.initPoses):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ztr = self.ztr0 - j*(self.ztr0 + self.gaitHeight)/self.initPoses
            self.ztl = self.ztl0 - j*(self.ztl0+self.gaitHeight)/self.initPoses
            # self.ytr = -self.d10 - j*self.amplitude/2 /self.initPoses
            # self.ytl =  self.d10 - j*self.amplitude/2 /self.initPoses
            self.ytr = -self.d10 - j* amplitude /self.initPoses
            self.ytl =  self.d10 - j* amplitude /self.initPoses
            angles = self.computeAlphaForWalk()
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                            returnCode = self.sim.simxSetJointPosition(self.clientID,
                                         self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                         self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        #self.sim.simxSynchronousTrigger(self.clientID)
                        self.trigger('walk_initial_pose')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        #servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        servoDatas = []
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if self.ACTIVESERVOS[i][0] == 8:
                                #n = joint_number - 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                #servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoData = self.Roki.Rcb4.ServoData()
                                servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoDatas.append(servoData)
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                            #servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        #servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        servoDatas = []
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                            #servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    if j == 0:
                        a=self.rcb.setServoPosAsync(servoDatas, 10, 9)
                    else:
                        a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)

        #if self.fr1 == 0:
        #    alpha01 = math.pi
        #else:
        #    alpha01 = math.pi/self.fr1*2
        #alpha = alpha01 * (self.fr1/2+ 1)
        #alpha_next = alpha01 * ((self.fr1 + 2)/2 + 1)
        #S = self.amplitude/2 * math.cos(alpha)
        #dS = self.amplitude/2 * math.cos(alpha_next) - S 
        #cycles = int((S + 60) / dS)
        #for j in range (cycles):
        #    self.ytr = -self.d10 - 60 + (j +1) *dS
        #    self.ytl =  self.d10 - 60 + (j +1) *dS
        #    angles = self.computeAlphaForWalk()
        #    if len(angles)==0:
        #        self.exitFlag = self.exitFlag +1
        #    else:
        #        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #            if self.glob.SIMULATION == 3: self.wait_sim_step()
        #            self.sim.simxPauseCommunication(self.clientID, True)
        #            for i in range(len(angles)):
        #                if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
        #                   returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
        #                                self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
        #                                self.sim.simx_opmode_oneshot)
        #                elif self.glob.SIMULATION == 0:
        #                    returnCode = self.sim.simxSetJointPosition(self.clientID,
        #                                 self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
        #                                 self.sim.simx_opmode_oneshot)
        #            self.sim.simxPauseCommunication(self.clientID, False)
        #            if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #                time.sleep(self.slowTime)
        #                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
        #                                      self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #                #self.Dummy_HData.append(Dummy_Hposition)
        #                #self.timeElapsed = self.timeElapsed +1
        #                # uprint(Dummy_Hposition)
        #                #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
        #                #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
        #            if self.glob.SIMULATION == 1:
        #                self.sim.simxSynchronousTrigger(self.clientID)
        #        elif self.glob.SIMULATION == 5:
        #            joint_number = len(angles)
        #            if self.model == 'Roki_2':
        #                servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
        #                for i in range(joint_number):
        #                    if self.ACTIVESERVOS[i][0] == 8:
        #                        n = joint_number - 1 + self.ACTIVESERVOS[i][1]
        #                        pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
        #                        servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
        #                    else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
        #                    servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
        #            else:
        #                servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
        #                for i in range(joint_number):
        #                    pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
        #                    servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
        #            a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)

    def walk_Cycle_v0(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        if self.rotation <= 0: 
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_RIGHT']
        else:
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_LEFT']
        if rotation > 0.125 : rotation = 0.125
        if rotation < -0.125 : rotation = -0.125
        alpha = 0
        if self.fr1 == 0:
            alpha01 = math.pi
        else:
            alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        if stepLength < 0:
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK_BACKWARDS'], self.params['BODY_TILT_AT_WALK_BACKWARDS']
        else:
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
            # counter = 0
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: 
            #         print('Sleeping time at walk cycle: ', counter * 0.02)
            #         break
            #     time.sleep(0.02)
            #     counter += 1
        fase_offset = 0.7 #1.57  #0.7
        for iii in range(0,frameNumberPerCycle,framestep):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            if 0<= iii <self.fr1 :                                              # FASA 1
                alpha = alpha01 * (iii/2+ fase_offset*framestep)
                #alpha = alpha01 * iii/2
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                alpha = alpha01 * ((iii-self.fr2)/2+ fase_offset*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                self.ztr = -self.gaitHeight + self.stepHeight
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                    self.ztr = -self.gaitHeight + self.stepHeight/2
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                self.ztl = -self.gaitHeight + self.stepHeight
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                    self.ztl = -self.gaitHeight + self.stepHeight/2
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            angles = self.computeAlphaForWalk()
            #print('iii = ', iii, 'xtr =', round(self.xtr,2), 'xtl =', round(self.xtl, 2))
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                print('bad_ik_calc:', 'iii = ', iii, 'xtr:', self.xtr, 'ytr:', self.ytr, 'ztr:', self.ztr, 'xtl:', self.xtl, 'ytl:', self.ytl, 'ztl:', self.ztl )
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        #uprint(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            self.trigger('walk_cycle_v0')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    servoDatas = []
                    if self.model == 'Roki_2':
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if self.ACTIVESERVOS[i][0] == 8:
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoData = self.Roki.Rcb4.ServoData()
                                servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoDatas.append(servoData)
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
                    #print('calc time =',time1 - time2, 'transfer time =', time2 )
                    time1 = time.perf_counter() - start1
                    #time.sleep(self.frame_delay/1000 - time1)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        if self.glob.with_Local:
            self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
            if self.first_Leg_Is_Right_Leg:
                self.local.coord_shift[1] = -self.side_step_right_yield * sideLength/20/1000
            else: self.local.coord_shift[1] = self.side_step_left_yield * sideLength/20/1000
            self.local.coordinate_record(odometry = True, shift = True)
            self.local.refresh_odometry()
        #self.first_Leg_Is_Right_Leg = tmp1
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
        else:
            if not self.glob.camera_streaming:
                start = time.perf_counter()
                self.glob.vision.detect_Ball_in_One_Shot()
                print('time used for detect_Ball_in_One_Shot:', time.perf_counter() - start)

    def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles, half = False):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        if self.rotation <= 0: 
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_RIGHT']
        else:
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_LEFT']
        if rotation > 0.125 : rotation = 0.125
        if rotation < -0.125 : rotation = -0.125
        alpha = 0
        if self.fr1 == 0:
            alpha01 = math.pi
        else:
            alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        if stepLength < 0:
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK_BACKWARDS'], self.params['BODY_TILT_AT_WALK_BACKWARDS']
        else:
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
            # counter = 0
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: 
            #         print('Sleeping time at walk cycle: ', counter * 0.02)
            #         break
            #     time.sleep(0.02)
            #     counter += 1
        fase_offset = 0.7 #1.57  #0.7
        #order = [10.7, 0, -10.5, -16, -10.7, 0, 10.5, 16]
        order = []
        pos = int(self.amplitude/2)
        for _ in range(self.fr1):
            pos -= int(self.amplitude/self.fr1)
            order.append(pos)
        #order = [8, 0, -8, -16, -8, 0, 8, 16]
        for iii in range(0,frameNumberPerCycle,framestep):
            if half and iii == (frameNumberPerCycle/2 - framestep): break 
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            if 0<= iii <self.fr1 :                                              # FASA 1
                #alpha = alpha01 * (iii/2+ fase_offset*framestep)
                #alpha = alpha01 * iii/2
                #S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                S = order[iii + framestep - 1] * (self.amplitude + self.sideLength ) / self.amplitude
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                #alpha = alpha01 * ((iii-self.fr2)/2+ fase_offset*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                #S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                S = - order[iii - self.fr1 - self.fr2 + framestep - 1] * (self.amplitude + self.sideLength ) / self.amplitude
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                self.ztr = -self.gaitHeight + self.stepHeight
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                    self.ztr = -self.gaitHeight + self.stepHeight/2
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                self.ztl = -self.gaitHeight + self.stepHeight
                #if cycle == number_Of_Cycles - 2:
                #    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                #        self.ztl = -self.gaitHeight
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                    self.ztl = -self.gaitHeight + self.stepHeight/2
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            angles = self.computeAlphaForWalk()
            #print('iii = ', iii, 'xtr =', round(self.xtr,2), 'xtl =', round(self.xtl, 2))
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                print('bad_ik_calc:', 'iii = ', iii, 'xtr:', self.xtr, 'ytr:', self.ytr, 'ztr:', self.ztr, 'xtl:', self.xtl, 'ytl:', self.ytl, 'ztl:', self.ztl )
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        #uprint(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            self.trigger('walk_Cycle')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    servoDatas = []
                    if self.model == 'Roki_2':
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if self.ACTIVESERVOS[i][0] == 8:
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoData = self.Roki.Rcb4.ServoData()
                                servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoDatas.append(servoData)
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
                    #print('calc time =',time1 - time2, 'transfer time =', time2 )
                    time1 = time.perf_counter() - start1
                    #time.sleep(self.frame_delay/1000 - time1)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        if self.glob.with_Local:
            self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
            if self.first_Leg_Is_Right_Leg:
                self.local.coord_shift[1] = -self.side_step_right_yield * sideLength/20/1000
            else: self.local.coord_shift[1] = self.side_step_left_yield * sideLength/20/1000
            if half: 
                self.local.coord_shift[0] *= 0.5
                self.local.coord_shift[1] *= 0.5
            self.local.coordinate_record(odometry = True, shift = True)
            self.local.refresh_odometry()
        #self.first_Leg_Is_Right_Leg = tmp1
        if self.glob.monitor_is_on: self.glob.monitor()
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
        else: 
            if self.glob.role == 'marathon':
                self.glob.vision.detect_Line_Follow_One_Shot()
            else:  
                if self.with_Vision:
                    start = time.perf_counter()
                    self.glob.vision.detect_Ball_in_One_Shot()
                    print('time used for detect_Ball_in_One_Shot:', time.perf_counter() - start)

    def walk_Cycle_With_Tors_v2(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.walk_Cycle_With_Tors_v2_init(stepLength,sideLength, rotation)
        self.walk_Cycle_With_Tors_v2_Phase1(cycle)
        self.walk_Cycle_With_Tors_v2_Phase2(cycle)
        self.walk_Cycle_With_Tors_v2_Phase3()
        self.walk_Cycle_With_Tors_v2_Phase4(cycle, number_Of_Cycles)

        self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        if self.first_Leg_Is_Right_Leg:
            self.local.coord_shift[1] = -self.side_step_right_yield * sideLength/20/1000
        else: self.local.coord_shift[1] = self.side_step_left_yield * sideLength/20/1000
        self.local.coordinate_record(odometry = True, shift = True)
        self.local.refresh_odometry()
        #self.first_Leg_Is_Right_Leg = tmp1
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)

    def walk_Cycle_With_Tors_v2_init(self, stepLength,sideLength, rotation):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        if rotation <= 0: 
            self.rotation = -self.rotation * 0.25 * 0.23 / self.params['ROTATION_YIELD_RIGHT']
        else:
            self.rotation = -self.rotation * 0.25 * 0.23 / self.params['ROTATION_YIELD_LEFT']
        if self.rotation > 0.125 : self.rotation = 0.125
        if self.rotation < -0.125 : self.rotation = -0.125
        alpha = 0
        if self.fr1 == 0:
            self.alpha01 = math.pi
        else:
            self.alpha01 = math.pi/self.fr1*2
        self.dx0_typical = self.stepLength/(2*self.fr1+self.fr2)        # CoM propulsion forward per framestep
        self.dy0_typical = self.sideLength/(2*self.fr1+self.fr2)        # CoM propulsion sideways per framestep
        self.dobavka_x_ot_torsa = self.SIZES[0] * 2 * math.sin(self.ugol_torsa) / self.fr2
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
        self.fase_offset = 1.25

    def walk_Cycle_With_Tors_v2_Phase1(self, cycle):
        phase = '1'
        for i in range(self.fr1):
            alpha = self.alpha01 * (i/2+ self.fase_offset)
            #alpha = alpha01 * i/2
            self.lateral_offset = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
            self.ytr = self.lateral_offset - self.d10 + self.sideLength/2 - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ytl = self.lateral_offset + self.d10 + self.sideLength/2 + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ztl = -self.gaitHeight
            self.ztr = -self.gaitHeight
            if cycle ==0: continue
            else: dx0 = self.dx0_typical + self.dobavka_x_ot_torsa
            #xtl_plan = self.stepLength * ( 0.5 -  self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1
            xtl_plan = self.stepLength * (0.5 - self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
            dx1 = (xtl_plan - self.xtl) / (self.fr1 - i)
            self.xtr += dx1
            self.xtl += dx1
            #self.xtl = self.xtl0 - dx0 - dx0 * i
            #self.xtr = self.xtr0 - dx0 - dx0 * i
            self.perform_motion(phase, i)

    def walk_Cycle_With_Tors_v2_Phase3(self):
        phase = '3'
        for i in range(self.fr1):
            alpha = self.alpha01 * ((i + self.fr1)/2+ self.fase_offset)
            #alpha = alpha01 * (i+self.fr1)/2
            self.lateral_offset = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
            self.ytr = self.lateral_offset - self.d10 - self.sideLength/2 - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ytl = self.lateral_offset + self.d10 + self.sideLength/2 + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ztl = -self.gaitHeight
            self.ztr = -self.gaitHeight
            #dx0 = self.dx0_typical + self.dobavka_x_ot_torsa
            #self.xtl -= dx0
            #self.xtr -= dx0
            xtr_plan = self.stepLength * (0.5 - self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
            dx1 = (xtr_plan - self.xtr) / (self.fr1 - i)
            self.xtr += dx1
            self.xtl += dx1
            self.perform_motion(phase, i)

    def walk_Cycle_With_Tors_v2_Phase2(self, cycle):
        phase = '2'
        dy = self.sideLength/self.fr2
        xtl_plan = self.stepLength * (0.5 - (self.fr1 + self.fr2)/( 2.0 * self.fr1 + self.fr2)) + self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
        xtr_plan = self.stepLength * 0.5 + self.dx0_typical + self.dobavka_x_ot_torsa
        dy0 = self.dy0_typical
        #segments = [1,2,2,1]
        for i in range(self.fr2):
            self.ztr = -self.gaitHeight + self.stepHeight
            #dx0 = self.dx0_typical
            dx2 = (xtl_plan - self.xtl) / (self.fr2 - i)
            if cycle ==0:
                self.tors_angle = math.asin(self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1)/2)
                #dx = (self.stepLength + self.dobavka_x_ot_torsa * self.fr1)/(self.fr2- 2)/2
            else:
                self.tors_angle = math.asin(math.sin(- self.ugol_torsa) + self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1))
                #dx = (self.stepLength + self.dobavka_x_ot_torsa * self.fr1)/(self.fr2- 2) #* 0.75
            if i==0:
                self.xtr += dx2 - self.dobavka_x_ot_torsa
                #self.ytr = self.lateral_offset - 64 + dy0
                self.ytr = self.lateral_offset - self.d10 + dy0 - (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.ztr = -self.gaitHeight + self.stepHeight/2
            elif i == (self.fr2 - 1):
                self.xtr += dx2 - self.dobavka_x_ot_torsa
                self.ytr = self.lateral_offset - self.d10 + 2*dy0 - self.sideLength - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            else:
                if i == 1: 
                    self.ztr = -self.gaitHeight + self.stepHeight/2
                    #dxr = (xtr_plan - self.xtr) / 6
                dx = (xtr_plan - self.xtr) / (self.fr2 - i)
                #dx = dxr * segments[i-1]
                self.xtr += dx 
                self.ytr = self.lateral_offset - 64 + dy0 - dy*self.fr2/(self.fr2- 2)*(i/2) - (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.wr = self.wl = self.rotation - i * self.rotation/(self.fr2)*2
            self.xtl += dx2
            self.ytl += dy0
            self.perform_motion(phase, i)

    def walk_Cycle_With_Tors_v2_Phase4(self, cycle, number_Of_Cycles):
        phase = '4'
        #segments = [1,2,2,1]
        dy0 = self.dy0_typical
        dy = self.sideLength/(self.fr2- 2)
        for i in range(self.fr2):
            self.ztl = -self.gaitHeight + self.stepHeight
            
            if cycle == number_Of_Cycles - 1:
                xtr_plan = xtl_plan = 0
                self.tors_angle = math.asin(math.sin(self.ugol_torsa) - self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1) / 2)
                if i== (self.fr2 - 1):
                    self.ztl = -self.gaitHeight
                    self.ytl = self.lateral_offset + self.d10
            else:
                self.tors_angle = math.asin(math.sin(self.ugol_torsa) - self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1))
                xtr_plan = self.stepLength * (0.5 - (self.fr1 + self.fr2)/( 2.0 * self.fr1 + self.fr2)) + self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
                xtl_plan = self.stepLength * 0.5 + self.dx0_typical + self.dobavka_x_ot_torsa

            dx4 = (xtr_plan - self.xtr) / (self.fr2 - i)
            if i== 0:
                self.xtl += dx4 - self.dobavka_x_ot_torsa
                #self.ytl = self.lateral_offset + 64 + dy0
                self.ytl = self.lateral_offset + self.d10 + dy0 + (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.ztl = -self.gaitHeight + self.stepHeight/2
            elif i== (self.fr2 - 1):
                self.xtl += dx4 - self.dobavka_x_ot_torsa
                self.ytl = self.lateral_offset + self.d10 + 2*dy0 - self.sideLength + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            else:
                if i == 1: 
                    self.ztl = -self.gaitHeight + self.stepHeight/2
                    dxl = (xtl_plan - self.xtl) / 6
                dx = (xtl_plan - self.xtl) / (self.fr2 - i)
                #dx = dxl * segments[i-1]
                self.xtl += dx
                self.ytl = self.lateral_offset + 64 + dy0 - dy * i /2 + (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.wr = self.wl = i * self.rotation/(self.fr2) * 2 - self.rotation
            self.xtr += dx4
            self.ytr += dy0
            self.perform_motion(phase, i)

    def walk_Cycle_With_Tors_v3(self, stepLength,sideLength, direction,cycle, number_Of_Cycles):
        self.walk_Cycle_With_Tors_v3_init(stepLength,sideLength)
        self.stabilize_rotation(direction)
        self.walk_Cycle_With_Tors_v3_Phase1(cycle, direction)
        #self.stabilize_rotation(direction)
        self.walk_Cycle_With_Tors_v3_Phase2(cycle, direction)
        #self.stabilize_rotation(direction)
        self.walk_Cycle_With_Tors_v3_Phase3(direction)
        #self.stabilize_rotation(direction)
        self.walk_Cycle_With_Tors_v3_Phase4(cycle, number_Of_Cycles, direction)

        self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        if self.first_Leg_Is_Right_Leg:
            self.local.coord_shift[1] = -self.side_step_right_yield * sideLength/20/1000
        else: self.local.coord_shift[1] = self.side_step_left_yield * sideLength/20/1000
        self.local.coordinate_record(odometry = True, shift = True)
        self.local.refresh_odometry()
        #self.first_Leg_Is_Right_Leg = tmp1
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)

    def stabilize_rotation(self, direction):
        self.refresh_Orientation()
        rotation = (direction - self.imu_body_yaw()) *1.1
        if rotation <= 0: 
            self.rotation = rotation * 0.25 * 0.23 / self.params['ROTATION_YIELD_RIGHT']
        else:
            self.rotation = rotation * 0.25 * 0.23 / self.params['ROTATION_YIELD_LEFT']
        if self.rotation > 0.6 : self.rotation = 0.6
        if self.rotation < -0.6 : self.rotation = -0.6

    def walk_Cycle_With_Tors_v3_init(self, stepLength,sideLength):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        alpha = 0
        if self.fr1 == 0:
            self.alpha01 = math.pi
        else:
            self.alpha01 = math.pi/self.fr1*2
        self.dx0_typical = self.stepLength/(2*self.fr1+self.fr2)        # CoM propulsion forward per framestep
        self.dy0_typical = self.sideLength/(2*self.fr1+self.fr2)        # CoM propulsion sideways per framestep
        self.dobavka_x_ot_torsa = self.SIZES[0] * 2 * math.sin(self.ugol_torsa) / self.fr2
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
        self.fase_offset = 1.25

    def walk_Cycle_With_Tors_v3_Phase1(self, cycle, direction):
        phase = '1'
        for i in range(self.fr1):
            alpha = self.alpha01 * (i/2+ self.fase_offset)
            #alpha = alpha01 * i/2
            self.lateral_offset = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
            self.ytr = self.lateral_offset - self.d10 + self.sideLength/2# - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ytl = self.lateral_offset + self.d10 + self.sideLength/2# + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ztl = -self.gaitHeight
            self.ztr = -self.gaitHeight
            if cycle ==0: continue
            else: dx0 = self.dx0_typical + self.dobavka_x_ot_torsa
            #xtl_plan = self.stepLength * ( 0.5 -  self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1
            xtl_plan = self.stepLength * (0.5 - self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
            dx1 = (xtl_plan - self.xtl) / (self.fr1 - i)
            self.xtr += dx1
            self.xtl += dx1
            #self.xtl = self.xtl0 - dx0 - dx0 * i
            #self.xtr = self.xtr0 - dx0 - dx0 * i
            self.perform_motion(phase, i, direction)

    def walk_Cycle_With_Tors_v3_Phase3(self, direction):
        phase = '3'
        for i in range(self.fr1):
            alpha = self.alpha01 * ((i + self.fr1)/2+ self.fase_offset)
            #alpha = alpha01 * (i+self.fr1)/2
            self.lateral_offset = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
            self.ytr = self.lateral_offset - self.d10 - self.sideLength/2# - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ytl = self.lateral_offset + self.d10 + self.sideLength/2# + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            self.ztl = -self.gaitHeight
            self.ztr = -self.gaitHeight
            #dx0 = self.dx0_typical + self.dobavka_x_ot_torsa
            #self.xtl -= dx0
            #self.xtr -= dx0
            xtr_plan = self.stepLength * (0.5 - self.fr1/( 2.0 * self.fr1 + self.fr2)) - self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
            dx1 = (xtr_plan - self.xtr) / (self.fr1 - i)
            self.xtr += dx1
            self.xtl += dx1
            self.perform_motion(phase, i, direction)

    def walk_Cycle_With_Tors_v3_Phase2(self, cycle, direction):
        phase = '2'
        dy = self.sideLength/self.fr2
        xtl_plan = self.stepLength * (0.5 - (self.fr1 + self.fr2)/( 2.0 * self.fr1 + self.fr2)) + self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
        xtr_plan = self.stepLength * 0.5 + self.dx0_typical + self.dobavka_x_ot_torsa
        dy0 = self.dy0_typical
        #segments = [1,2,2,1]
        wr0 = self.wr
        wl0 = self.wl
        #self.wr = self.wl = 0
        for i in range(self.fr2):
            self.ztr = -self.gaitHeight + self.stepHeight
            #dx0 = self.dx0_typical
            dx2 = (xtl_plan - self.xtl) / (self.fr2 - i)
            if cycle ==0:
                self.tors_angle = math.asin(self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1)/2)
                #dx = (self.stepLength + self.dobavka_x_ot_torsa * self.fr1)/(self.fr2- 2)/2
            else:
                self.tors_angle = math.asin(math.sin(- self.ugol_torsa) + self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1))
                #dx = (self.stepLength + self.dobavka_x_ot_torsa * self.fr1)/(self.fr2- 2) #* 0.75
            if i==0:
                self.xtr += dx2 - self.dobavka_x_ot_torsa
                #self.ytr = self.lateral_offset - 64 + dy0
                self.ytr = self.lateral_offset - self.d10 + dy0# - (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.ztr = -self.gaitHeight + self.stepHeight/2
            elif i == (self.fr2 - 1):
                self.xtr += dx2 - self.dobavka_x_ot_torsa
                self.ytr = self.lateral_offset - self.d10 + 2*dy0 - self.sideLength# - (1-math.cos(self.tors_angle)) * self.SIZES[0]
            else:
                if i == 1: 
                    self.ztr = -self.gaitHeight + self.stepHeight/2
                    #dxr = (xtr_plan - self.xtr) / 6
                dx = (xtr_plan - self.xtr) / (self.fr2 - i)
                #dx = dxr * segments[i-1]
                self.xtr += dx 
                self.ytr = self.lateral_offset - 64 + dy0 - dy*self.fr2/(self.fr2- 2)*(i/2)# - (1-math.cos(self.tors_angle)) * self.SIZES[0]
                #self.wr = self.wl = self.rotation - i * self.rotation/(self.fr2)*2
            #if i >= self.fr2 - 3 : self.wl = self.rotation * 4
            if self.rotation < 0: 
                self.wl = i * self.rotation/(self.fr2 - 1)
                self.wr = wr0 - (self.rotation - wr0) * i / self.fr2
            else:
                self.wl = wl0 - wl0 * i / self.fr2
                self.wr = wr0 - wr0 * i / self.fr2
            self.xtl += dx2
            self.ytl += dy0
            self.perform_motion(phase, i, direction)

    def walk_Cycle_With_Tors_v3_Phase4(self, cycle, number_Of_Cycles, direction):
        phase = '4'
        #segments = [1,2,2,1]
        dy0 = self.dy0_typical
        dy = self.sideLength/(self.fr2- 2)
        wr0 = self.wr
        wl0 = self.wl
        #self.wr = self.wl = 0
        for i in range(self.fr2):
            self.ztl = -self.gaitHeight + self.stepHeight
            
            if cycle == number_Of_Cycles - 1:
                xtr_plan = xtl_plan = 0
                self.tors_angle = math.asin(math.sin(self.ugol_torsa) - self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1) / 2)
                if i== (self.fr2 - 1):
                    self.ztl = -self.gaitHeight
                    self.ytl = self.lateral_offset + self.d10
            else:
                self.tors_angle = math.asin(math.sin(self.ugol_torsa) - self.dobavka_x_ot_torsa / self.SIZES[0] * (i + 1))
                xtr_plan = self.stepLength * (0.5 - (self.fr1 + self.fr2)/( 2.0 * self.fr1 + self.fr2)) + self.dobavka_x_ot_torsa * self.fr1 * self.fr2 / ( 2.0 * self.fr1 + self.fr2)
                xtl_plan = self.stepLength * 0.5 + self.dx0_typical + self.dobavka_x_ot_torsa

            dx4 = (xtr_plan - self.xtr) / (self.fr2 - i)
            if i== 0:
                self.xtl += dx4 - self.dobavka_x_ot_torsa
                #self.ytl = self.lateral_offset + 64 + dy0
                self.ytl = self.lateral_offset + self.d10 + dy0# + (1-math.cos(self.tors_angle)) * self.SIZES[0]
                self.ztl = -self.gaitHeight + self.stepHeight/2
            elif i== (self.fr2 - 1):
                self.xtl += dx4 - self.dobavka_x_ot_torsa
                self.ytl = self.lateral_offset + self.d10 + 2*dy0 - self.sideLength# + (1-math.cos(self.tors_angle)) * self.SIZES[0]
            else:
                if i == 1: 
                    self.ztl = -self.gaitHeight + self.stepHeight/2
                    dxl = (xtl_plan - self.xtl) / 6
                dx = (xtl_plan - self.xtl) / (self.fr2 - i)
                #dx = dxl * segments[i-1]
                self.xtl += dx
                self.ytl = self.lateral_offset + 64 + dy0 - dy * i /2# + (1-math.cos(self.tors_angle)) * self.SIZES[0]
                #self.wr = self.wl = i * self.rotation/(self.fr2) * 2 - self.rotation
            #if i >= self.fr2 - 3 : self.wr = - self.rotation * 4
            if self.rotation > 0: 
                self.wr = - i * self.rotation/(self.fr2 - 1)
                self.wl = self.wr
            else:
                self.wr = wr0 - wr0 * i / self.fr2
                self.wl = wl0 - wl0 * i / self.fr2
            self.xtr += dx4
            self.ytr += dy0
            self.perform_motion(phase, i, direction)

    def perform_motion(self, phase, iteration, direction):
        #self.stabilize_rotation(direction)
        #if self.rotation > 0: 
        #    self.xtr *= 1.5
        #    self.xtl *= 0.5
        #if self.rotation < 0: 
        #    self.xtl *= 1.5
        #    self.xtr *= 0.5
        angles = self.computeAlphaForWalk(hands_on = True)
        #if self.rotation > 0: 
        #    self.xtr /= 1.5
        #    self.xtl /= 0.5
        #if self.rotation < 0: 
        #    self.xtl /= 1.5
        #    self.xtr /= 0.5
        if not self.falling_Flag ==0: return
        if len(angles)==0:
            print('bad_ik_calc:', 'phase:', phase, ' i = ', iteration, 'xtr:', int(self.xtr), 'ytr:', int(self.ytr), 'ztr:', self.ztr, 'xtl:', int(self.xtl), 'ytl:', int(self.ytl), 'ztl:', self.ztl )
            self.exitFlag = self.exitFlag +1
        else:
            if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                if self.glob.SIMULATION == 3: self.wait_sim_step()
                angles[10] = self.tors_angle
                angles[5] -= self.tors_angle
                angles[16] -= self.tors_angle
                for i in range(len(angles)):
                    if self.keep_hands_up:
                        if i in self.hand_joints : continue
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                        returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                    self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                    self.sim.simx_opmode_oneshot)
                    elif self.glob.SIMULATION == 0:
                        returnCode = self.sim.simxSetJointPosition(self.clientID,
                                    self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                    self.sim.simx_opmode_oneshot)
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    time.sleep(self.slowTime)
                    returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                    self.Dummy_HData.append(Dummy_Hposition)
                    returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                    self.BallData.append(self.Ballposition)
                    #uprint(self.euler_angle)
                    self.timeElapsed = self.timeElapsed +1
                    #uprint(Dummy_Hposition)
                    #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                    #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        #self.sim.simxSynchronousTrigger(self.clientID)
                        self.trigger('perform_motion')
            elif self.glob.SIMULATION == 5:
                joint_number = len(angles)
                servoDatas = []
                if self.model == 'Roki_2':
                    for i in range(joint_number):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.ACTIVESERVOS[i][0] == 8:
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                        else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                        servoData = self.Roki.Rcb4.ServoData()
                        servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                        servoDatas.append(servoData)
                else:
                    servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                    for i in range(joint_number):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                        servoData = self.Roki.Rcb4.ServoData()
                        servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                        servoDatas.append(servoData)
                a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)


    def walk_Final_Pose(self, respect_body_tilt = False):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        framestep = self.simThreadCycleInMs//10
        if self.glob.SIMULATION == 5:
            self.wait_for_gueue_end(self.with_Vision)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: break
            #     time.sleep(0.02)
        if respect_body_tilt: 
            xr, xl = self.xr, self.xl
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']
        for j in range (self.initPoses):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ztr = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/self.initPoses
            self.ztl = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            self.ytl =  self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            if j == self.initPoses - 1:
                angles = self.computeAlphaForWalk(hands_on = False)
            else: angles = self.computeAlphaForWalk()
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        #self.sim.simxSynchronousTrigger(self.clientID)
                        self.trigger('walk_final_pose')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    servoDatas = []
                    if self.model == 'Roki_2':
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if self.ACTIVESERVOS[i][0] == 8:
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoData = self.Roki.Rcb4.ServoData()
                                servoData.Id, servoData.Sio, servoData.Data = 13, self.ACTIVESERVOS[i][1], pos
                                servoDatas.append(servoData)
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    else:
                        for i in range(joint_number):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoData = self.Roki.Rcb4.ServoData()
                            servoData.Id, servoData.Sio, servoData.Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            servoDatas.append(servoData)
                    a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
                    #uprint(servoDatas)
                    #uprint(clock.avg())
                    time1 = time.perf_counter() -start1
                    #time.sleep(self.frame_delay/1000 - time1)
        #self.robot_In_0_Pose = True
        if respect_body_tilt: 
            self.xr, self.xl = xr, xl

    def walk_Final_Pose_After_Kick(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_KICK'], self.params['BODY_TILT_AT_KICK']
        framestep = self.simThreadCycleInMs//10
        pose_taking_cycles = 2
        if self.glob.SIMULATION == 5:
            self.wait_for_gueue_end(self.with_Vision)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: break
            #     time.sleep(0.02)
        for j in range (pose_taking_cycles):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ztr = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/pose_taking_cycles #149
            self.ztl = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/pose_taking_cycles # 149
            self.ytr = -self.d10 - (pose_taking_cycles-(j+1))*self.amplitude/2 /pose_taking_cycles
            self.ytl =  self.d10 - (pose_taking_cycles-(j+1))*self.amplitude/2 /pose_taking_cycles
            if j == pose_taking_cycles - 1:
                angles = self.computeAlphaForWalk(hands_on = False)
            else: angles = self.computeAlphaForWalk()
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        #self.sim.simxSynchronousTrigger(self.clientID)
                        self.trigger('walk_Final_Pose_After_Kick')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number - 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
                    #uprint(servoDatas)
                    #uprint(clock.avg())
                    time1 = time.perf_counter() - start1
                    #time.sleep(self.frame_delay/1000 - time1)
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        #self.robot_In_0_Pose = True

    def kick(self, first_Leg_Is_Right_Leg, small = False, kick_offset = 0):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        gaitHeight = 210
        stepHeight = 55
        stepLength = 64
        kick_size = self.kick_power - 30
        #if small : kick_size = -10
        tmp1 = self.first_Leg_Is_Right_Leg
        self.first_Leg_Is_Right_Leg = first_Leg_Is_Right_Leg
        tmp = self.gaitHeight
        self.gaitHeight = gaitHeight
        self.walk_Initial_Pose(amplitude = 46)
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_KICK'], self.params['BODY_TILT_AT_KICK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        fase_offset = 0.7
        if self.glob.SIMULATION == 5:
            self.wait_for_gueue_end(self.with_Vision)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: break
            #     time.sleep(0.02)
        for iii in range(0,frameNumberPerCycle,framestep):
            if 0<= iii <self.fr1 :
                alpha = alpha01 * (iii/2+fase_offset*framestep)
                S = (self.amplitude/2 )*math.cos(alpha)
                self.ytr = S - self.d10
                self.ytl = S + self.d10
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                continue
            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :
                alpha = alpha01 * ((iii-self.fr2)/2+fase_offset*framestep)
                S = (self.amplitude/2)*math.cos(alpha)
                self.ytr = S - self.d10
                self.ytl = S + self.d10
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0
            if self.fr1<= iii <self.fr1+self.fr2:
                self.ztr = -gaitHeight + stepHeight
                dx = stepLength/2/self.fr2*2
                dx0 = stepLength/(2*self.fr1+self.fr2+4)*framestep
                if iii==self.fr1:
                    self.xtr -= dx0
                    self.ytr = S - (64 + kick_offset)
                elif iii == (self.fr1 +self.fr2 - 2):
                    self.xtr -= dx0
                    self.ytr = S - (64 + kick_offset)
                else:
                    self.xtr += dx*self.fr2/(self.fr2-2 * framestep)
                    self.ytr = S - (64 + kick_offset)
                if iii == self.fr1 +self.fr2 - 10: self.xtr += kick_size
                if iii == self.fr1 +self.fr2 - 4: self.xtr -= kick_size
                self.xtl -= dx0
            if 2*self.fr1+self.fr2<= iii :
                self.ztl = -gaitHeight + stepHeight
                dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                dx = (stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2 - 2 * framestep) * framestep
                if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.ztl = -gaitHeight
                    self.ytl = S + self.d10

                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    self.ytl = S + 64
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + 64
                else:
                    self.xtl += dx
                    self.ytl = S + 64
                self.xtr -= dx0
            angles = self.computeAlphaForWalk()
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                print("unresolved IK at kick")
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #uprint(self.euler_angle)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            self.trigger('kick')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number- 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas,self.frames_per_cycle, 0)
                    #uprint(disp)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        self.walk_Final_Pose_After_Kick()
        self.pause_in_ms(100)
        self.local.coord_shift[0] = self.first_step_yield/2000
        self.local.coord_shift[1] = 0
        self.local.coordinate_record(odometry = True, shift = True)
        self.local.refresh_odometry()
        self.gaitHeight = tmp
        self.first_Leg_Is_Right_Leg = tmp1

    def refresh_Orientation(self):
        if self.glob.SIMULATION == 5:
            for _ in range(10):
                result1, pitch, roll, yaw = self.stm_channel.pitch_roll_yaw_from_imu_in_body()
                if result1: 
                    self.body_euler_angle['pitch'], self.body_euler_angle['roll'], self.body_euler_angle['yaw'] = pitch, roll, yaw
                    self.body_euler_angle['yaw'] -= self.direction_To_Attack
                    self.body_euler_angle['yaw'] += self.imu_drift_speed * (time.perf_counter() - self.start_point_for_imu_drift)
                    break
                time.sleep(0.002)
            for _ in range(10):
                result2, pitch, roll, yaw = self.stm_channel.pitch_roll_yaw_from_imu_in_head()
                if result2: 
                    self.euler_angle['pitch'], self.euler_angle['roll'], self.euler_angle['yaw'] = pitch, roll, yaw
                    self.euler_angle['yaw'] -= self.direction_To_Attack
                    self.euler_angle['yaw'] += self.imu_drift_speed * (time.perf_counter() - self.start_point_for_imu_drift)
                    break
                time.sleep(0.002)
            #print('body pitch:', round(self.body_euler_angle['pitch'], 2), 'body yaw:', round(self.body_euler_angle['yaw'], 2) )
        else:
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            Dummy_Hquaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
            self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
            #self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.euler_angle['yaw']-= self.direction_To_Attack
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            self.body_euler_angle['yaw'] -= self.direction_To_Attack
            self.body_euler_angle['yaw'] += self.imu_drift_speed * (time.perf_counter() - self.start_point_for_imu_drift)
            self.euler_angle['yaw'] -= self.direction_To_Attack
            self.euler_angle['yaw'] += self.imu_drift_speed * (time.perf_counter() - self.start_point_for_imu_drift)

    def turn(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        pose_taking_cycles = 2
        if self.glob.SIMULATION == 5:
            self.wait_for_gueue_end(self.with_Vision)
            # while True:
            #     if self.stm_channel.mb.GetBodyQueueInfo()[1].Size < 3: break
            #     time.sleep(0.02)

        for j in range (pose_taking_cycles):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ztr = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/pose_taking_cycles #149
            self.ztl = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/pose_taking_cycles # 149
            self.ytr = -self.d10 - (pose_taking_cycles-(j+1))*self.amplitude/2 /pose_taking_cycles
            self.ytl =  self.d10 - (pose_taking_cycles-(j+1))*self.amplitude/2 /pose_taking_cycles
            if j == pose_taking_cycles - 1:
                angles = self.computeAlphaForWalk(hands_on = False)
            else: angles = self.computeAlphaForWalk()
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                    if self.glob.SIMULATION == 1:
                        #self.sim.simxSynchronousTrigger(self.clientID)
                        self.trigger('turn')
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number - 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
                    time1 = time.perf_counter() - start1
        #self.robot_In_0_Pose = True

    def hard_kick(self, kick_by_right = 1, kick_offset = 0):
        var = roki2met.roki2met
        intercom = self.glob.stm_channel.zubr       # used for communication between head and zubr-controller with memIGet/memISet commands
        if kick_offset < -60 : kick_offset = -60
        if kick_offset > 80 : kick_offset = 80
        intercom.memISet(var.kick_by_right, kick_by_right)
        intercom.memISet(var.kick_power, self.kick_power)
        intercom.memISet(var.kick_offset, kick_offset)
        self.glob.rcb.motionPlay(31)



if __name__=="__main__":
    print('This is not main module!')

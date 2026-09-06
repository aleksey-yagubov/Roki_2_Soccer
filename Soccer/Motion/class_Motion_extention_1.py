import math, time

class Motion_extention_1:

    def walk_Cycle_slow(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles, half = False):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        if self.rotation <= 0: 
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_RIGHT']
        else:
            rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD_LEFT']
        alpha = 0
        if self.fr1 == 0:
            alpha01 = math.pi
        else:
            alpha01 = math.pi/self.fr1*2
        fading = self.fr1 /2 #32
        skew = 0.1
        swing_leg_distance_max = 120
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = (self.sideLength)/(2 * self.fr2)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)
        for iii in range(0,frameNumberPerCycle ,framestep):
            if half and iii == (frameNumberPerCycle/2 - framestep): break
            self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
            if 0<= iii <self.fr1 :                                              # FASA 1
                if iii < self.fr1/2 :
                    alpha = alpha01 * (iii + framestep)
                    S = (self.amplitude/4 )*math.cos(alpha) + (self.amplitude/4 )
                else:
                    alpha = alpha01 * (iii + framestep) - math.pi
                    S = (self.amplitude/4 )*math.cos(alpha) - (self.amplitude/4 )
                #alpha = alpha01 * (iii + framestep)/2
                #S = (self.amplitude/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: 
                    dx0 = dx0_typical
                    if iii < fading:
                        self.ztl = -self.gaitHeight + self.stepHeight * (fading - iii) / fading
                        self.yl += skew * (fading - iii + framestep) / fading
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2  <= iii < 2*self.fr1+self.fr2  :                     # FASA 3
                #alpha = alpha01 * (iii-self.fr2 + framestep)/2
                #S = (self.amplitude/2 )*math.cos(alpha)
                if iii < 3/2 * self.fr1 + self.fr2:
                    alpha = alpha01 * (iii-self.fr2 + framestep) - math.pi
                    S = (self.amplitude/4 )*math.cos(alpha) - (self.amplitude/4 )
                else:
                    alpha = alpha01 * (iii-self.fr2 + framestep) - 2 * math.pi
                    S = (self.amplitude/4 )*math.cos(alpha) + (self.amplitude/4 )
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                if iii < self.fr1+self.fr2 + fading:
                    self.ztr = -self.gaitHeight + self.stepHeight * (self.fr1+self.fr2 + fading - iii) / fading
                    self.yr -= skew * (self.fr1+self.fr2 + fading - iii + framestep) / fading
                else:
                    self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii < self.fr1+self.fr2:                               # FASA 2
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
                if iii < self.fr1 + self.fr2/2:
                    swing_leg_distance = self.d10 + (swing_leg_distance_max - self.d10) * 2 * (iii - self.fr1) / self.fr2
                else: 
                    swing_leg_distance = swing_leg_distance_max + (self.d10 - swing_leg_distance_max) *(2 * iii - 2 * self.fr1 - self.fr2) / self.fr2
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - self.d10 + dy0
                    self.ytr = S - swing_leg_distance + dy0
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - swing_leg_distance + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - swing_leg_distance + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            #if self.fr1+self.fr2 <= iii < self.fr1+self.fr2 +16:                 # FASA 2+
            #    self.ztr = -self.gaitHeight + self.stepHeight * (self.fr1+self.fr2 +16 - iii) / 16

            #if 2*(self.fr1+self.fr2) + 16  <= iii :                 # FASA 4+
            #    self.ztl = -self.gaitHeight + self.stepHeight * (2*(self.fr1+self.fr2 + 16) - iii) / 16

            if 2*self.fr1+self.fr2  <= iii < 2*(self.fr1+self.fr2)  :                                         # FASA 4
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

                if iii < 2 * self.fr1 + self.fr2 * 3/2:
                    swing_leg_distance = self.d10 + (swing_leg_distance_max - self.d10) * 2 * (iii - (2*self.fr1+self.fr2)) / self.fr2
                else: 
                    swing_leg_distance = swing_leg_distance_max + (self.d10 - swing_leg_distance_max) *(2 * iii - 4 * self.fr1 - 3 * self.fr2) / self.fr2
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + self.d10 + self.sideLength/2
                    self.ytl = S + swing_leg_distance + self.sideLength/2
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + swing_leg_distance + self.sideLength/2
                else:
                    self.xtl += dx
                    self.ytl = S + swing_leg_distance + self.sideLength/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            angles = self.computeAlphaForWalk()
            #if iii  == self.fr1+self.fr2:
            if iii == 0: self.camera_counter = 0
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        self.timeElapsed = self.timeElapsed +1
                        if self.glob.SIMULATION == 1:
                            #self.sim_simxSynchronousTrigger(self.clientID)
                            self.trigger('walk_Cycle_slow')
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
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        if self.first_Leg_Is_Right_Leg:
            self.local.coord_shift[1] = -self.side_step_right_yield * abs(sideLength)/20/1000
        else: self.local.coord_shift[1] = self.side_step_left_yield * abs(sideLength)/20/1000
        self.local.coordinate_record(odometry = True, shift = True)
        if self.glob.SIMULATION == 5: self.wait_for_gueue_end(self.with_Vision)

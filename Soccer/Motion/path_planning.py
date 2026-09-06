import sys
import os
import math
import array
import json
                                        # 1 - Simulation synchronous with physics, 
                                        # 3 - Simulation streaming with physics



goalPostRadius = 0.15       # Radius to walk around a goal post (in m).
ballRadius = 0.1           # Radius to walk around the ball (in m).
uprightRobotRadius = 0.2  # Radius to walk around an upright robot (in m).
roundAboutRadiusIncrement = 0.15

def uprint(*text):
    #with open("output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class Glob:
    def __init__(self):
        self.COLUMNS = 18
        self.ROWS = 13
        self.pf_coord =   [0.276, 0.749, 2]  #[-0.4, 0.0 , 0] # [0.5, 0.5 , -math.pi * 3/4]
        self.ball_coord = [-0.132, 0.957]        #[0, 0]
        self.obstacles = [[0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]  #[[0, 0, 0.15], [0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]
        #self.ball_coord = [2, 0]
        #self.obstacles = [[0.4, 0.025, 0.2], [0.725, -0.475, 0.2], [0.8, 0.55, 0.2], [1.175, 0, 0.2], [1.625, -0.4, 0.2], [1.7, 0.425, 0.2]]
        self.landmarks = {"post1": [[ 1.8, -0.6 ]], "post2": [[ 1.8, 0.6 ]], "post3": [[ -1.8, 0.6 ]], "post4": [[ -1.8, -0.6 ]],
                          "unsorted_posts": [[ 1.8, 0.6 ],[ 1.8, -0.6 ],[ -1.8, 0.6 ],[ -1.8, -0.6 ]],
                          "FIELD_WIDTH": 2.6, "FIELD_LENGTH": 3.6 }
        self.params = {'CYCLE_STEP_YIELD': 103.5}
        self.cycle_step_yield = 103.5
        current_work_directory = os.getcwd()
        current_work_directory = current_work_directory.replace('\\', '/')
        current_work_directory += '/'
        self.strategy_data = array.array('b',(0 for i in range(self.COLUMNS * self.ROWS * 2)))
        self.import_strategy_data(current_work_directory)

    def import_strategy_data(self, current_work_directory):
        with open(current_work_directory + "Init_params/strategy_data.json", "r") as f:
            loaded_Dict = json.loads(f.read())
        if loaded_Dict.get('strategy_data') != None:
            strategy_data = loaded_Dict['strategy_data']
        for column in range(self.COLUMNS):
            for row in range(self.ROWS):
                index1 = column * self.ROWS + row
                power = strategy_data[index1][2]
                yaw = int(strategy_data[index1][3] * 40)  # yaw in radians multiplied by 40
                self.strategy_data[index1*2] = power
                self.strategy_data[index1*2+1] = yaw

class Forward:
    def __init__(self, glob):
        self.glob = glob

    def direction_To_Guest(self):
        if self.glob.ball_coord[0] < 0: 
            return 0
        elif self.glob.ball_coord[0] > 0.8 and abs(self.glob.ball_coord[1]) > 0.6:
            return math.atan(-self.glob.ball_coord[1]/(1.8-self.glob.ball_coord[0]))
        elif self.glob.ball_coord[0] < 1.5 and abs(self.glob.ball_coord[1]) < 0.25:
            if (1.8-self.glob.ball_coord[0]) == 0: return 0
            else: 
                if abs(self.glob.ball_coord[1]) < 0.2:
                    return math.atan((math.copysign(0.5, self.glob.ball_coord[1])-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
                else: 
                    return math.atan((0.5* (round(random.random(),0)*2 - 1)-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
        else:
            return math.atan(-self.glob.pf_coord[1]/(2.4-self.glob.pf_coord[0]))

class Forward_Vector_Matrix:
    def __init__(self, glob):
        self.glob = glob
        #self.direction_To_Guest = 0
        self.kick_Power = 1
        

    def direction_To_Guest(self):
        if abs(self.glob.ball_coord[0])  >  self.glob.landmarks["FIELD_LENGTH"] / 2:
            ball_x = math.copysign(self.glob.landmarks["FIELD_LENGTH"] / 2, self.glob.ball_coord[0])
        else: ball_x = self.glob.ball_coord[0]
        if abs(self.glob.ball_coord[1])  >  self.glob.landmarks["FIELD_WIDTH"] / 2:
            ball_y = math.copysign(self.glob.landmarks["FIELD_WIDTH"] / 2, self.glob.ball_coord[1])
        else: ball_y = self.glob.ball_coord[1]
        col = math.floor((ball_x + self.glob.landmarks["FIELD_LENGTH"] / 2) / (self.glob.landmarks["FIELD_LENGTH"] / self.glob.COLUMNS))
        row = math.floor((- ball_y + self.glob.landmarks["FIELD_WIDTH"] / 2) / (self.glob.landmarks["FIELD_WIDTH"] / self.glob.ROWS))
        if col >= self.glob.COLUMNS : col = self.glob.COLUMNS - 1
        if row >= self.glob.ROWS : row = self.glob.ROWS -1
        direction_To_Guest = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2 + 1] / 40
        self.kick_Power = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2]
        print('direction_To_Guest = ', math.degrees(direction_To_Guest))
        return direction_To_Guest


class PathPlan:
    def __init__(self, glob):
        self.glob = glob
        self.posts = [self.glob.landmarks["post1"][0], self.glob.landmarks["post2"][0], self.glob.landmarks["post3"][0], self.glob.landmarks["post4"][0]]
        self.goal_bottoms = [[[self.posts[0][0]+0.10, self.posts[0][1]],[self.posts[1][0]+0.10, self.posts[1][1]]],
                             [[self.posts[2][0]-0.10, self.posts[2][1]],[self.posts[3][0]-0.10, self.posts[3][1]]],
                             [[self.posts[0][0], self.posts[0][1]],[self.posts[0][0]+0.35, self.posts[0][1]]],
                             [[self.posts[1][0], self.posts[1][1]],[self.posts[1][0]+0.35, self.posts[1][1]]],
                             [[self.posts[2][0], self.posts[2][1]],[self.posts[2][0]-0.35, self.posts[2][1]]],
                             [[self.posts[3][0], self.posts[3][1]],[self.posts[3][0]-0.35, self.posts[3][1]]]]
        

    def coord2yaw(self, x, y):
        if x == 0:
            if y > 0 : yaw = math.pi/2
            else: yaw = -math.pi/2
        else: yaw = math.atan(y/x)
        if x < 0: 
            if yaw > 0: yaw -= math.pi
            else: yaw += math.pi
        return yaw

    def intersection_line_segment_and_line_segment(self, x1, y1, x2, y2, x3, y3, x4, y4 ):
        """
        x = x1 + (x2 - x1) * t1    t1 - paramentric coordinate
        y = y1 + (y2 - y1) * t1
        x = x3 + (x4 - x3) * t2    t2 - paramentric coordinate
        y = y3 + (y4 - y3) * t2
        x1 + (x2 - x1) * t1 = x3 + (x4 - x3) * t2
        y1 + (y2 - y1) * t1 = y3 + (y4 - y3) * t2
        t1 = (x3 + (x4 - x3) * t2 - x1) / (x2 - x1)
        t1 = (y3 + (y4 - y3) * t2 - y1) / (y2 - y1)
        y1 + (y2 - y1) * (x3 + (x4 - x3) * t2 - x1) / (x2 - x1) = y3 + (y4 - y3) * t2
        (y2 - y1) * (x4 - x3)/ (x2 - x1) * t2 - (y4 - y3) * t2 = y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)
        t2 = (y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)) /((y2 - y1) * (x4 - x3)/ (x2 - x1) - (y4 - y3))
        if t1 == 0:
            t2 = (y1 - y3)/ (y4 - y3)
            t2 = (x1 - x3)/ (x4 - x3)
        """
        if x2 == x1:
            if x4 == x3:
                if x1 == x3 and max(y1,y2) >= min(y3,y4) and max(y3, y4) >= min(y1,y2):
                    return True
                else: return False
            elif y2 == y1:
                if x3 == x4:
                    if y3 == y4:
                        if x1 == x3 and y1 == y3:
                            return True
                        else: return False
                    else:
                        t2 = (y1 - y3)/ (y4 - y3)
                        if x1 == x3 and 0 <= round(t2, 4) <= 1:
                            return True
                        else: return False
                else:
                    dt =  (y1 - y3)/ (y4 - y3) - (x1 - x3)/ (x4 - x3)
                    if round(dt, 4) == 0:
                        return True
                    else: return False
            else:
                t2 = (x1 - x3)/(x4 - x3)
                t1 = (y3 - y1 + (y4 - y3) * t2) / (y2 - y1)
                if 0 <= round(t1, 4) <= 1 and 0 <= round(t2, 4) <= 1:
                    return True
                else: return False
        else:
            if (y2 - y1) * (x4 - x3) == (y4 - y3) * (x2 - x1):
                if max(x1,x2) >= min(x3,x4) and max(x3, x4) >= min(x1,x2): 
                    return True
                else: return False
            else:
                t2 = (y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)) /((y2 - y1) * (x4 - x3)/ (x2 - x1) - (y4 - y3))
                t1 = (x3 + (x4 - x3) * t2 - x1) / (x2 - x1)
                if 0 <= round(t1, 4) <= 1 and 0 <= round(t2, 4) <= 1:
                    return True
                else: return False


    def intersection_line_segment_and_circle(self, x1, y1, x2, y2, xc, yc, R):
        """
        x = x1 + (x2 - x1) * t    t - paramentric coordinate
        y = y1 + (y2 - y1) * t
        R**2 = (x - xc)**2 + (y - yc)**2
        (x1 + (x2 - x1) * t - xc)**2 + (y1 + (y2 - y1) * t - yc)**2 - R**2 = 0
        ((x2 - x1) * t)**2 + (x1 - xc)**2 + 2 * (x2 - x1) * (x1 - xc) * t + 
        ((y2 - y1) * t)**2 + (y1 - yc)**2 + 2 * (y2 - y1) * (y1 - yc) * t - R**2 = 0
        ((x2 - x1)**2 + (y2 - y1)**2) * t**2 + (2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)) * t +
        (x1 - xc)**2 + (y1 - yc)**2 - R**2 = 0
        a * t**2 + b * t + c = 0
        a = (x2 - x1)**2 + (y2 - y1)**2
        b = 2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)
        c = (x1 - xc)**2 + (y1 - yc)**2 - R**2
        """
        a = (x2 - x1)**2 + (y2 - y1)**2
        b = 2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)
        c = (x1 - xc)**2 + (y1 - yc)**2 - R**2
        successCode, t1, t2 = self.square_equation(a,b,c)
        if successCode:
            if 0 <= round(t1, 4) <= 1 or 0 <= round(t2, 4) <= 1 or (t1 > 1 and t2 < 0) or (t2 > 1 and t1 < 0):
               return True
        return False

    def intersection_circle_segment_and_circle(self, x1, y1, x2, y2, x0, y0, CW, xc, yc, R):
        R0sq = (x1 - x0)**2 + (y1 - y0)**2 
        if yc == y0:
            xp1 = xp2 = (R0sq - R**2 + xc**2 - x0**2)/(2 * (xc - x0))
            if R0sq - (xp1 - x0)**2 < 0: return False 
            yp1 = y0 + math.sqrt(R0sq - (xp1 - x0)**2)
            yp2 = y0 - math.sqrt(R0sq - (xp1 - x0)**2)
        else:
            A = (x0 - xc)/(yc - y0)
            B = (R0sq - R**2 + xc**2 + yc**2 - x0**2 - y0**2)/(2*(yc - y0))
            a = 1 + A**2
            b = 2 * (-A * y0 + A * B - x0)
            c = x0**2 - R0sq + (B - y0)**2
            succsessCode, xp1, xp2 = self.square_equation(a, b, c)
            if not succsessCode: return False
            yp1 = A * xp1 + B
            yp2 = A * xp2 + B
        alpha2 = self.coord2yaw(x2 - x0, y2 - y0)
        alpha1 = self.coord2yaw(x1 - x0, y1 - y0)
        alphap2 = self.coord2yaw(xp2 - x0, yp2 - y0)
        alphap1 = self.coord2yaw(xp1 - x0, yp1 - y0)
        if CW:
            if alpha1 < alpha2: 
                alpha1 += math.pi * 2
                if alphap1 < 0 : alphap1 += math.pi * 2
                if alphap2 < 0 : alphap2 += math.pi * 2
            if alpha2 <= alphap1 <= alpha1 or alpha2 <= alphap2 <= alpha1:
                return True
        else:
            if alpha2 < alpha1:
                alpha2 += math.pi * 2
                if alphap1 < 0 : alphap1 += math.pi * 2
                if alphap2 < 0 : alphap2 += math.pi * 2
            if alpha1 <= alphap1 <= alpha2 or alpha1 <= alphap2 <= alpha2:
                return True
        return False

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi 
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def delta_yaw(self, start_yaw, dest_yaw, CW):
        s = math.degrees(start_yaw)
        d = math.degrees(dest_yaw)
        if CW:
            if start_yaw < dest_yaw:
                start_yaw += math.pi * 2
        else:
            if dest_yaw < start_yaw:
                dest_yaw += math.pi * 2
        delta_yaw = dest_yaw - start_yaw
        #print('CW = ', CW, 'start_yaw = ', s, 'dest_yaw = ', d, 'delta_yaw = ', math.degrees(delta_yaw))
        return delta_yaw

    def path_calc_optimum(self, start_coord, target_coord):
        dest, centers, number_Of_Cycles = self.path_calc(start_coord, target_coord)
        if len(centers) > 0:
            x1, y1, x2, y2, cx, cy, R, CW = centers[0]
            if R <= 0.08:
                start_yaw = start_coord[2]
                dest_yaw = self.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1])
                delta_yaw = self.delta_yaw(start_yaw, dest_yaw, CW)
                if abs(delta_yaw) > math.pi: centers[0][7] = not CW
            x1, y1, x2, y2, cx, cy, R, CW = centers[len(centers)-1]
            if R <= 0.08:
                start_yaw = self.coord2yaw(dest[len(dest)-1][0] - dest[len(dest)-2][0], dest[len(dest)-1][1] - dest[len(dest)-2][1])
                dest_yaw = target_coord[2]
                delta_yaw = self.delta_yaw(start_yaw, dest_yaw, CW)
                if abs(delta_yaw) > math.pi: centers[len(centers)-1][7] = not CW
        return dest, centers, number_Of_Cycles

    def path_calc(self, start_coord, target_coord):
        x1, y1, yaw1 = start_coord 
        x2, y2, yaw2 = target_coord
        dest1, centers1, number_Of_Cycles1 = self.arc_path_internal( x1, y1, yaw1, x2, y2, yaw2)
        dest2, centers2, number_Of_Cycles2 = self.arc_path_external( x1, y1, yaw1, x2, y2, yaw2)
        if number_Of_Cycles1 < number_Of_Cycles2: return dest1, centers1, number_Of_Cycles1
        else: return dest2, centers2, number_Of_Cycles2
        return dest1, centers1

    def arc_path_external(self, x1, y1, yaw1, x2, y2, yaw2):
        number_Of_Cycles_min = 1000
        for i in range(10):
            for j in range(10):
                R1 = i * 0.05
                R2 = j * 0.05
                #R1 = 0.2
                #R2 = 0.2
                if (y2-y1) < 0:
                    xc1 = x1 - R1 * math.sin(yaw1)
                    yc1 = y1 + R1 * math.cos(yaw1)
                    xc2 = x2 - R2 * math.sin(yaw2)
                    yc2 = y2 + R2 * math.cos(yaw2)
                    CW1 = False
                    CW2 = False
                else:
                    xc1 = x1 + R1 * math.sin(yaw1)
                    yc1 = y1 - R1 * math.cos(yaw1)
                    xc2 = x2 + R2 * math.sin(yaw2)
                    yc2 = y2 - R2 * math.cos(yaw2)
                    CW1 = True
                    CW2 = True
                successCode, xp1, yp1 = self.external_tangent_line(True, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW1 )
                if successCode:
                    successCode, xp2, yp2 = self.external_tangent_line(False, R2, R1, x2, y2, xc2, yc2, xc1, yc1, CW2 )
                    if successCode:
                        dest = [[xp1,yp1], [xp2,yp2]]
                        centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1], [xp2, yp2, x2, y2, xc2, yc2, R2, CW2]]
                        nearestObstacle = self.check_Obstacle(xp1, yp1, xp2, yp2)
                        if nearestObstacle >= 0 :
                            roundAboutRadius = self.glob.obstacles[nearestObstacle][2] / 2 + roundAboutRadiusIncrement
                        #if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                        #                             self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                            for variant in range(2):
                                if variant == 0:
                                    CW = CW1
                                    successCode1, xp1, yp1 = self.external_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode2, xp2, yp2 = self.external_tangent_line(False,
                                           roundAboutRadius, R1, x2, y2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode3, xp3, yp3 = self.external_tangent_line(True,
                                           roundAboutRadius, R2, x1, y1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode4, xp4, yp4 = self.external_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode1 and successCode2 and successCode3 and successCode4) : continue
                                if variant == 1:
                                    CW = not CW1
                                    successCode5, xp1, yp1 = self.internal_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode6, xp2, yp2 = self.internal_tangent_line(False,
                                           roundAboutRadius, R1, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode7, xp3, yp3 = self.internal_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode8, xp4, yp4 = self.internal_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode5 and successCode6 and successCode7 and successCode8) : continue
                                dest = [[xp1,yp1], [xp2,yp2], [xp3,yp3], [xp4,yp4]]
                                centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1],
                                            [xp2, yp2, xp3, yp3, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], roundAboutRadius, CW],
                                            [xp4, yp4, x2, y2, xc2, yc2, R2, CW2]]
                                price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                                number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                                if number_Of_Cycles < number_Of_Cycles_min:
                                    number_Of_Cycles_min = number_Of_Cycles
                                    dest_min = dest
                                    centers_min = centers
                        else:
                            price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                            number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                            if number_Of_Cycles < number_Of_Cycles_min:
                                number_Of_Cycles_min = number_Of_Cycles
                                dest_min = dest
                                centers_min = centers
        if number_Of_Cycles_min == 1000: return [], [], number_Of_Cycles_min
        else: return dest_min, centers_min, number_Of_Cycles_min

    def check_Obstacle(self, xp1, yp1, xp2, yp2):
        nearestObstacle = -1
        obstacles = []
        distances = []
        for j in range(len(self.glob.obstacles)):
            roundAboutRadius = self.glob.obstacles[j][2] / 2 + roundAboutRadiusIncrement
            if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                                            self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius):
                obstacles.append(j)
                distances.append(math.sqrt((xp1 - self.glob.obstacles[j][0])**2 + (yp1 - self.glob.obstacles[j][1])**2))
        if obstacles != []:
            nearestObstacle = obstacles[distances.index(min(distances))]
        return nearestObstacle

    def check_Limits(self, x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest):
        permit = True
        for i in range(0, len(dest), 2):
            if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                        ballRadius + roundAboutRadiusIncrement): permit = False
        ind = len(dest) - 1
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                    ballRadius + roundAboutRadiusIncrement): permit = False
        if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                      self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius): permit = False
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius): permit = False

        for j in range(4):
            goalPostX, goalPostY = self.glob.landmarks["unsorted_posts"][j]
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    goalPostX, goalPostY, goalPostRadius): permit = False
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                    goalPostX, goalPostY, goalPostRadius): permit = False
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         goalPostX, goalPostY, goalPostRadius): permit = False
        #if permit == False: print('permit denied')
        return permit

    def check_Price(self, x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers):
        price = 0
        for i in range(0, len(dest), 2):
            if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                         ballRadius + roundAboutRadiusIncrement): price += 200
        ind = len(dest) - 1
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                    ballRadius + roundAboutRadiusIncrement): price += 200
        for j in range(len(self.glob.obstacles)):
            roundAboutRadius = self.glob.obstacles[j][2] / 2 + roundAboutRadiusIncrement
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                          self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 200
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                        self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 100
            for i in range(0, len(dest), 2):
                    if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 300 - i * 100
        for j in range(4):
            goalPostX, goalPostY = self.posts[j]
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    goalPostX, goalPostY, goalPostRadius): price += 200
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                    goalPostX, goalPostY, goalPostRadius): price += 300
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         goalPostX, goalPostY, goalPostRadius): price += 300 - i * 100

        for j in range(6):
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_line_segment(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                   self.goal_bottoms[j][0][0], self.goal_bottoms[j][0][1], self.goal_bottoms[j][1][0], self.goal_bottoms[j][1][1]):
                   price += 300 # - i * 100
            for i in range(len(centers)):
                if self.intersection_line_segment_and_circle(self.goal_bottoms[j][0][0], self.goal_bottoms[j][0][1],
                   self.goal_bottoms[j][1][0], self.goal_bottoms[j][1][1], 
                   centers[i][4], centers[i][5], centers[i][6]): price += 300 #- i * 100

        return price

    def number_Of_Cycles_count(self, dest, centers, yaw1, yaw2):
        prop_yaw_glob1 = self.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1])
        x1, y1, x2, y2, cx, cy, R, CW = centers[0]
        prop_yaw_local1 = self.delta_yaw(yaw1, prop_yaw_glob1, CW)
        number_Of_Cycles = math.ceil(abs(prop_yaw_local1 / 0.2))
        for i in range(0,len(dest),2):
            distance = math.sqrt((dest[i+1][0] - dest[i][0])**2 + (dest[i+1][1] - dest[i][1])**2)
            number_Of_Cycles += math.ceil(abs(distance / (self.glob.cycle_step_yield/1000)))
            x1, y1, x2, y2, cx, cy, R, CW = centers[int(i/2+1)]
            if len(dest) == i+2 : 
                prop_yaw_local2 = self.delta_yaw(prop_yaw_glob1, yaw2, CW)
            else:
                prop_yaw_glob2 = self.coord2yaw(dest[i+3][0] - dest[i+2][0], dest[i+3][1] - dest[i+2][1])
                prop_yaw_local2 = self.delta_yaw(prop_yaw_glob1, prop_yaw_glob2, CW)
                prop_yaw_glob1 = prop_yaw_glob2
            number_Of_Cycles += math.ceil(abs(prop_yaw_local2 / 0.2))
        return number_Of_Cycles


    def external_tangent_line(self, start, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW ):
        if R1 == 0: return True, x1, y1
        L = math.sqrt((xc2 - xc1)**2 + (yc2 - yc1)**2)
        if R1 == R2:
            xp1 = [xc1 + (yc2-yc1) * R1 /L, xc1 - (yc2-yc1) * R1/L]
            yp1 = [yc1 - (xc2-xc1) * R1/L, yc1 + (xc2-xc1) * R1/L]
        else:
            L1 = L * R1 / abs(R1 - R2)
            L2 = L * R2 / abs(R1 - R2)
            x0 = (R2 * xc1 - R1 * xc2) / (R2 - R1)
            y0 = (R2 * yc1 - R1 * yc2) / (R2 - R1) 
            if yc1 == y0:
                xp = (L1**2 - 2 * R1**2 + xc1**2 - x0**2)/(2 * xc1 - 2 * x0)
                xp1 = [xp, xp]
                tmp = R1**2 - (xp - xc1)**2
                if tmp < 0 : return False, 0, 0
                yp1 = [yc1 + math.sqrt(tmp), yc1 - math.sqrt(tmp)]
            else:
                A = (x0 - xc1) / (yc1 - y0)
                B = (L1**2 - 2 * R1**2 + xc1**2 + yc1**2 - x0**2 - y0**2) / (2 * yc1 - 2 * y0)
                ap = 1 + A**2
                bp = 2 * (-A * yc1 + A * B - xc1)
                cp = xc1**2 + (B - yc1)**2 - R1**2
                successCode, xp10, xp11 = self.square_equation(ap, bp, cp)
                if not successCode: 
                    return False, 0, 0
                xp1 =[xp10, xp11]
                yp1 = [A * xp1[0] + B, A * xp1[1] + B]
        #alpha_1 = self.coord2yaw(x1 - xc1, y1 - yc1)
        al0 = self.coord2yaw(xp1[0] - xc1, yp1[0] - yc1)
        al1 = self.coord2yaw(xp1[1] - xc1, yp1[1] - yc1)
        if CW: da = -math.pi/2
        else: da = math.pi/2
        directToOtherEnd = self.coord2yaw(xc2 - xc1, yc2 - yc1)
        alpha_p1 = [abs(self.norm_yaw(da + al0 - directToOtherEnd)),
                    abs(self.norm_yaw(da + al1 - directToOtherEnd))]
        if start:
            ind = alpha_p1.index(min(alpha_p1))
        else: ind = alpha_p1.index(max(alpha_p1))
        return True, xp1[ind], yp1[ind]

    def arc_path_internal(self, x1, y1, yaw1, x2, y2, yaw2):
        number_Of_Cycles_min = 1000
        for i in range(10):
            for j in range(10):
                R1 = i * 0.05
                R2 = j * 0.05
                #R1 = 0.2
                #R2 = 0.2
                if (y2-y1) < 0:
                    xc1 = x1 + R1 * math.sin(yaw1)
                    yc1 = y1 - R1 * math.cos(yaw1)
                    xc2 = x2 - R2 * math.sin(yaw2)
                    yc2 = y2 + R2 * math.cos(yaw2)
                    CW1 = True
                    CW2 = False
                else:
                    xc1 = x1 - R1 * math.sin(yaw1)
                    yc1 = y1 + R1 * math.cos(yaw1)
                    xc2 = x2 + R2 * math.sin(yaw2)
                    yc2 = y2 - R2 * math.cos(yaw2)
                    CW1 = False
                    CW2 = True
                successCode, xp1, yp1 = self.internal_tangent_line(True, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW1)
                if successCode:
                    successCode, xp2, yp2 = self.internal_tangent_line(False, R2, R1, x2, y2, xc2, yc2, xc1, yc1, CW2)
                    if successCode:
                        dest = [[xp1,yp1], [xp2,yp2]]
                        centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1], [xp2, yp2, x2, y2, xc2, yc2, R2, CW2]]
                        nearestObstacle = self.check_Obstacle(xp1, yp1, xp2, yp2)
                        if nearestObstacle >= 0 :
                            roundAboutRadius = self.glob.obstacles[nearestObstacle][2] / 2 + roundAboutRadiusIncrement
                        #if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                        #                             self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                            for variant in range(2):
                                if variant == 0:
                                    CW = CW1
                                    successCode1, xp1, yp1 = self.external_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode2, xp2, yp2 = self.external_tangent_line(False,
                                           roundAboutRadius, R1, x2, y2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode3, xp3, yp3 = self.internal_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode4, xp4, yp4 = self.internal_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode1 and successCode2 and successCode3 and successCode4) : continue
                                if variant == 1:
                                    CW = not CW1
                                    successCode5, xp1, yp1 = self.internal_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode6, xp2, yp2 = self.internal_tangent_line(False,
                                           roundAboutRadius, R1, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode7, xp3, yp3 = self.external_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode8, xp4, yp4 = self.external_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode5 and successCode6 and successCode7 and successCode8) : continue
                                dest = [[xp1,yp1], [xp2,yp2], [xp3,yp3], [xp4,yp4]]
                                centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1],
                                            [xp2, yp2, xp3, yp3, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], roundAboutRadius, CW],
                                            [xp4, yp4, x2, y2, xc2, yc2, R2, CW2]]
                                price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                                number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                                if number_Of_Cycles < number_Of_Cycles_min:
                                    number_Of_Cycles_min = number_Of_Cycles
                                    dest_min = dest
                                    centers_min = centers
                        else:
                            price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                            number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                            if number_Of_Cycles < number_Of_Cycles_min:
                                number_Of_Cycles_min = number_Of_Cycles
                                dest_min = dest
                                centers_min = centers
        if number_Of_Cycles_min == 1000: return [], [], number_Of_Cycles_min
        else: return dest_min, centers_min, number_Of_Cycles_min

    def internal_tangent_line(self, start, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW ):
        if R1 == 0: return True, x1, y1
        L = math.sqrt((xc2 - xc1)**2 + (yc2 - yc1)**2)
        L1 = L * R1/(R1 +R2)
        x3 = xc1 + (xc2 - xc1) * R1 / (R1 + R2)
        y3 = yc1 + (yc2 - yc1) * R1 / (R1 + R2)
        if round(y3, 4) != round(yc1, 4):
            A = - (x3 - xc1) / (y3 - yc1)
            B = ( 2 * R1**2 - L1**2 - xc1**2 + x3**2 - yc1**2 + y3**2)/ 2 /(y3 - yc1)
            a = 1 + A**2
            b = 2 * A *(B - yc1) - 2 * xc1
            c = xc1**2 + (B - yc1)**2 - R1**2
            succsessCode, xp10, xp11 = self.square_equation(a, b , c)
            if not succsessCode: 
                return False, 0, 0
            xp1 =[xp10, xp11]
            yp1 = [A * xp1[0] + B, A * xp1[1] + B]
        else:
            tmp1 = ( R1**2 - L1**2 - xc1**2 + x3**2 - yc1**2 + y3**2)/ 2 /(x3 - xc1)
            xp1 = [tmp1,tmp1]
            ttt = R1**2 - (tmp1 - xc1)**2
            if ttt < 0: return False, 0, 0
            tmp2 = math.sqrt(ttt)
            yp1 = [yc1 + tmp2, yc1 - tmp2]
        #alpha_1 = self.coord2yaw(x1 - xc1, y1 - yc1)
        al0 = self.coord2yaw(xp1[0] - xc1, yp1[0] - yc1)
        al1 = self.coord2yaw(xp1[1] - xc1, yp1[1] - yc1)
        if CW: da = -math.pi/2
        else: da = math.pi/2
        directToOtherEnd = self.coord2yaw(xc2 - xc1, yc2 - yc1)
        alpha_p1 = [abs(self.norm_yaw(da + al0 - directToOtherEnd)),
                    abs(self.norm_yaw(da + al1 - directToOtherEnd))]
        if start:
            ind = alpha_p1.index(min(alpha_p1))
        else: ind = alpha_p1.index(max(alpha_p1))
        return True, xp1[ind], yp1[ind]


    def square_equation(self, a,b,c):
        D = b**2 - 4 * a * c
        if D < 0: return False, 0, 0
        return True, (-b + math.sqrt(D))/(2 * a), (-b - math.sqrt(D))/(2 * a)

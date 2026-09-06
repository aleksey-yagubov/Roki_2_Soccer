
import os, json, array
from random import getrandbits

import math
from math import pi
import json
import time

SDVIG = 32768


def weight_calculation( n, weights, observations, landmarks, gauss_noise, line_gauss_noise, p):

    def gaussian( x, sigma):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def new_observation_score( i, observations, landmarks, gauss_noise, p):
        # particle weight calculation
        prob = 1.0
        part0 = (p[i * 4] - SDVIG)/ 1000
        part1 = (p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (p[i * 4 + 2] - SDVIG)/ 1000
        sin_p = math.sin(part2) 
        cos_p = math.cos(part2)
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            for landmark in landmarks[color_landmarks]:
                min_dist = 1000
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts = part0 + observation[0] * cos_p - observation[1] * sin_p
                        y_posts = part1 + observation[0] * sin_p + observation[1] * cos_p
                        dist =(x_posts - landmark[0])**2 + (y_posts - landmark[1])**2
                        if min_dist > dist:
                            min_dist = dist
                            weight = observation[2]
                if min_dist != 1000:
                    prob *= gaussian(math.sqrt(min_dist), gauss_noise)*weight
        return prob
    def new_calc_lines_score( i, lines, landmarks, line_gauss_noise, p):
        '''
        line = (ro, theta)
        '''
        part0 = (p[i * 4] - SDVIG)/ 1000
        part1 = (p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (p[i * 4 + 2] - SDVIG)/ 1000
        prob = 1
        if lines != []:
            for line in lines:
                min_dist = 1000
                for landmark_line in landmarks:
                    for coord in landmarks[landmark_line]:
                        yaw = (part2 + line[1])%(2*math.pi)
                        if landmark_line == 'x':
                            dist = math.fabs(coord - (part0 + line[0]*math.cos(yaw)))
                        else:
                            dist = math.fabs(coord - (part1 + line[0]*math.sin(yaw)))
                        if min_dist > dist:
                            min_dist = dist
                if min_dist != 1000:
                    prob *= gaussian(min_dist, line_gauss_noise)*line[2]
        return prob
    # тело функции
    S = 0.0
    for i in range(n):
        weight = int(new_observation_score(i,observations, landmarks, gauss_noise, p)
                            *new_calc_lines_score(i,observations['lines'], landmarks['lines'], line_gauss_noise, p)
                            * 20000)
        weights[i] = weight
        S += weight
    return weights, S

def randrange( start, stop=None):
#helper function for working with random bit sequence
    if stop is None:
        stop = start
        start = 0
    upper = stop - start
    bits = 0
    pwr2 = 1
    while upper > pwr2:
        pwr2 <<= 1
        bits += 1
    while True:
        r = getrandbits(bits)
        if r < upper:
            break
    return r + start

def random():
    #getting a random number from 0 to 1
    return randrange(10000) / 10000

def gauss( mu, sigma):
    #getting a random number from Gaussian distribution
    x2pi = random() * math.pi * 2
    g2rad = math.sqrt(-2.0 * math.log(1.0 - random()))
    z = math.cos(x2pi) * g2rad
    return mu + z * sigma

def gaussian( x, sigma):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))


class ParticleFilter():
    def __init__(self, myrobot, landmarks_filename, particles_number, current_work_directory, global_vars):
        self.counter1 = 0
        self.counter2 = 0
        self.n = particles_number
        self.myrobot = myrobot
        self.count = 0
        self.p = array.array('I',(0 for i in range(self.n*4)))
        self.tmp = array.array('I',(0 for i in range(self.n*4)))
        self.weights = array.array('I',(0 for i in range(self.n)))
        self.new_p = array.array('I',(0 for i in range(self.n)))
        with open(landmarks_filename, "r") as f:
            self.landmarks = json.loads(f.read())
        self.landmark_lines_x = array.array('f',self.landmarks['lines']['x'])
        self.landmark_lines_y = array.array('f',self.landmarks['lines']['y'])
        self.land_keys =list(self.landmarks.keys())
        self.land_keys.pop(self.land_keys.index('lines'))
        self.land_keys.pop(self.land_keys.index('FIELD_WIDTH'))
        self.land_keys.pop(self.land_keys.index('FIELD_LENGTH'))
        self.array_landmarks = []
        for i in range(len(self.land_keys)):
            marks = array.array('f',[])
            for j in range(len(self.landmarks[self.land_keys[i]])):
                mark1 = array.array('f',self.landmarks[self.land_keys[i]][j])
                marks.extend(mark1)
            self.array_landmarks.append(marks)
        with open(current_work_directory + "Soccer/Localisation/PF/pf_constants.json", 'r') as constants:
            constants = json.load(constants)
        self.forward_noise = constants['noise']['forward_noise']
        self.turn_noise = constants['noise']['turn_noise']
        self.sense_noise = constants['noise']['sense_noise']
        self.gauss_noise = constants['noise']['gauss_noise']
        self.yaw_noise = constants['noise']['yaw_noise']
        self.line_gauss_noise = constants['noise']['line_gauss_noise']
        self.other_coord_noise = constants['noise']['other_coord_noise']

        self.number_of_res = constants['consistency']['number_of_res']
        self.consistency = constants['consistency']['consistency']
        self.goodObsGain = constants['consistency']['goodObsGain']
        self.badObsCost = constants['consistency']['badObsCost']
        self.stepCost = constants['consistency']['stepCost']
        self.dist_threshold = constants['consistency']['dist_threshold']
        self.con_threshold = constants['consistency']['con_threshold']
        self.spec_threshold = constants['consistency']['spec_threshold']
        self.gen_particles()
        #self.unsorted_posts = unsorted_posts
        #self.post1 = post1
        self.unsorted_posts, self.post1, self.post2, self.post3, self.post4, self.lines, self.Line_crosses, self.penalty, self.coord_for_PF_global = global_vars

    def norm_yaw(self, yaw):
        yaw %= 2 * pi
        if yaw > pi:  yaw -= 2* pi
        if yaw < -pi: yaw += 2* pi
        return yaw

    def gen_particles(self):
        for i in range(self.n):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG
        self.limit_paricles_coord(self.p, self.n)
        self.count += 1

    def gen_n_particles_robot(self, start_row):
        for i in range(start_row, self.n, 1):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG


    def gen_n_particles_robot_coord(self, start_row, coord):
        for i in range(start_row, self.n, 1):
            x_coord = coord[0] + gauss(0, self.other_coord_noise)
            y_coord = coord[1] + gauss(0, self.other_coord_noise)
            yaw = coord[2] + gauss(0, self.yaw_noise)*2
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG

    def update_consistency(self, observations):
        stepConsistency = 0
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            if len(observations[color_landmarks]) != 0:
                for observation in observations[color_landmarks]:
                    dists = []
                    for landmark in self.landmarks[color_landmarks]:

                        # calc posts coords in field for every mesurement
                        x_posts = (self.myrobot.x + observation[0]*math.cos(self.myrobot.yaw)
                                   - observation[1]*math.sin(self.myrobot.yaw))
                        y_posts = (self.myrobot.y + observation[0]*math.sin(self.myrobot.yaw)
                                   + observation[1]*math.cos(self.myrobot.yaw))
                        #print('x_posts, y_posts', x_posts, y_posts)
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                        #print('dist, len =', dist, len(dists))
                    if min(dists) < self.dist_threshold:
                        stepConsistency += self.goodObsGain

                        #print('good step', stepConsistency)
                    else:
                        stepConsistency -= self.badObsCost
                        #print('bad step', stepConsistency)
            else:
                stepConsistency -= self.stepCost
        #print('step cons', stepConsistency)
        self.consistency += stepConsistency
        if self.consistency > self.spec_threshold:
            self.consistency = self.spec_threshold
        elif self.consistency < 0.0:
            self.consistency = 0.0
        print('consistency', self.consistency)

    def particles_move(self, coord):
        self.myrobot.move(coord['shift_x'],
                          coord['shift_y'], coord['shift_yaw'])
        for i in range(self.n):
            orientation = (self.p[i * 4 + 2] - SDVIG) / 1000 + float(coord['shift_yaw'])
            if orientation < 0:
                orientation += (math.pi*2)
            orientation %= (2 * math.pi)
            x = self.p[i * 4]
            y = self.p[i * 4 + 1]
            yaw = (self.p[i * 4 + 2] - SDVIG) / 1000
            x1 = int(x + (coord['shift_x'] * math.cos(yaw) - coord['shift_y'] * math.sin(yaw)) * 1000)
            y1 = int(y + (coord['shift_x'] * math.sin(yaw) + coord['shift_y'] * math.cos(yaw)) * 1000)
            yaw1 = int(orientation * 1000 + SDVIG)
            self.p[i * 4] = x1
            self.p[i * 4 + 1] = y1
            self.p[i * 4 + 2] = yaw1
        self.count += 1
        self.limit_paricles_coord(self.p, self.n)

    def observation_to_predict(self, observations):
        predicts = []
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            if ((color_landmarks == 'angle') or (color_landmarks == 'lines')):
                continue
            for landmark in self.landmarks[color_landmarks]:
                if len(observations[color_landmarks]) != 0:
                    for obs in observations[color_landmarks]:
                        y_posts = self.myrobot.x + \
                            obs[0]*math.sin(-self.myrobot.yaw) + \
                            obs[1]*math.cos(-self.myrobot.yaw)
                        x_posts = self.myrobot.y + \
                            obs[0]*math.cos(-self.myrobot.yaw) - \
                            obs[1]*math.sin(-self.myrobot.yaw)
                        predicts.append([x_posts, y_posts])
        return predicts

    def limit_paricles_coord(self, p, stop_row):
        half_w = self.landmarks['FIELD_WIDTH'] / 2
        half_l = self.landmarks['FIELD_LENGTH'] / 2
        for i in range(stop_row):
            x = (p[i * 4] - SDVIG) / 1000
            y = (p[i * 4 + 1] - SDVIG) / 1000
            if math.fabs(x) > half_l + 0.3:
                p[i * 4] = int(math.copysign(half_l + 0.3, x) * 1000 + SDVIG)
            if math.fabs(y) > half_w + 0.3:
                p[i * 4 + 1] = int(math.copysign(half_w + 0.3, y) * 1000 + SDVIG)

    def resampling_wheel(self):
        row = 0
        for i in range(self.n): self.new_p[i] = 0
        index = int(random() * self.n)
        beta = 0.0
        mw = 0
        for i in range(self.n):
            m = self.weights[i]
            if m > mw : mw = m
        for i in range(self.n):
            beta += random() * 2.0 * mw
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.n
            if self.new_p[index] == 0:  self.new_p[index] = 1
            else: 
                self.new_p[index] += 1
        for el in range(self.n):
            if self.new_p[el] == 0: continue
            self.tmp[row*4:row*4+3] = self.p[el*4:el*4+3]
            self.tmp[row *4 + 3] = self.weights[el] * self.new_p[el]
            row += 1
        return row

    def resampling(self, observations, other_coord):
        for i in range(self.n): 
            self.tmp[i *4] = 0
            self.tmp[i *4 + 1] = 0
            self.tmp[i *4 + 2] = 0
            self.tmp[i *4 + 3] = 0
            self.weights[i] = 0 
        
        S = self.weight_calc_wrap(observations)
        if S == 0: return
        S = S / 20000
        for i in range(self.n):
            w = int(self.weights[i]/S)
            self.weights[i] = w
        row = self.resampling_wheel()
        S = 0.0
        for i in range(row):
            S += (self.tmp[i *4 + 3] / 20000)
        for i in range(row):
            tempo = int(self.tmp[i * 4 + 3]/S)
            self.tmp[i * 4 + 3] = tempo
        if not other_coord or other_coord == [1000,1000,1000]:
            self.gen_n_particles_robot(row)
        else:
            self.gen_n_particles_robot_coord(row, other_coord)
        self.p[:row *4] = self.tmp[:row *4]
        self.limit_paricles_coord(self.p, self.n)
        self.update_coord(self.p, row)
        self.count += 1
        self.update_consistency(observations)

    def weight_calc_wrap(self, observations):
        landmarks = self.landmarks
        n = self.n
        p = self.p
        weights = self.weights
        gauss_noise = self.gauss_noise
        line_gauss_noise = self.line_gauss_noise
        weights, S = weight_calculation(n, weights, observations, landmarks, gauss_noise, line_gauss_noise, p)
        self.weights = weights
        return S

    #def custom_reset(self, x, y, yaw):
    #    self.myrobot.x = x
    #    self.myrobot.y = y
    #    self.myrobot.yaw = yaw
    #    self.p = gen_n_particles_robot(0)

    # ------------------------------
    # need to add to handle the fall
    # ------------------------------

    def fall_reset(self, noise = 0.3):
        self.myrobot.x += gauss(0, self.sense_noise)
        self.myrobot.y += gauss(0, self.sense_noise)
        self.myrobot.yaw += gauss(0, self.yaw_noise)
        self.myrobot.yaw = self.norm_yaw(self.myrobot.yaw)
        for i in range(self.n):
            x1 = self.p[i * 4] + int(gauss(0, noise) * 1000)
            y1 = self.p[i * 4 + 1] + int(gauss(0, noise) * 1000)
            yaw1 = self.p[i * 4 + 2] + int(gauss(0, noise / 3) * 1000)
            self.p[i * 4] = x1
            self.p[i * 4 +1] = y1
            self.p[i * 4 + 2] = yaw1
        self.consistency *= 0.5

    def update_coord(self, particles, stop_row):
        x = 0.0
        y = 0.0
        orientation = 0.0
        adding = 0.0
        if (self.myrobot.yaw < math.pi/2) or (self.myrobot.yaw > math.pi*3/2):
            adding = math.pi*2
        for i in range(stop_row):
            x_old = (particles[i * 4]- SDVIG) / 1000
            y_old = (particles[i * 4 + 1]- SDVIG) / 1000
            yaw_old = (particles[i * 4 + 2]- SDVIG) / 1000
            w_old = particles[i * 4 + 3] / 20000
            x += x_old * w_old
            y += y_old * w_old
            if (yaw_old < math.pi):
                culc_yaw = yaw_old + adding
            else:
                culc_yaw = yaw_old
            orientation += culc_yaw * w_old
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = self.norm_yaw(orientation)

    def return_coord(self):
        return self.myrobot.x, self.myrobot.y, self.myrobot.yaw

    def updatePF(self, measurement, other_coord):
        observations = {}
        for key in self.land_keys + ['lines'] :
            glob_array = eval('self.'+ key)
            element_list = []
            for j in range(int(glob_array[0])):
                element = [glob_array[j * 3 + 1], glob_array[j * 3 + 2], glob_array[j * 3 + 3]]
                element_list.append(element)
            glob_array[0] = 0
            observations.update({key:element_list})
        #self.coord_for_PF_global[:] = self.coord_for_PF_global
        for i in range(self.number_of_res):
            #self.resampling(measurement, other_coord)
            self.resampling(observations, self.coord_for_PF_global[:])
        return self.return_coord()

class Agent:

    def __init__(self, x=0, y=0, yaw=0):
        self.x = x          # agent's x coordinate
        self.y = y          # agent's y coordinate
        self.yaw = yaw      # agent's angle

    def move(self, x, y, yaw):
        # turn, and add randomomness to the turning command
        orientation = self.yaw + float(yaw)
        if orientation < 0:
            orientation += (math.pi*2)
        orientation %= (2 * math.pi)
        self.x += x*math.cos(self.yaw) - y*math.sin(self.yaw)
        self.y += x*math.sin(self.yaw) + y*math.cos(self.yaw)
        self.yaw = orientation

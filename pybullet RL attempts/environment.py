import time
import numpy as np
import gymnasium as gym
#import tkinter as tk
#from PIL import ImageTk, Image
from scipy.stats import norm
import pybullet as p
import pybullet_data
from urdfpy import URDF
import os
from scipy.stats import norm
import matplotlib.pyplot as plt

np.random.seed(1)
#PhotoImage = ImageTk.PhotoImage
UNIT = 50  # pixels
HEIGHT = 10  # grid height
WIDTH = 20  # grid width

grid_dx=8
grid_dy=17

anti_exploitation = np.ones((WIDTH, HEIGHT))

step_counter=0

def Get_distance_between_objects(a, b, max_dist):
    Distance = p.getClosestPoints(bodyA=a, bodyB=b,distance=max_dist)
    #print(Distance)
    #dist = Distance[0][8]
    dist=Distance[0][8]
    return dist

def Punishment(min_dist):
    stick=0
    for i in range(1, 35, 1):
        #print(i)
        dist = Get_distance_between_objects(36, i, 100)
        #utility = k/dist
        if dist<0.01:
            delta_stick = 1
            #print("Crash!")
        elif dist <= min_dist:
            delta_stick = 0.1
        elif dist > min_dist:
            delta_stick = 0
        else:
            print("Error")
        stick = stick + delta_stick
    return stick

def prob(upper, lower):
    cdf_upper_limit = norm(loc = 0 , scale = 1).cdf(upper)
    cdf_lower_limit = norm(loc = 0 , scale = 1).cdf(lower)
 
    p = cdf_upper_limit - cdf_lower_limit
    return p

def probability_grid(x, y, sigmas_covered):
    arr = np.zeros((x+2, y+2))#the plus one is to pad the array with zeros
    sum=0
    for i in range(x):
        for j in range(y):
            u_x = -sigmas_covered+(2*(sigmas_covered)/x)*(i+1)
            l_x = -sigmas_covered+(2*(sigmas_covered)/x)*(i)
            prob_x = prob(u_x, l_x)
            
            u_y = -sigmas_covered+(2*(sigmas_covered)/y)*(j+1)
            l_y = -sigmas_covered+(2*(sigmas_covered)/y)*(j)
            prob_y = prob(u_y, l_y)
            
            p_box = prob_x*prob_y
            sum = sum + p_box
            arr[i, j] = p_box
    return arr, sum

class Env(gym.Env):
    
    
    def __init__(self):
        super(Env, self).__init__()
        self.action_space = ['N', 'S', 'W', 'E']
        self.n_actions = len(self.action_space)
        #self.title('Q Learning')
        p.connect(p.GUI) # p.direct for no visualisation, faster
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #p.setAdditionalSearchPath("Users\User\Documents\Python Scripts")
        #p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        #self.geometry('{0}x{1}'.format(HEIGHT * UNIT, HEIGHT * UNIT))
        #self.shapes = self.load_images()
        #self.canvas = self._build_canvas()
        self._build_canvas()
        #self.texts = []
        global targid
        targid = p.loadURDF (os.path.join ("pybullet RL attempts/urdfs/cf2x.urdf"),[9.5, -9.5, 3], [0, 0, 0, 1])#, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        #print("targid: ", p.getBodyUniqueId(targid), ":::", p.getCollisionShapeData(1, -1))
        p.resetDebugVisualizerCamera(cameraDistance=11, cameraYaw=0, cameraPitch=-89, cameraTargetPosition = [10, -8, 0])

        

    def _build_canvas(self):
        #load assets
        floor = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1]) # location, orientaton as quarternion
        #targid = URDF.load("gym-pybullet-drones/gym_pybullet_drones/assets/cf2x.urdf")
        #global targid
        #targid = p.loadURDF (os.path.join ("pybullet RL attempts/urdfs/cf2x.urdf"),[10, -10, 3], [0, 0, 0, 1])#, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        #print("targid: ", p.getBodyUniqueId(targid), ":::", p.getCollisionShapeData(1, -1))
        #box1 = p.loadURDF(os.path.join ("gym-pybullet-drones/gym_pybullet_drones/assets/box.urdf"),[0, 1, 1], [0, 0, 0, 1], useFixedBase=True)

        #Border
        wall1 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall1.urdf"),[1.55, -5.95, 0],  p.getQuaternionFromEuler([0, 0, 1.571]), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        #print("Wall1: ", p.getBodyUniqueId(wall1), ":::", p.getCollisionShapeData(2, -1))
        wall2 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall2.urdf"),[10.5, -1.37, 0],  p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        #print("Wall2: ", p.getBodyUniqueId(wall2), ":::", p.getCollisionShapeData(3, -1))
        wall3 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall3.urdf"),[19.47, -5.91, 0],  p.getQuaternionFromEuler([0, 0, 1.571]), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        wall4 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall4.urdf"),[10.54, -10.45, 0],  p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        
        #Fan Fawr
        wall5 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall5.urdf"),[4.53, -7.78, 0],  p.getQuaternionFromEuler([0, 0, 0.743]), useFixedBase=True)
        wall6 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall6.urdf"),[5.02, -7.73, 0],  p.getQuaternionFromEuler([0, 0, 1.525]), useFixedBase=True)
        wall7 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall7.urdf"),[4.56, -8.18, 0],  p.getQuaternionFromEuler([0, 0, 0.091]), useFixedBase=True)

        #Craig-Gwaun-Taf
        wall8 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall8.urdf"),[8.38, -6.4, 0],  p.getQuaternionFromEuler([0, 0, 1.409]), useFixedBase=True)
        wall9 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall9.urdf"),[8.95, -4.84, 0],  p.getQuaternionFromEuler([0, 0, 0.041]), useFixedBase=True)
        wall10 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall10.urdf"),[9.74, -4.37, 0],  p.getQuaternionFromEuler([0, 0, 0.775]), useFixedBase=True)
        wall11 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall11.urdf"),[10.21, -4.33, 0],  p.getQuaternionFromEuler([0, 0, 1.571]), useFixedBase=True)
        wall12 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall12.urdf"),[10.41, -5.03, 0],  p.getQuaternionFromEuler([0, 0, -0.887]), useFixedBase=True)
        wall13 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall13.urdf"),[10.15, -5.49, 0],  p.getQuaternionFromEuler([0, 0, 0.341]), useFixedBase=True)
        wall14 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall14.urdf"),[10.18, -6.96, 0],  p.getQuaternionFromEuler([0, 0, -1.137]), useFixedBase=True)
        wall15 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall15.urdf"),[10.25, -8.27, 0],  p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        wall16 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall16.urdf"),[9.83, -7.83, 0],  p.getQuaternionFromEuler([0, 0, 1.347]), useFixedBase=True)
        wall17 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall17.urdf"),[9.71, -7.12, 0],  p.getQuaternionFromEuler([0, 0, -0.908]), useFixedBase=True)
        wall18 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall18.urdf"),[9.41, -7.03, 0],  p.getQuaternionFromEuler([0, 0, 1.403]), useFixedBase=True)
        wall19 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall19.urdf"),[8.74, -7.63, 0],  p.getQuaternionFromEuler([0, 0, 0.521]), useFixedBase=True)

        #Cribyn
        wall20 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall20.urdf"),[11.23, -5.13, 0],  p.getQuaternionFromEuler([0, 0, 0.212]), useFixedBase=True)
        wall21 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall21.urdf"),[11.59, -5.56, 0],  p.getQuaternionFromEuler([0, 0, -1.352]), useFixedBase=True)
        wall22 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall22.urdf"),[11.34, -5.65, 0],  p.getQuaternionFromEuler([0, 0, -0.834]), useFixedBase=True)

        #Gwaun Cerrig Llwydion
        wall23 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall23.urdf"),[14.08, -6.56, 0],  p.getQuaternionFromEuler([0, 0, 0.949]), useFixedBase=True)
        wall24 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall24.urdf"),[14.72, -6.1, 0],  p.getQuaternionFromEuler([0, 0, 0.03]), useFixedBase=True)
        wall25 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall25.urdf"),[15.35, -5.68, 0],  p.getQuaternionFromEuler([0, 0, 0.915]), useFixedBase=True)
        wall26 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall26.urdf"),[15.9, -5.33, 0],  p.getQuaternionFromEuler([0, 0, -0.441]), useFixedBase=True)
        wall27 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall27.urdf"),[16.24, -5.81, 0],  p.getQuaternionFromEuler([0, 0, -1.305]), useFixedBase=True)
        wall28 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall28.urdf"),[16.66, -6.21, 0],  p.getQuaternionFromEuler([0, 0, -0.184]), useFixedBase=True)
        wall29 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall29.urdf"),[16.97, -6.75, 0],  p.getQuaternionFromEuler([0, 0, 1.505]), useFixedBase=True)
        wall30 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall30.urdf"),[16.63, -7.33, 0],  p.getQuaternionFromEuler([0, 0, 0.156]), useFixedBase=True)
        wall31 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall31.urdf"),[15.87, -6.82, 0],  p.getQuaternionFromEuler([0, 0, -0.891]), useFixedBase=True)
        wall32 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall32.urdf"),[15.01, -6.77, 0],  p.getQuaternionFromEuler([0, 0, 0.903]), useFixedBase=True)
        wall33 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall33.urdf"),[13.94, -7.41, 0],  p.getQuaternionFromEuler([0, 0, 0.175]), useFixedBase=True)
        wall34 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall34.urdf"),[13.19, -7.28, 0],  p.getQuaternionFromEuler([0, 0, -1.1]), useFixedBase=True)
        wall35 = p.loadURDF(os.path.join ("pybullet RL attempts/urdfs/Brecon/Wall35.urdf"),[13.42, -7.05, 0],  p.getQuaternionFromEuler([0, 0, 0.015]), useFixedBase=True)
        


    #load images was here

    #text was here

    def coords_to_state(self):
        global targid
        position , _ =p.getBasePositionAndOrientation(targid)
        x_pos=int(position[0])
        y_pos=int(position[1])
        x = x_pos-3
        y = y_pos+9
        #x = int((coords[0] - 25) / 50)
        #y = int((coords[1] - 25) / 50)
        return [x, y]

    def state_to_coords(self):
        global targid
        position , _ =p.getBasePositionAndOrientation(targid)
        x=int(position[0])
        y=int(position[1])
        return [x, y]

    def reset(self):
        global anti_exploitation
        anti_exploitation = np.ones((HEIGHT, WIDTH))
        global reward_sum
        reward_sum=0
        global reward
        reward=0
        global g
        global targid
        #self.update()
        time.sleep(0.5)
        p.resetBasePositionAndOrientation(p.getBodyUniqueId(targid), [10, -10, 3], [0, 0, 0, 1])
        #print(p.getBodyUniqueId(targid))
        g, sum = probability_grid(grid_dy, grid_dx, 2)
        


    def step(self, action):
        global reward
        global reward_sum
        global anti_exploitation
        global targid
        global g
        #state, _ = int(p.getBasePositionAndOrientation(targid))+[-3, 9]
        #base_action = np.array([0, 0])
        #self.render()

        if action == 0:  # up
            p.resetBaseVelocity(targid, [0, 10, 0])
        elif action == 1:  # down
            p.resetBaseVelocity(targid, [0, -10, 0])
        elif action == 2:  # left
            p.resetBaseVelocity(targid, [10, 0, 0])
        elif action == 3:  # right
            p.resetBaseVelocity(targid, [-10, 0, 0])

        # move agent
        #self.canvas.move(self.rectangle, base_action[0], base_action[1])
        for i in range(24):
            #time.sleep(0.01)
            p.stepSimulation()
            focus_position , _ =p.getBasePositionAndOrientation(targid)#returns position and oreintation hence weird syntax at start
            p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-45, cameraTargetPosition = focus_position)
        
        #time.sleep(0.01)
        # move rectangle to top level of canvas
        #self.canvas.tag_raise(self.rectangle)
        next_state, _ = p.getBasePositionAndOrientation(targid)
        next_state = [int(next_state[0])-3, int(next_state[1])+9]
        #print(next_state)

        # reward function
        position , _ =p.getBasePositionAndOrientation(targid)
        x_pos=int(position[0])
        y_pos=int(position[1])
        X_pos = x_pos-3
        Y_pos = y_pos+9
        reward = g[X_pos, Y_pos]*100
        #print(X_pos, Y_pos, g[X_pos, Y_pos])
        g[X_pos, Y_pos] = 0
        #print(g)
        reward = reward - 10*Punishment(0.1)
        reward_sum=reward_sum+reward
        #print("reward:", reward,"   reward_sum:", reward_sum, "   Positive only:", g[X_pos, Y_pos]*100,   "    Punishment", 10*Punishment(0.1))
        if reward_sum<-10:
            done=True
            
        else:
            done=False

        #next_state = self.coords_to_state(next_state)
        next_state = self.coords_to_state()
        return next_state, reward, done, reward_sum

    def render(self):
        time.sleep(0.001)
        #p.stepSimulation()

    def q_table(self):
        print(self.q_table)
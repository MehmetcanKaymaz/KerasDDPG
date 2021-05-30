from utils import Utils
from controller import Controller
from nonlinear_model import NonlinearModel
import random
import math
import numpy as np

u = Utils()
controller = Controller()


class Env:
    def __init__(self):
        self.info = ""
        self.dtau = 0.01
        self.dt = 0.0001
        self.Nsolver = int(self.dtau / self.dt)
        self.states_arrr=[]
        self.ref_action=[]

        self.obs_space = 3
        self.action_space = 3
        self.action_space_max = 10
        self.action_space_min = -10
        self.obs=[0,0,0,0,0,0]
        self.d=500
        self.target=self.__random_target()
        self.states = self.__initial_states()
        self.obs_states = self.__obs_calc()

        self.index = 0
        self.errsum=[0,0,0,0]
        self.lasterr=[0,0,0,0]
        self.ref_att=[0,0,0,0]

    def reset(self):
        self.target=self.__random_target()
        self.states = self.__initial_states()
        self.obs_states = self.__obs_calc()
        self.index = 0
        self.errsum=[0,0,0,0]
        self.lasterr=[0,0,0,0]
        return self.obs_states

    def step(self, action):
        self.states_arrr.append(self.obs_states)     
        next_states = self.__make_action(action)  
        self.ref_action.append(self.ref_att)   
        self.states = next_states
        self.obs_states = self.__obs_calc()
        done = self.__done_calc()
        reward = self.__reward_calc()
        self.index+=1

        return self.obs_states, reward, done,{}

    def __initial_states(self):
        return [u.feetTometer(176), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4500.0, 0.0, 0.0, 0.0]

    def __obs_calc(self):
        pose=self.__pos_state()
        for i in range(3):
            self.obs[i]=self.target[i]-pose[i]
        phi = u.radTodeg(self.states[3])
        theta = u.radTodeg(self.states[4])
        psi = u.radTodeg(self.states[5])
        self.obs[3]=phi
        self.obs[4]=theta
        self.obs[5]=psi
        return self.obs

    def __make_action(self, actions):
        new_states = self.states  
        actions=self.__action_converter(actions)
        #self.ref_att=controller.PositionController(new_states, self.target)
        data = controller.PIDController(new_states, actions,self.errsum,self.lasterr,self.dtau)  # self.errsum,self.lasterr,self.dtau
        U=data[0]
        self.errsum=data[1]
        self.lasterr=data[2]
        new_states = NonlinearModel(new_states, U, self.Nsolver, self.dt)
        return new_states

    def __done_calc(self):
        donef = False

        x_err=self.obs[0]
        y_err=self.obs[1]
        z_err=self.obs[2]
        
        self.d=u.sqrt(pow(x_err,2)+pow(y_err,2)+pow(z_err,2))
        
        if self.d<20:
            donef=True
        
        phi = u.radTodeg(self.states[3])
        theta = u.radTodeg(self.states[4])
        psi = u.radTodeg(self.states[5])
        
        if abs(phi)>60 or abs(theta)>60 or abs(psi)>60:
            donef=True
        
        if self.index == 1000:
            donef = True
        return donef

    def __reward_calc(self):
        reward=-self.d
        return reward
    
    def __random_target(self):
        xd=random.uniform(400,500)
        yd=random.uniform(-200,200)
        zd=random.uniform(-20,20)
        return [xd,yd,zd]
    
    def __pos_state(self):
        x=self.states[10]
        y=self.states[11]
        z=self.states[12]
        return [x,y,z]
    
    def __action_converter(self,action):
        actions=[0,0,0,0]
        actions[0]=action[0]*30*math.pi/180
        actions[1]=action[1]*30*math.pi/180
        actions[2]=action[2]*30*math.pi/180
        actions[3]=65+action[3]*15
        return actions
    def save_data(self):
        state_arr=np.array(self.states_arrr)
        action_arr=np.array(self.ref_action)
        #np.savetxt("state_data.txt",state_arr)
        np.savetxt("action_data.txt",action_arr)    

